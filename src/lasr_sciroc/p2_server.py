#!/usr/bin/python

import yaml
from os import path

import rospy
import actionlib
import rosparam

# Actionlib messages
import lasr_pnp_bridge.msg as lpb_msg
from std_msgs.msg import String, Header
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Quaternion, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from object_detection_yolo_opencv4.msg import count_objectsAction, count_objectsGoal
from pal_interaction_msgs.msg import TtsGoal, TtsAction
from sensor_msgs.msg import Image
from collections import defaultdict

class P2Server(object):
    _feedback = lpb_msg.BridgeFeedback()
    _result = lpb_msg.BridgeResult()

    def __init__(self, server_name):
        # bridge server
        self._bridge_server = actionlib.SimpleActionServer(server_name, lpb_msg.BridgeAction, execute_cb=self.execute_cb, auto_start=False)
        self._bridge_server.start()

        # Initialising clients: move_base, playmotion, objectRecognition and table_status
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        self.object_recognition_client = actionlib.SimpleActionClient('/count_objects', count_objectsAction)
        self.speech_client = actionlib.SimpleActionClient('/tts', TtsAction)

        # Bool variable and wake_word subscriber for voice plan activation
        self.running = False
        rospy.Subscriber('/wake_word/wake_word_detected', String, self.handle_wake_word_detected)
        self.plan_publisher = rospy.Publisher('/sciroc/planToExec', String, queue_size=1)
    
    def handle_wake_word_detected(self, data):
        wake_word = data.data
        if not self.running and wake_word == 'start the demo':
            self.running = True
            self.plan_publisher.publish('p1PlanNew')
  
    def execute_cb(self, goal):
        rospy.loginfo("----------ExternalServer start----------")

        # log action and parameters
        rospy.loginfo("action is: " + str(goal.action))
        rospy.loginfo("params are: " + str(goal.params))

        # reset result 
        self._result = lpb_msg.BridgeResult()

        # call the action
        getattr(self, goal.action)(*goal.params)

        self._bridge_server.set_succeeded(self._result)

        # end callback
        rospy.loginfo("----------ExternalServer end----------")

    def gotoHome(self):
        rospy.loginfo('Going to home')
        home = rospy.get_param('/Home')

        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = Pose(position = Point(**home['loc']['position']),
            orientation = Quaternion(**home['loc']['orientation']))

        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) #waits forever
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")

    def goto(self):
        table_index = rospy.get_param('/HAL9000/current_table')
        pose_index = rospy.get_param('/HAL9000/current_pose')
        print table_index
        # TODO: move to individual action file
        rospy.loginfo('Going to: %d with pose %d', table_index, pose_index)

        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        # For now
        if pose_index == 0:
            pose = 'far_pose'
        else:
            pose = 'close_pose'

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = Pose(position = Point(**self.tables['table' + str(table_index)]['loc'][pose_index][pose]['position']),
            orientation = Quaternion(**self.tables['table' + str(table_index)]['loc'][pose_index][pose]['orientation']))

        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) #waits forever
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")

        # If we moved close to the table means we have already taken the picture
        if pose_index == 1:
           self._result.condition_event = ['pictureDone']
           rospy.set_param('/HAL9000/current_pose', 0)

    
    def initialise_p2(self):
        # Get the ID of a table that needs serving
        for table in self.tables:
            if (self.tables[table])['status'] == 'Needs serving':
                rospy.set_param('/HAL9000/current_table', self.tables[table]['id'])
                break

        # Set the table pose to the close pose
        rospy.set_param('/HAL9000/current_pose', 1)

        # Get the updated Tables Dictionary from the parameter server
        self.tables = rospy.get_param("/tables")

    def gotoNeedServing(self):
        table_index = rospy.get_param('/HAL9000/current_table')
        pose_index = rospy.get_param('/HAL9000/current_pose')
        # Goto the table
        rospy.loginfo('Going to: %d with pose %d because it needs serving', table_index, pose_index)

        # For now
        if pose_index == 0:
            pose = 'far_pose'
        else:
            pose = 'close_pose'

        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = Pose(position = Point(**self.tables['table' + str(table_index)]['loc'][pose_index][pose]['position']),
            orientation = Quaternion(**self.tables['table' + str(table_index)]['loc'][pose_index][pose]['orientation']))

        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) #waits forever
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")

    
    def takeOrder(self):
        table_index = rospy.get_param('/HAL9000/current_table')
        pose_index = rospy.get_param('/HAL9000/current_pose')

        # For now we will just change the rosparam server as if the customer placed the order
        order = ['water', 'biscotti', 'berry smoothie']
        print('[INFO] Updating the order at table %d', table_index)
        print('[INFO] the order is ')
        print(*order, sep = ", ")  
        rospy.set_param('/tables/table' + str(table_index) + '/order', order)
        rospy.loginfo('Updated the order of table %d successfully', table_index)
        rospy.sleep(5)

    
    def gotoBar(self):
        rospy.loginfo('Going to bar')
        home = rospy.get_param('/Home')

        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = Pose(position = Point(**home['loc']['position']),
            orientation = Quaternion(**home['loc']['orientation']))

        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) #waits forever
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")

    
    def orderConfirm(self):
        table_index = rospy.get_param('/HAL9000/current_table')
        pose_index = rospy.get_param('/HAL9000/current_pose')

        # Create a tts goal
        tts_goal = TtsGoal()
        tts_goal.rawtext.lang_id = 'en_GB'

        # Fetch the order from the parameter server
        order = rospy.get_param('/tables/table' + str(table_index) + '/order')
        rospy.loginfo('The order fetched from the parameter server is ')
        print(*order, sep = ", ")  

        # Fill the speech goal
        tts_goal.rawtext.text = 'The order of table {0} is {1}'.format(table_index, order)
        self.speech_client.send_goal(tts_goal)
        rospy.sleep(10)

    
    def checkOrder(self):
        table_index = rospy.get_param('/HAL9000/current_table')

        # Fetch the order from the parameter server
        order = rospy.get_param('/tables/table' + str(table_index) + '/order')
        rospy.loginfo('The order fetched from the parameter server in checkOrderCorrectness is ')
        print(*order, sep = ", ")  

        # Look down to see the items on the counter
        # Wait for the play motion server to come up and send goal
        self.play_motion_client.wait_for_server(rospy.Duration(15.0))
        pose_goal = PlayMotionGoal()
        pose_goal.motion_name = "look_down"
        pose_goal.skip_planning = True
        self.play_motion_client.send_goal(pose_goal)
        rospy.loginfo('Looking down goal sent')
        rospy.sleep(3)

        # Run the object detection client on the items
        goal = lasr_object_detection_yolo.msg.yolo_detectionGoal()
        goal.image_raw = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        goal.dataset = "costa"
        goal.confidence = 0.3
        goal.nms = 0.5
        self.object_recognition_client.send_goal(goal)
        rospy.loginfo('Object check goal sent')
        client.wait_for_result()
        result = client.get_result()
        rospy.loginfo('Got the result back')

        order_count = defaultdict(int)
        for item in order:
            order_count[item] += 1
        
        # Say what we saw
        object_count = defaultdict(int)
        for detection in result.detected_objects:
            object_count[detection.name] += 1
        for count in object_count:
            print('I see ' + str(object_count[count]) + ' of ' + str(count))

        # Compare the result to the order and announce the missing item
        for item in object_count:
            if object_count[item] != order_count[item]:
                missing_item = item
                break
        if missing_item:
            rospy.loginfo('The missing item is %s', missing_item)

        # Say the missing item
        tts_goal = TtsGoal()
        tts_goal.rawtext.lang_id = 'en_GB'
        tts_goal.rawtext.text = 'The item {0} is wrong. Please correct the order and place it on my back'.format(missing_item)
        self.speech_client.send_goal(tts_goal)
        rospy.sleep(4)

    
    def waitConfirmation(self):
        # Get his current pose
        current_pose = rospy.wait_for_message('/amcl_pose', Pose)

        # Change orientation to turn TIAGo 180 degrees
        current_pose.orientation.w = -current_pose.orientation.w
        turn  = current_pose

        # Send new pose
        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = turn
        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) #waits forever
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")

        # Wait for keyword detection porcupine
        # nothing for now
        rospy.sleep(15)
