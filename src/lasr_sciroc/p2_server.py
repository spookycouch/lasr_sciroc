#!/usr/bin/python

import yaml
from os import path

import rospy
import actionlib
import rosparam
import rosnode
import tf.transformations
# Actionlib messages
import lasr_pnp_bridge.msg as lpb_msg
from std_msgs.msg import String, Header
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Quaternion, Pose, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from lasr_object_detection_yolo.msg import yolo_detectionGoal, yolo_detectionAction
from pal_interaction_msgs.msg import TtsGoal, TtsAction
from sensor_msgs.msg import Image
from collections import defaultdict
from math import pi as PI
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
        self.object_recognition_client = actionlib.SimpleActionClient('/yolo_detection', yolo_detectionAction)
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
    
    def p2Initialise(self):
        # Get the Tables Dictionary from the parameter server
        self.tables = rospy.get_param("/tables")

        # Get the ID of a table that needs serving
        needServing_exist = False
        for table in self.tables:
            if (self.tables[table])['status'] == 'Needs serving':
                if not needServing_exist:
                    needServing_exist = True
                    next_table = self.tables[table]['id']
                elif self.tables[table]['id'] < next_table:
                    next_table = self.tables[table]['id']
        
        if needServing_exist:
            rospy.set_param('/HAL9000/current_table', next_table)
            print "\033[1;33m" + "The next table that needs serving is " + str(next_table) + "\033[0m"
        else:
            # if all tables have been served, counting is done
            self._result.condition_event = ['doneServing']

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
        print(order)  
        rospy.set_param('/tables/table' + str(table_index) + '/order', order)
        rospy.loginfo('Updated the order of table %d successfully', table_index)
        rospy.sleep(5)

    
    def gotoBar(self):
        rospy.loginfo('Going to bar')
        bar = rospy.get_param('/Bar')

        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = Pose(position = Point(**bar['loc']['position']),
            orientation = Quaternion(**bar['loc']['orientation']))

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
        print(order)  

        # Fill the speech goal
        tts_goal.rawtext.text = 'The order of table {0} is {1}'.format(table_index, order)
        self.speech_client.send_goal(tts_goal)
        rospy.sleep(10)

    
    def checkOrder(self):
        table_index = rospy.get_param('/HAL9000/current_table')

        # Fetch the order from the parameter server
        order = rospy.get_param('/tables/table' + str(table_index) + '/order')
        rospy.loginfo('The order fetched from the parameter server in checkOrderCorrectness is ')
        print(order)  

        while True:
            rospy.sleep(5)
            # Look down to see the items on the counter
            # Wait for the play motion server to come up and send goal
            self.play_motion_client.wait_for_server(rospy.Duration(15.0))
            pose_goal = PlayMotionGoal()
            pose_goal.motion_name = "look_down"
            pose_goal.skip_planning = True
            self.play_motion_client.send_goal(pose_goal)
            rospy.loginfo('Looking down goal sent')
            rospy.sleep(5)

            # Run the object detection client on the items
            goal = yolo_detectionGoal
            goal.image_raw = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
            goal.dataset = "costa"
            goal.confidence = 0.3
            goal.nms = 0.5
            self.object_recognition_client.send_goal(goal)
            rospy.loginfo('Object check goal sent')
            self.object_recognition_client.wait_for_result()
            result = self.object_recognition_client.get_result()
            rospy.loginfo('Got the result back')

            order_count = defaultdict(int)
            for item in order:
                order_count[item] += 1
            
            object_count = defaultdict(int)
            for detection in result.detected_objects:
                object_count[detection.name] += 1
            for count in object_count:
                print('I see ' + str(object_count[count]) + ' of ' + str(count))

            # Look back up
            # Wait for the play motion server to come up and send goal
            self.play_motion_client.wait_for_server(rospy.Duration(15.0))
            pose_goal = PlayMotionGoal()
            pose_goal.motion_name = "back_to_default"
            pose_goal.skip_planning = True
            self.play_motion_client.send_goal(pose_goal)
            rospy.loginfo('Back to default goal sent')
            rospy.sleep(5)

            # Compare the result to the order and announce the missing item
            missing_items = defaultdict(int)
            excess_items = defaultdict(int)
            for item in order_count:
                if object_count[item] < order_count[item]:
                    missing_items[item] = order_count[item] - object_count[item]
            for item in object_count:
                if object_count[item] > order_count[item]:
                    excess_items[item] = object_count[item] - order_count[item]

            # Output incorrect items
            if len(missing_items) or len(excess_items):
                speech_out = 'The order is incorrect. Please correct the order.'
                if len(missing_items):
                    speech_out += '. The missing items are '
                    for item in missing_items:
                        speech_out += ', ' + str(missing_items[item]) + ' ' + str(item)
                        if not missing_items[item] == 1:
                            speech_out += 's'
                # output excess items
                if len(excess_items):
                    speech_out += '. The excess items are:'
                    for item in excess_items:
                        speech_out += ', ' +  str(excess_items[item]) + ' ' + str(item)
                        if not excess_items[item] == 1:
                            speech_out += 's'
                self.talk(speech_out)
            else:
                self.talk('Order is correct, please place the items on my back.')
                break
            rospy.sleep(4)

    def talk(self, speech_in):
        print('\033[1;36mTIAGO: ' + speech_in + '\033[0m')
        tts_goal = TtsGoal()
        tts_goal.rawtext.lang_id = 'en_GB'
        tts_goal.rawtext.text = speech_in
        self.speech_client.send_goal(tts_goal)

    def shiftQuaternion(self, orientation, radians):
        theEuler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        if(theEuler[2] > 0):
            newEuler = theEuler[0], theEuler[1], theEuler[2] - radians
        else:
            newEuler = theEuler[0], theEuler[1], theEuler[2] + radians
        theFakeQuaternion = tf.transformations.quaternion_from_euler(newEuler[0], newEuler[1], newEuler[2])
        theRealQuaternion = Quaternion(theFakeQuaternion[0], theFakeQuaternion[1], theFakeQuaternion[2], theFakeQuaternion[3])
        return theRealQuaternion
    
    def waitConfirmation(self):
        # RESTART MOVE BASE WITH LOWER YAW THRESHOLD
        rospy.loginfo('killing move base server')
        rospy.set_param("/move_base/PalLocalPlanner/yaw_goal_tolerance", 0.05)
        rosnode.kill_nodes(['/move_base'])
        if self.move_base_client.wait_for_server(rospy.Duration(10)):
            rospy.loginfo('Move base server is back up')
        else:
            rospy.loginfo("Failed to connect to move base")
        
        rospy.sleep(2)
        table_index = rospy.get_param('/HAL9000/current_table')

        # Get his current pose
        current_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped).pose.pose

        # Send new pose
        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        goal = MoveBaseGoal()

        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        # Change orientation to turn TIAGo 180 degrees
        goal.target_pose.pose = Pose(position=current_pose.position, orientation=self.shiftQuaternion(current_pose.orientation, PI))
        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) #waits forever
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")

        # Wait for keyword detection porcupine
        # nothing for now
        rospy.sleep(15)

        # Set the table to be already served
        rospy.set_param('/tables/table' + str(table_index) + '/status', 'Already served')
