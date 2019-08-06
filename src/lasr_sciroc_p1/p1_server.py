#!/usr/bin/python

import yaml
from os import path

import rospy
import actionlib
import rosparam

# Our table class
from Table import Table

# Actionlib messages
import lasr_pnp_bridge.msg as lpb_msg
from std_msgs.msg import String, Header
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Quaternion, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from object_detection_yolo_opencv4.msg import count_objectsAction, count_objectsGoal
from table_status.msg import TableStatusAction, TableStatusGoal
from pal_interaction_msgs.msg import TtsGoal, TtsAction

class P1Server(object):
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
        self.table_status_client = actionlib.SimpleActionClient('/tableStatus', TableStatusAction)
        #self.speech_client = actionlib.SimpleActionClient('/tts', TtsAction)

        # Bool variable and wake_word subscriber for voice plan activation
        self.running = False
        rospy.Subscriber('/wake_word/wake_word_detected', String, self.handle_wake_word_detected)
        self.plan_publisher = rospy.Publisher('/p1/planToExec', String, queue_size=1)

        # Get the Tables Dictionary from the parameter server
        self.tables = rospy.get_param("/tables")
        self.tables_order = [self.tables[table]['id'] for table in self.tables if self.tables[table]['count']]
    
    def handle_wake_word_detected(self, data):
        wake_word = data.data
        if not self.running and wake_word == 'start the demo':
            self.running = True
            self.plan_publisher.publish('demo_plan')
  
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

    def initialise(self):
        # initialise PNP variables
        self._result.set_variables = [lpb_msg.VariableValue('tableIndex', '0'), lpb_msg.VariableValue('poseIndex', '0')]

    def gotoHome(self):
        rospy.loginfo('Going to home')

        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = Pose(position = Point(**self.tables['Home']['loc']['position']),
            orientation = Quaternion(**self.tables['Home']['loc']['orientation']))

        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) #waits forever
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")

    def goto(self, table_index, pose_index):
        # TODO: move to individual action file
        rospy.loginfo('Going to: %s' % self.tables_order[int(table_index)])

        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        # For now
        if int(pose_index) == 0:
            pose = 'far_pose'
        else:
            pose = 'close_pose'

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = Pose(position = Point(**self.tables['table' + table_index]['loc'][int(pose_index)][pose]['position']),
            orientation = Quaternion(**self.tables['table' + table_index]['loc'][int(pose_index)][pose]['orientation']))

        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) #waits forever
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")

        # If we moved close to the table means we have already taken the picture
        if pose_index == '1':
           self._result.condition_event = ['pictureDone']
           self._result.set_variables = [lpb_msg.VariableValue('poseIndex', str(0))]

    def countPeople(self, table_index):
        # TODO: move to individual action file
        # Take a picture of the table from afar
        # Wait for recognition action server to come up and send goal
        self.object_recognition_client.wait_for_server(rospy.Duration(15.0))
        recognition_goal = count_objectsGoal()
        recognition_goal.image_topic = "/xtion/rgb/image_raw"
        recognition_goal.confidence = 0.3
        recognition_goal.nms = 0.3
        self.object_recognition_client.send_goal(recognition_goal)
        rospy.loginfo('Recognition goal sent')

        # Waits for the server to finish performing the action.
        rospy.loginfo('Waiting for the detection result...')
        self.object_recognition_client.wait_for_result()

        # Prints out the result of executing the action
        self.countPeople_result = self.object_recognition_client.get_result()
        print('{0} persons'.format(self.countPeople_result.person))

        # Update the number of people in the parameter server
        rospy.loginfo('Updating the number of people found at table %s' % self.tables_order[int(table_index)])
        rospy.set_param('/tables/table' + table_index + '/person_count', self.countPeople_result.person)
        rospy.loginfo('Updated the person counter successfully')

        # Switch to the pose closer to the table
        rospy.loginfo('Switching to the second pose')
        self._result.set_variables = [lpb_msg.VariableValue('poseIndex', str(1))]

    # Sleeps are required to avoid Tiago's busy status from body motions controllers
    def identifyStatus(self, table_index):
        # TODO: move to individual action file
        rospy.loginfo('Identifying the status of: %s' % self.tables_order[int(table_index)])

        # Step 1: Look down to see the table
        # Wait for the play motion server to come up and send goal
        self.play_motion_client.wait_for_server(rospy.Duration(15.0))
        pose_goal = PlayMotionGoal()
        pose_goal.motion_name = "look_down"
        pose_goal.skip_planning = True
        self.play_motion_client.send_goal(pose_goal)
        rospy.loginfo('Looking down goal sent')
        rospy.sleep(3)

        # Step 2: Take a picture of the table surface
        # Wait for recognition action server to come up and send goal
        self.object_recognition_client.wait_for_server(rospy.Duration(15.0))
        recognition_goal = count_objectsGoal()
        recognition_goal.image_topic = "/xtion/rgb/image_raw"
        recognition_goal.confidence = 0.3
        recognition_goal.nms = 0.3
        self.object_recognition_client.send_goal(recognition_goal)
        rospy.loginfo('Recognition goal sent')

        # Waits for the server to finish performing the action.
        rospy.loginfo('Waiting for the detection result...')
        self.object_recognition_client.wait_for_result()

        # Prints out the result of executing the action
        surfaceObjects_result = self.object_recognition_client.get_result()
        print('{0} cups {1} bowls'.format(surfaceObjects_result.cup, surfaceObjects_result.bowl))

        # Step 3: Check the table surface for items
        # Wait for the table status action server to come up and send goal
        self.table_status_client.wait_for_server(rospy.Duration(15.0))
        status_goal = TableStatusGoal()
        status_goal.num_of_reads = 5
        self.table_status_client.send_goal(status_goal)
        rospy.loginfo('Check table status goal sent')

        # Waits for the server to finish performing the action.
        rospy.loginfo('Waiting for the status result...')
        self.table_status_client.wait_for_result()

        # Print the table status 
        status_result = self.table_status_client.get_result()
        rospy.loginfo('PclCheck result of %s is %s' % (self.tables_order[int(table_index)], status_result.status))

        # Step 4: Get head and torso back to default
        pose_goal.motion_name = "back_to_default"
        self.play_motion_client.send_goal(pose_goal)
        rospy.loginfo('Default head position goal sent')
        rospy.sleep(3)

        # Step 4: Decide on table status and send tts goal to the sound server
        # Create a tts goal
        #tts_goal = TtsGoal()
        #tts_goal.rawtext.lang_id = 'en_GB'
        foundPerson = False
        foundConsumable = False
        clean = False

        if self.countPeople_result.person > 0:
            foundPerson = True

        if surfaceObjects_result.cup or surfaceObjects_result.bowl > 0:
            foundConsumable = True

        if status_result.status == "clean":
            clean = True

        if foundPerson and foundConsumable:
            result = 'Status of {0} is Already served'.format(self.tables_order[int(table_index)])
        elif foundPerson and (not foundConsumable):
            result = 'Status of {0} is Needs serving'.format(self.tables_order[int(table_index)])
        elif not foundPerson:
            if clean:
                result = 'Status of {0} is Clean'.format(self.tables_order[int(table_index)])
            else:
                result = 'Status of {0} is Dirty'.format(self.tables_order[int(table_index)])
        
        # Update the status of the table in the parameter server
        rospy.loginfo('Updating the table status of table %s' % self.tables_order[int(table_index)])
        rospy.set_param('/tables/table' + table_index + '/status', result)
        rospy.loginfo('Updated the table status successfully')

        rospy.loginfo(result)
        #tts_goal.rawtext.text = result
        #self.tts_pub.publish(tts_goal)
        rospy.sleep(1)

    def count(self, table_index):
        # TODO: move to individual action file
        rospy.loginfo('Counting: %s' % self.tables_order[int(table_index)])

        if int(table_index) == len(self.tables_order) - 1:
            self._result.condition_event = ['doneCounting']
        else:
            self._result.set_variables = [lpb_msg.VariableValue('tableIndex', str(int(table_index) + 1))]

        







