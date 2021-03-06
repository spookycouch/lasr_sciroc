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

class DemoServer(object):
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
        self.speech_client = actionlib.SimpleActionClient('/tts', TtsAction)

        self.running = False
        rospy.Subscriber('/wake_word/wake_word_detected', String, self.handle_wake_word_detected)
        self.plan_publisher = rospy.Publisher('/p1/planToExec', String, queue_size=1)

        # load locations
        config_path = rospy.get_param('~config_path', path.join(path.dirname(path.realpath(__file__)), '../../config'))
        demo_locations_path = path.join(config_path, 'demo.yaml')
        demo_locations = yaml.load(file(demo_locations_path, 'r'))['locations']

        # demo load
        self.locations_order = [loc['id'] for loc in demo_locations if loc['count']]
        self.locations = {
            loc['id']: Pose(
                position=Point(**loc['pose']['position']),
                orientation=Quaternion(**loc['pose']['orientation'])
            )
            for loc in demo_locations
        }

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
        self._result.condition_event = ['set_0']

    # goto function for the demo
    def goto(self, table_index):
        # TODO: move to individual action file
        rospy.loginfo('Going to: %s' % self.locations_order[int(table_index)])

        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = self.locations[self.locations_order[int(table_index)]]

        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) #waits forever
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")

    def gotoHome(self):
        rospy.loginfo('Going to home')

        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = self.locations['home']

        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) #waits forever
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")

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

        # Switch to the pose closer to the table
        rospy.loginfo('Switching to the second pose')
        self._result.set_variables = [lpb_msg.VariableValue('poseIndex', str(1))]

    # demo identifystatus
    def identifyStatus(self, table_index):
        # TODO: move to individual action file
        rospy.loginfo('Identifying the status of: %s' % self.locations_order[int(table_index)])

        # Step 1: Look down to see the table
        # Wait for the play motion server to come up and send goal
        self.play_motion_client.wait_for_server(rospy.Duration(15.0))
        pose_goal = PlayMotionGoal()
        pose_goal.motion_name = "look_down"
        pose_goal.skip_planning = True
        self.play_motion_client.send_goal(pose_goal)
        rospy.loginfo('Looking down goal sent')
        rospy.sleep(4)

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
        rospy.loginfo('PclCheck result of %s is %s' % (self.locations_order[int(table_index)], status_result.status))

        # Step 4: Get head and torso back to default
        pose_goal.motion_name = "back_to_default"
        self.play_motion_client.send_goal(pose_goal)
        rospy.loginfo('Default head position goal sent')
        rospy.sleep(4)

        # Step 4: Decide on table status and publish it to the sound server
        # Create a tts goal
        tts_goal = TtsGoal()
        tts_goal.rawtext.lang_id = 'en_GB'
        foundConsumable = False
        clean = False

        if surfaceObjects_result.cup or surfaceObjects_result.bowl > 0:
            foundConsumable = True

        if status_result.status == "clean":
            clean = True

        if not foundConsumable and clean:
            result = 'The status of the {0} is Clean'.format(self.locations_order[int(table_index)])
        else:
            result = 'The status of the {0} is Dirty'.format(self.locations_order[int(table_index)])
        
        rospy.loginfo(result)
        tts_goal.rawtext.text = result
        self.speech_client.send_goal(tts_goal)
        rospy.sleep(1)
        self.running = False

    def count(self, table_index):
        # TODO: move to individual action file
        rospy.loginfo('Counting: %s' % self.locations_order[int(table_index)])

        if int(table_index) == len(self.locations_order) - 1:
            self._result.condition_event = ['doneCounting']
        else:
            self._result.set_variables = [lpb_msg.VariableValue('tableIndex', str(int(table_index) + 1))]