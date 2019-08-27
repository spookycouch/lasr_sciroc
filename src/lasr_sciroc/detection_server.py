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
from pcl_segmentation.msg import DetectCustomerAction, DetectCustomerGoal
from pal_interaction_msgs.msg import TtsGoal, TtsAction

class DetectionServer(object):
    _feedback = lpb_msg.BridgeFeedback()
    _result = lpb_msg.BridgeResult()

    def __init__(self, server_name):
        # bridge server
        self._bridge_server = actionlib.SimpleActionServer(server_name, lpb_msg.BridgeAction, execute_cb=self.execute_cb, auto_start=False)
        self._bridge_server.start()

        # Initialising clients: move_base, playmotion and detection 
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        self.speech_client = actionlib.SimpleActionClient('/tts', TtsAction)
        self.detection_client = actionlib.SimpleActionClient('/DetectCustomer', DetectCustomerAction)

        self.running = False
        rospy.Subscriber('/wake_word/wake_word_detected', String, self.handle_wake_word_detected)
        self.plan_publisher = rospy.Publisher('/p1/planToExec', String, queue_size=1)

        # load locations
        config_path = rospy.get_param('~config_path', path.join(path.dirname(path.realpath(__file__)), '../../config'))
        detection_locations_path = path.join(config_path, 'detect.yaml')
        detection_locations = yaml.load(file(detection_locations_path, 'r'))['locations']

        # detection load
        self.locations = {
            loc['id']: Pose(
                position=Point(**loc['pose']['position']),
                orientation=Quaternion(**loc['pose']['orientation'])
            )
            for loc in detection_locations
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
        self._result.condition_event = ['set_start']

    # goto function for the demo
    def gotoTable(self):
        # TODO: move to individual action file
        rospy.loginfo('Going to a random place')

        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = self.locations['table']

        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) #waits forever
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")

    def gotoWaitingArea(self):
        rospy.loginfo('Going to waiting area')

        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = self.locations['waitingArea']

        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) #waits forever
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")

        rospy.sleep(2)

    def gotoWaitingAreaBad(self):
        rospy.loginfo('Going to waiting area')

        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = self.locations['waitingAreaBad']

        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) #waits forever
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")

        rospy.sleep(2)

    def gotoWaitingAreaReallyBad(self):
        rospy.loginfo('Going to waiting area')

        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = self.locations['waitingAreaReallyBad']

        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) #waits forever
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")

        rospy.sleep(2)

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

    def detect(self, detection_action):
        rospy.loginfo('Starting detection, action is %s', detection_action)

        self.detection_client.wait_for_server(rospy.Duration(15.0))

        goal = DetectCustomerGoal()
        goal.detection_action = detection_action

        rospy.loginfo('Sending detection goal ...')
        self.detection_client.send_goal(goal)
        
        # Waits for the server to finish performing the action.
        rospy.loginfo('Waiting for the detection result...')
        self.detection_client.wait_for_result()

        # Prints out the result of executing the action
        result = self.detection_client.get_result()
        print "Found customer is: %r" % (result.foundCustomer)

        # Switch to the pose closer to the table
        if detection_action == "start":
            rospy.loginfo('Switching to checking mode for next action')
            self._result.set_variables = [lpb_msg.VariableValue('detectAction', "check")]

if __name__== '__main__':
    rospy.init_node('DetectionServer')
    DetectionServer(rospy.get_name())
    rospy.spin()