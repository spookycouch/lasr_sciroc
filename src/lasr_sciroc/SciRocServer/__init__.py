#!/usr/bin/env python

import rospy
import tf
import actionlib

# Actionlib messages
import lasr_pnp_bridge.msg as lpb_msg
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction
from play_motion_msgs.msg import PlayMotionAction
from pal_interaction_msgs.msg import TtsAction
from control_msgs.msg import PointHeadAction

class SciRocServer(object):
    _feedback = lpb_msg.BridgeFeedback()
    _result = lpb_msg.BridgeResult()

    # Imports
    from .movement_actions import gotoTable, gotoLocation, gotoPose, lookAt, playMotion
    from .speech_actions import planWakeWord, talk, keywordDetected, keywordCallback, logText
    from .vision_actions import detectObject, getDepthMask, getDepthNanMask, applyDepthMask, getPcl2AndImage, getTransformedPoint, setCupSize, locateCustomer, getRecentPcl, pclToImage

    def __init__(self, server_name):
        rospy.loginfo('%s Action Server has been initialised!', server_name)
        # bridge server
        self._bridge_server = actionlib.SimpleActionServer(server_name, lpb_msg.BridgeAction, execute_cb=self.execute_cb, auto_start=False)
        self._bridge_server.start()

        # Initialising clients: move_base, playmotion, objectRecognition and table_status
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        self.speech_client = actionlib.SimpleActionClient('/tts', TtsAction)
        self.point_head_client = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)

        # Bool variable and wake_word subscriber for voice plan activation
        self.running = False
        self.sub = rospy.Subscriber('/wake_word/wake_word_detected', String, self.planWakeWord)

        self.transformer = tf.TransformListener()

    
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
