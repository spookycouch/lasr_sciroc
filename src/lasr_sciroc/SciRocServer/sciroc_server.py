#!/usr/bin/env python

import rospy
import tf
import actionlib

# Actionlib messages
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction
from play_motion_msgs.msg import PlayMotionAction
from pal_interaction_msgs.msg import TtsAction
from control_msgs.msg import PointHeadAction
from lasr_speech.msg import informationAction

class SciRocServer(object):
    # Imports
    from .movement_actions import gotoTable, gotoLocation, gotoPose, lookAt, playMotion
    from .speech_actions import detectKeyword, planWakeWord, talk, keywordDetected, keywordCallback, logText

    from .vision_actions import detectObject, getDepthMask, getDepthNanMask, applyDepthMask, getPcl2AndImage, getTransformedPoint, setCupSize, locateCustomer, getRecentPcl, pclToImage

    def __init__(self, server_name):
        # Initialising clients: move_base, playmotion, objectRecognition and table_status
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        self.speech_client = actionlib.SimpleActionClient('/tts', TtsAction)
        self.point_head_client = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)
        self.keyword_client = actionlib.SimpleActionClient('/serve_food', informationAction)


        # Bool variable and wake_word subscriber for voice plan activation
        self.running = False
        self.sub = rospy.Subscriber('/wake_word/wake_word_detected', String, self.planWakeWord)

        # transformer for points
        self.transformer = tf.TransformListener()