#!/usr/bin/env python
import rospy
import actionlib

from SciRocServer import SciRocServer
from p2_server import P2Server
from openpose_actions import get_waving_bbox

from lasr_dialogflow.msg import DialogAction, DialogGoal


def serve():
    # init the node so we can access stuff
    # rospy.init_node('sciroc')
    p2 = P2Server('server2')

    dialog_client = actionlib.SimpleActionClient('/lasr_dialogflow/LasrDialogflow', DialogAction)
    dialog_client.wait_for_server()

    # --------------- #
    # P2 SERVER START #
    # --------------- #

    # use dialogflow to take the customer's order
    rospy.loginfo('at table with waving person')
    dialog_client.send_goal(DialogGoal('takeOrder'))
    dialog_client.wait_for_result()
    rospy.loginfo('order taken')
    
    # Going to bar to communicate the customer's order
    p2.playMotion('back_to_default')
    p2.gotoLocation("Bar")
    rospy.loginfo('at bar')

    p2.orderConfirm()
    rospy.loginfo('order communicated')

    p2.checkOrder()

    # the order has been checked until it's made correct -> ready to dispatch -> load the items
    p2.waitLoad()

    # Bring the items to the waving person's table
    p2.gotoLocation('POI')

    # Waiting till the items are unloaded
    p2.waitUnload()

    # The table with waving person has been marked as served
    rospy.loginfo('waving person table served')

    # ------------- #
    # P2 SERVER END #
    # ------------- #

    rospy.loginfo('main.py ended')

if __name__ == '__main__':
    serve()