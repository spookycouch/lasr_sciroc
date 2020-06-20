#!/usr/bin/env python
import rospy
import actionlib

from SciRocServer import SciRocServer
from p1_server import P1Server
from p2_server import P2Server
from p3_server import P3Server

from lasr_dialogflow.msg import DialogAction, DialogGoal

if __name__ == '__main__':
    # init the node so we can access stuff
    rospy.init_node('sciroc')
    p1 = P1Server('server1')
    p2 = P2Server('server2')
    p3 = P3Server('server3')

    dialog_client = actionlib.SimpleActionClient('/lasr_dialogflow/LasrDialogflow', DialogAction)
    dialog_client.wait_for_server()

    # --------------- #
    # P2 SERVER START #
    # --------------- #
    
    # findUnserved() was designed to update a PNP condition event.
    # to remove PNP i set a variable on the param server instead
    #
    # this needs to be updated to use calum and juan's waving person detection later
    #
    p2.findUnserved()
    if rospy.has_param('/done_serving') and rospy.get_param('/done_serving'):
        print 'done serving'
    else:
        print 'not done serving'

    # use dialogflow to take the customer's order
    # dialog_client.send_goal(DialogGoal('takeOrder'))
    
    # go to the unserved table
    p2.gotoTable('0')

    # ------------- #
    # P2 SERVER END #
    # ------------- #

    rospy.loginfo('main.py ended')