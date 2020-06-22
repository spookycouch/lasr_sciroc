#!/usr/bin/env python
import rospy
import actionlib

from SciRocServer import SciRocServer
from p1_server import P1Server
from p2_server import P2Server
from p3_server import P3Server
from openpose_actions import get_waving_bbox

from lasr_dialogflow.msg import DialogAction, DialogGoal


def detectWavingPerson():
    print get_waving_bbox()

def goToWavingPerson():
    p2.gotoTable('0')

if __name__ == '__main__':
    # init the node so we can access stuff
    rospy.init_node('sciroc')
    p1 = P1Server('server1')
    p2 = P2Server('server2')
    p3 = P3Server('server3')

    dialog_client = actionlib.SimpleActionClient('/lasr_dialogflow/LasrDialogflow', DialogAction)
    dialog_client.wait_for_server()
    # p2.gotoLocation("Bar")
    # p2.checkOrder()

    # --------------- #
    # P2 SERVER START #
    # --------------- #
    
    # go to the unserved table
    p2.gotoTable('0')
    
    # Identify the table with a waving person
    detectWavingPerson()                                     # TODO: import Juan's code

    # Go to the table with waving person
    goToWavingPerson()                                       # TODO: import Jared's code

    # use dialogflow to take the customer's order
    rospy.loginfo('at table with waving person')
    dialog_client.send_goal(DialogGoal('takeOrder'))
    dialog_client.wait_for_result()
    rospy.loginfo('order taken')
    
    # Going to bar to communicate the customer's order
    p2.gotoLocation("Bar")
    rospy.loginfo('at bar')

    p2.orderConfirm()
    rospy.loginfo('order communicated')

    p2.checkOrder()

    # the order has been checked until it's made correct -> ready to dispatch -> load the items
    p2.waitLoad()

    # Bring the items to the waving person's table
    goToWavingPerson()                                       # TODO: import Jared's code

    # Waiting till the items are unloaded
    p2.waitUnload()

    # The table with waving person has been marked as served
    rospy.loginfo('waving person table served')

    # Starting to server other tables

    # findUnserved() was designed to update a PNP condition event.
    # to remove PNP i set a variable on the param sat table with waving personerver instead
    #
    # this needs to be updated to use calum and juan's waving person detection later
    #
    p2.findUnserved()
    if rospy.has_param('/done_serving') and rospy.get_param('/done_serving'):
        print 'done serving'
    else:
        print 'not done serving'

    # go to the unserved table
    p2.gotoTable('0')



    # ------------- #
    # P2 SERVER END #
    # ------------- #

    rospy.loginfo('main.py ended')