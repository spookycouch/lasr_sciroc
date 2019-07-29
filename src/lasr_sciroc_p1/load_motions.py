#! /usr/bin/env python

import rospy
import actionlib
import rosparam
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal


if __name__ == '__main__':
    rospy.init_node('loader')

    # load the YAML file into the parameter server
    paramlist = rosparam.load_file("/tiago_ws/src/lasr_sciroc_p1/config/motions.yaml")
    for params, ns in paramlist:
        rosparam.upload_params(ns, params)
        