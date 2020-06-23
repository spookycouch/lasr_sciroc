#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
import sys
import actionlib
import tf


def rotate(degrees):
    rospy.sleep(1)
    amcl_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
    tiago_loc = amcl_msg.pose.pose.position
    orientation = amcl_msg.pose.pose.orientation
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
    euler = (roll, pitch, yaw)
    yaw_deg = euler[2] *(180/math.pi)
    #if yaw_deg < 0:
    #    yaw_deg+=360
    org_ang = yaw_deg*(math.pi/180)
    ang = degrees*(math.pi/180)
    loc = Point(tiago_loc.x, tiago_loc.y, tiago_loc.x)
    (x,y,z,w)= tf.transformations.quaternion_from_euler(0,0, (ang+org_ang))
    targ_q = Quaternion(x,y,z,w)
    movebase_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    movebase_client.wait_for_server()
    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = 'map'
    mb_goal.target_pose.header.stamp = rospy.Time.now()
    mb_goal.target_pose.pose.position = loc
    mb_goal.target_pose.pose.orientation = targ_q
    movebase_client.send_goal(mb_goal)
    if movebase_client.wait_for_result():
        rospy.loginfo('ROTATED: ' + str(degrees) + 'DEGREES')
    return
