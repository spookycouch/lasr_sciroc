#!/usr/bin/python
import rospy
import sys
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf2_ros import TransformException
import tf
import actionlib
from math import sqrt, atan2
from utilities import TheGlobalClass


def get_closer_to_person(guest_location):
    target_distance = 1.0

    try:
        move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        move_base_client.wait_for_server()

        person_point = guest_location.point

        amcl_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        robot_point = amcl_msg.pose.pose.position

        dist_x = person_point.x - robot_point.x
        dist_y = person_point.y - robot_point.y
        euclidian_dist = sqrt(dist_x * dist_x + dist_y * dist_y)

        # calculate target point if euclidian distance is not within threshold.
        # otherwise tiago is nearby, rotate around current point.
        if euclidian_dist > target_distance + (target_distance/10):
            # ratio of (desired dist)/(total dist)
            ratio = (euclidian_dist - target_distance)/euclidian_dist
            # add (ratio * actual dist) to robot point, basically scale the triangle
            target_x = robot_point.x + (ratio * dist_x)
            target_y = robot_point.y + (ratio * dist_y)
        else:
            target_x = robot_point.x
            target_y = robot_point.y
        
        target_point = Point(target_x, target_y, 0)

        # see if the target point works, try eight other candidates if not
        possible_points = []
        possible_points.append(target_point)
        # N,S,E,W
        possible_points.append(Point(person_point.x + 0, person_point.y - 1, 0))
        possible_points.append(Point(person_point.x + 0, person_point.y + 1, 0))
        possible_points.append(Point(person_point.x - 1, person_point.y + 0, 0))
        possible_points.append(Point(person_point.x + 1, person_point.y + 0, 0))
        # NE,NW,SE,SW
        possible_points.append(Point(person_point.x + 1, person_point.y + 1, 0))
        possible_points.append(Point(person_point.x - 1, person_point.y - 1, 0))
        possible_points.append(Point(person_point.x - 1, person_point.y + 1, 0))
        possible_points.append(Point(person_point.x + 1, person_point.y - 1, 0))

        for point in possible_points:
            possible, possible_pose, end_tol = TheGlobalClass.make_plan(point, current_pose = amcl_msg.pose.pose)
            print 'plan for:', target_point.x, target_point.y, possible
            if possible:
                target_point = possible_pose.position
                break

        
        # since point is along same line, use current pos to get new rotation
        current_angle = atan2(dist_y, dist_x)
        (x, y, z, w) = tf.transformations.quaternion_from_euler(0, 0, current_angle)
        target_quaternion = Quaternion(x, y, z, w)

        # create and send move base goal
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.frame_id = 'map'
        mb_goal.target_pose.header.stamp = rospy.Time.now()
        mb_goal.target_pose.pose.position = target_point
        mb_goal.target_pose.pose.orientation = target_quaternion
        
        # update POI on param server
        rospy.loginfo('person is at {}, {}'.format(person_point.x, person_point.y))
        x, y, z, w = (float(i) for i in (x,y,z,w))
        target_x, target_y = (float(i) for i in (target_x, target_y))
        rospy.set_param('/POI/location/position', {'x':target_x , 'y':target_y , 'z':0})
        rospy.set_param('/POI/location/orientation', {'x':x , 'y':y , 'z':z , 'w':w})

        # print mb_goal
        move_base_client.send_goal(mb_goal)
        if move_base_client.wait_for_result():
            # get new rotation
            rospy.loginfo('Getting a better angle!')
            amcl_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
            robot_point = amcl_msg.pose.pose.position
            target_x = robot_point.x
            target_y = robot_point.y
            target_point = Point(target_x, target_y, 0)

            dist_x = person_point.x - target_x
            dist_y = person_point.y - target_y
            current_angle = atan2(dist_y, dist_x)
            (x, y, z, w) = tf.transformations.quaternion_from_euler(0, 0, current_angle)
            target_quaternion = Quaternion(x, y, z, w)

            # rotate            
            mb_goal.target_pose.header.stamp = rospy.Time.now()
            mb_goal.target_pose.pose.position = target_point
            mb_goal.target_pose.pose.orientation = target_quaternion
            move_base_client.send_goal(mb_goal)

            # update POI on param server
            x, y, z, w = (float(i) for i in (x,y,z,w))
            target_x, target_y = (float(i) for i in (target_x, target_y))
            rospy.set_param('/POI/location/position', {'x':target_x , 'y':target_y , 'z':0})
            rospy.set_param('/POI/location/orientation', {'x':x , 'y':y , 'z':z , 'w':w})

            move_base_client.wait_for_result()
            rospy.loginfo('Got a better angle, now SHOULD be facing person!')
            return 'outcome1'
        else:
            rospy.logwarn("Couldn't reach the goal!")
    except TransformException as e:
        print e
        print 'transform error! probably tf extrapolation'

