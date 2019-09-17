#!/usr/bin/python

import rospy
import rospkg
import numpy as np
from datetime import datetime
from SciRocServer import SciRocServer
from turn_robot.srv import TurnToPoint
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal
from elevator import TheGlobalClass
from math import sqrt
from lasr_sciroc.srv import RobotStatus, RobotStatusResponse

import cv2
from cv_bridge import CvBridge, CvBridgeError

class P3Server(SciRocServer):
    def __init__(self, server_name):
        SciRocServer.__init__(self, server_name)

    def findReady(self):
        # Fetch tables dictionary from the parameter server
        tables = rospy.get_param("/tables")

        # get an unknown table of the lowest id
        next_table_id = None
        for table in tables:
            if tables[table]['status'] == 'Ready':
                if next_table_id is None or tables[table]['id'] < next_table_id:
                    next_table_id = tables[table]['id']

        if next_table_id is not None:
            rospy.set_param('/current_table', 'table' + str(next_table_id))
            print "\033[1;33m" + "The next table is " + str(next_table_id) + "\033[0m"

            # Update the RobotStatus on the hub using the service
            rospy.wait_for_service('/robot_status')
            try:
                robot_status = rospy.ServiceProxy('/robot_status', RobotStatus)
                response = robot_status('Checking for new customers', 'EPISODE3')
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        else:
            # if all tables have been identified, counting is done
            self._result.condition_event = ['doneEscorting']
    
    def detectAndLocateCustomer(self):
        # Get Cuboid for Min and Max points of the table
        cuboid = rospy.get_param('/WaitingArea/cuboid')

        foundCustomer_counter = 0
        while not rospy.is_shutdown():
            depth_points, image = self.getPcl2AndImage()
            mask_msg = self.getDepthMask(depth_points, cuboid['min_xyz'], cuboid['max_xyz'])
            image_masked = self.applyDepthMask(image, mask_msg.mask, 175)
            detection_result = self.detectObject(image_masked, "coco", 0.3, 0.3)

            # Save img to img dir for logging
            rospack = rospkg.RosPack()
            savedir = rospack.get_path('lasr_sciroc') + '/images/'
            now = datetime.now()
            cv2.imwrite(savedir + now.strftime("%Y-%m-%d-%H:%M:%S") + '.png', np.fromstring(detection_result.image_bb.data))

            foundCustomer = False
            persons_location = []
            for detection in detection_result.detected_objects:
                if detection.name == 'person':
                    foundCustomer = True
                    location = self.locateCustomer(detection, depth_points)
                    persons_location.append(location)
                    print(location)
            
            if foundCustomer:
                foundCustomer_counter += 1
                rospy.loginfo('Found customer! counter at {}'.format(foundCustomer_counter))
            else:
                rospy.loginfo('Didnt find a new customer, Tiago is so sad :( reseting counter')
                foundCustomer_counter = 0
            
            if foundCustomer_counter == 2:
                break
            else:
                rospy.sleep(1)

        # Calculate the goal pose and the distance for each person detected 
        persons_info = []
        for location in persons_location:
            goal_pose, distance = self.calculateApproachPoint(location, 0.8)
            persons_info.append((goal_pose, distance))

        # Findout whos the nearest person
        nearest_person = None
        for person_info in persons_info:
            if nearest_person is None or person_info[1] < nearest_person[1]:
                nearest_person = person_info

        # Go to the nearest person
        self.talk('New customer detected')
        self.gotoPose(nearest_person[0])

    def calculateApproachPoint(self, person_position, distance_factor):
        # Get Tiago's current pose
        amcl_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        tiago_pose = amcl_msg.pose.pose

        # We will get 80% of the distance to the point
        goal_pose = Pose()
        goal_pose.position.x = tiago_pose.position.x + (person_position.point.x - tiago_pose.position.x)*distance_factor
        goal_pose.position.y = tiago_pose.position.y + (person_position.point.y - tiago_pose.position.y)*distance_factor
        goal_pose.position.z = 0.0
        goal_pose.orientation = TheGlobalClass.quaternion_from_point_at_point(goal_pose.position, person_position.point)

        # Calculate distance
        distance = sqrt( (tiago_pose.position.x - person_position.point.x)**2 + (tiago_pose.position.y - person_position.point.y)**2 )

        return goal_pose, distance

    def greetCustomer(self):
        self.talk('Hello my name is Tiago, please follow me to a ready table!')

        # Update the RobotStatus on the hub using the service
        rospy.wait_for_service('/robot_status')
        try:
            robot_status = rospy.ServiceProxy('/robot_status', RobotStatus)
            response = robot_status('Escorting new customer to a ready table', 'EPISODE3')
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def confirmCustomerEscorted(self):
        # Get the current table from the parameter server
        current_table = rospy.get_param('/current_table')
        
        # Get Cuboid for Min and Max points of the table
        cuboid = rospy.get_param('/tables/' + current_table + '/cuboid')

        # Get Left and Right points of the sides of the table
        side_points = rospy.get_param('/tables/' + current_table + '/lookLR')

        self.talk('This is your table, sit down please so I can confirm that you are truly a devoted customer to the Tiago coffee shop!')
        rospy.sleep(2)
        i = 0
        while not rospy.is_shutdown():
            self.lookAt(side_points[i])
            if i == 1:
                i = 0
            else:
                i += 1
            # Run YOLO object detection
            depth_points, image = self.getPcl2AndImage()
            mask_msg = self.getDepthMask(depth_points, cuboid['min_xyz'], cuboid['max_xyz'])
            image_masked = self.applyDepthMask(image, mask_msg.mask, 175)
            detection_result = self.detectObject(image_masked, "coco", 0.3, 0.3)

            # Save img to img dir for logging
            rospack = rospkg.RosPack()
            savedir = rospack.get_path('lasr_sciroc') + '/images/'
            now = datetime.now()
            cv2.imwrite(savedir + now.strftime("%Y-%m-%d-%H:%M:%S") + '.png', detection_result.image_bb)

            customerSatDown = False
            for detection in detection_result.detected_objects:
                if detection.name == 'person':
                    customerSatDown = True
                    break
            
            if customerSatDown:
                # Update the parameter server
                rospy.set_param('/tables/' + current_table + '/status', 'Needs serving')
                self.talk('Enjoy your stay in my Coffee shop!')

                # RETURN TO DEFAULT POSE
                self.playMotion('back_to_default')

                 # Update the RobotStatus on the hub using the service
                rospy.wait_for_service('/robot_status')
                try:
                    robot_status = rospy.ServiceProxy('/robot_status', RobotStatus)
                    response = robot_status('New Customer escorted', 'EPISODE3')
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

                self.talk('I will now counting the people at the table!')
                break
            else:
                rospy.sleep(1)

