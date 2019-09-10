#!/usr/bin/python

import rospy

from SciRocServer import SciRocServer
from turn_robot.srv import TurnToPoint
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal

class P3Server(SciRocServer):
    def __init__(self, server_name):
        SciRocServer.__init__(self, server_name)
    
    def detectAndLocateCustomer(self):
        depth_points = self.getRecentPcl()
        image_bgr = self.pclToImage(depth_points)
        detection_result = self.detectObject(image_bgr, 'coco', 0.3, 0.5)
        detected_objects = detection_result.detected_objects
        foundCustomer = False
        for detection in detected_objects:
            if detection.name == 'person':
                foundCustomer = True
                location = self.locateCustomer(detection, depth_points)
                print(location)
                break
        
        # Call other shit
        if foundCustomer:
            result = self.calculateApproachPoint(location, 0.8)
            self.gotoPose(result)   

    def calculateApproachPoint(self, person_position, distance_factor):
        # Make Tiago turn to face the person using the turn to point service
        rospy.loginfo("Waiting for service turn_to_point")
        rospy.wait_for_service('/turn_to_point', timeout=10.0)
        rospy.loginfo("Turning towards person midpoint")
        try:
            turn_to_point = rospy.ServiceProxy('/turn_to_point', TurnToPoint)
            ttp = turn_to_point(person_position.point.x, person_position.point.y)
            if ttp.success:
                print "Successfully turned towards person"
            else:
                print "Failed to turned towards person"
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # Get Tiago's current pose
        amcl_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        tiago_pose = amcl_msg.pose.pose

        # We will get 80% of the distance to the point
        goal_pose = Pose()
        goal_pose.position.x = tiago_pose.position.x + (person_position.point.x - tiago_pose.position.x)*distance_factor
        goal_pose.position.y = tiago_pose.position.y + (person_position.point.y - tiago_pose.position.y)*distance_factor
        goal_pose.position.z = 0.0
        goal_pose.orientation = tiago_pose.orientation

        return goal_pose

    def gotoPose(self, goal_pose):
        rospy.loginfo('Going to %s', goal_pose)

        # Wait for the action server to come up
        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        # Create the move_base goal and send it
        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = goal_pose

        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) 
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")

        # Make Tiago turn to face the person using the turn to point service
        rospy.loginfo("Waiting for service turn_to_point")
        rospy.wait_for_service('/turn_to_point', timeout=10.0)
        rospy.loginfo("Turning towards person midpoint")
        try:
            turn_to_point = rospy.ServiceProxy('/turn_to_point', TurnToPoint)
            ttp = turn_to_point(goal_pose.position.x, goal_pose.position.y)
            if ttp.success:
                print "Successfully turned towards person"
            else:
                print "Failed to turned towards person"
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def findReady(self):
        # Get the Tables Dictionary from the parameter server
        tables = rospy.get_param("/tables")

        # Get the ID of a table that needs serving
        clean_table = {}
        for table in tables:
            if (tables[table])['status'] == 'Ready':
                if not len(clean_table):
                    clean_table['data'] = tables[table]['id']
                elif tables[table]['id'] < clean_table['data']:
                    clean_table['data'] = tables[table]['id']
        
        # if a clean table is found, set PNP foundReady to true
        if len(clean_table):
            rospy.set_param('/current_table', 'table' + str(clean_table['data']))
            print "\033[1;33m" + "The next table that needs serving is " + str(next_table) + "\033[0m"
            self._result.condition_event = ['foundReady']
        else:
            print "\033[1;33m" + "No clean tables found. " + "\033[0m"