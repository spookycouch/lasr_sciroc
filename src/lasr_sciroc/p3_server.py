#!/usr/bin/python

import rospy

from SciRocServer import SciRocServer
from turn_robot.srv import TurnToPoint
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal
from elevator import TheGlobalClass

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
        
        # Call other stuff
        if foundCustomer:
            result = self.calculateApproachPoint(location, 0.8)
            self.gotoPose(result)   

    def calculateApproachPoint(self, person_position, distance_factor):
        # Get Tiago's current pose
        amcl_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        tiago_pose = amcl_msg.pose.pose

        # We will get 80% of the distance to the point
        goal_pose = Pose()
        goal_pose.position.x = tiago_pose.position.x + (person_position.point.x - tiago_pose.position.x)*distance_factor
        goal_pose.position.y = tiago_pose.position.y + (person_position.point.y - tiago_pose.position.y)*distance_factor
        goal_pose.position.z = 0.0
        goal_pose.orientation = TheGlobalClass.quaternionAtPointFromPoint(goal_pose.position, person_position.point)

        return goal_pose