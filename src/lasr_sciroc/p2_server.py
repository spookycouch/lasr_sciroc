#!/usr/bin/python

import rospy
import tf.transformations

from SciRocServer import SciRocServer
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Quaternion
from collections import defaultdict
from math import pi as PI

class P2Server(SciRocServer):
    def __init__(self, server_name):
        SciRocServer.__init__(self, server_name)
    
    def findUnserved(self):
        # Get the Tables Dictionary from the parameter server
        tables = rospy.get_param("/tables")

        # Get the ID of a table that needs serving
        needServing_exist = False
        for table in tables:
            if (tables[table])['status'] == 'Needs serving':
                if not needServing_exist:
                    needServing_exist = True
                    next_table = tables[table]['id']
                elif tables[table]['id'] < next_table:
                    next_table = tables[table]['id']
        
        if needServing_exist:
            rospy.set_param('/HAL9000/current_table', next_table)
            print "\033[1;33m" + "The next table that needs serving is " + str(next_table) + "\033[0m"
        else:
            # if all tables have been served, counting is done
            self._result.condition_event = ['doneServing']

    
    def takeOrder(self):
        table_index = rospy.get_param('/HAL9000/current_table')

        # For now we will just change the rosparam server as if the customer placed the order
        order = ['water', 'biscotti', 'berry smoothie']
        print('[INFO] Updating the order at table %d', table_index)
        print('[INFO] the order is ')
        print(order)  
        rospy.set_param('/tables/table' + str(table_index) + '/order', order)
        rospy.loginfo('Updated the order of table %d successfully', table_index)
        rospy.sleep(5)

    
    def orderConfirm(self):
        table_index = rospy.get_param('/HAL9000/current_table')

        # Fetch the order from the parameter server
        order = rospy.get_param('/tables/table' + str(table_index) + '/order')
        rospy.loginfo('The order fetched from the parameter server is ')
        print(order)  

        # Say the order
        self.talk('The order of table {0} is {1}'.format(table_index, order))
        rospy.sleep(10)

    
    def checkOrder(self):
        table_index = rospy.get_param('/HAL9000/current_table')

        # Fetch the order from the parameter server
        order = rospy.get_param('/tables/table' + str(table_index) + '/order')
        rospy.loginfo('The order fetched from the parameter server in checkOrderCorrectness is ')
        print(order)  

        while True:
            rospy.sleep(5)
            # Look down to see the items on the counter
            self.playMotion('look_down')

            # Run the object detection client on the items
            image_raw = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
            result = self.detectObject(image_raw, "costa", 0.3, 0.3)

            order_count = defaultdict(int)
            for item in order:
                order_count[item] += 1
            
            object_count = defaultdict(int)
            for detection in result.detected_objects:
                object_count[detection.name] += 1
            for count in object_count:
                print('I see ' + str(object_count[count]) + ' of ' + str(count))

            # Back to default pose
            self.playMotion('back_to_default')

            # Compare the result to the order and announce the missing item
            missing_items = defaultdict(int)
            excess_items = defaultdict(int)
            for item in order_count:
                if object_count[item] < order_count[item]:
                    missing_items[item] = order_count[item] - object_count[item]
            for item in object_count:
                if object_count[item] > order_count[item]:
                    excess_items[item] = object_count[item] - order_count[item]

            # Output incorrect items
            if len(missing_items) or len(excess_items):
                speech_out = 'The order is incorrect. Please correct the order.'
                if len(missing_items):
                    speech_out += '. The missing items are '
                    for item in missing_items:
                        speech_out += ', ' + str(missing_items[item]) + ' ' + str(item)
                        if not missing_items[item] == 1:
                            speech_out += 's'
                # output excess items
                if len(excess_items):
                    speech_out += '. The excess items are:'
                    for item in excess_items:
                        speech_out += ', ' +  str(excess_items[item]) + ' ' + str(item)
                        if not excess_items[item] == 1:
                            speech_out += 's'
                self.talk(speech_out)
                # REMOVE THSI BREAK !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                break
            else:
                self.talk('Order is correct, please place the items on my back.')
                break
            rospy.sleep(4)

    def shiftQuaternion(self, orientation, radians):
        theEuler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        if(theEuler[2] > 0):
            newEuler = theEuler[0], theEuler[1], theEuler[2] - radians
        else:
            newEuler = theEuler[0], theEuler[1], theEuler[2] + radians
        theFakeQuaternion = tf.transformations.quaternion_from_euler(newEuler[0], newEuler[1], newEuler[2])
        theRealQuaternion = Quaternion(theFakeQuaternion[0], theFakeQuaternion[1], theFakeQuaternion[2], theFakeQuaternion[3])
        return theRealQuaternion
    
    def turn(self):
        # Get his current pose
        current_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped).pose.pose

        # Change orientation to turn TIAGo 180 degrees
        self.move_base_client.wait_for_server(rospy.Duration(15.0))
        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = Pose(position=current_pose.position, orientation=self.shiftQuaternion(current_pose.orientation, PI))

        # Send the move_base goal
        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) 
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")


    def waitLoad(self):
        # Turn TIAGo so customers grab the tings
        self.turn()

        # Wait for keyword detection porcupine
        # nothing for now
        rospy.sleep(15)
    
    def waitUnload(self):
        # Turn TIAGo so customers grab the tings
        self.turn()

        # Ask them very politely to take the tings off
        self.talk('Please collect your order and say we dont remember the keyword just now')

        # Wait for keyword detection porcupine
        # nothing for now
        rospy.sleep(15)

        # Set the table to be already served
        table_index = rospy.get_param('/HAL9000/current_table')
        rospy.set_param('/tables/table' + str(table_index) + '/status', 'Already served')
