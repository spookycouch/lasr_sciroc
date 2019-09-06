#!/usr/bin/python

import rospy
import tf

from SciRocServer import SciRocServer
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Quaternion
from collections import defaultdict
from math import pi as PI

import cv2
from cv_bridge import CvBridge, CvBridgeError

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
            rospy.set_param('/current_table', 'table' + str(next_table))
            print "\033[1;33m" + "The next table that needs serving is " + str(next_table) + "\033[0m"
        else:
            # if all tables have been served, counting is done
            self._result.condition_event = ['doneServing']

    
    def orderConfirm(self):
        current_table = rospy.get_param('/current_table')

        # Fetch the order from the parameter server
        order = rospy.get_param('/tables/' + current_table + '/order')
        rospy.loginfo('The order fetched from the parameter server is ')
        print(order)  

        # Say the order
        self.talk('The order of {0} is {1}'.format(current_table, order))
        rospy.sleep(3)

    
    def checkOrder(self):
        current_table = rospy.get_param('/current_table')

        # Fetch the order from the parameter server
        order = rospy.get_param('/tables/' + current_table + '/order')
        rospy.loginfo('The order fetched from the parameter server in checkOrder is ')
        print(order)  

        # wait for the keyword
        # self.keywordDetected('check order')

        while True:
            # Look down to see the items on the counter
            self.playMotion('look_down')

            # Run the object detection client on the items
            depth_points = self.getRecentPcl()
            image_raw = self.pclToImage(depth_points)
            result = self.detectObject(image_raw, "coco", 0.3, 0.3)

            order_count = defaultdict(int)
            for item in order:
                order_count[item] += 1
            
            object_count = defaultdict(int)
            for detection in result.detected_objects:
                if(detection.name == 'cup'):
                    self.setCupSize(detection, depth_points)
                object_count[detection.name] += 1
            
            # view the image - debug
            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(result.image_bb, "bgr8")
            cv2.imshow('image_masked', frame)
            cv2.waitKey(0)

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
            else:
                self.talk('Order is correct, please place the items on my back.')
                break
            rospy.sleep(4)

    def waitLoad(self):
        # Turn TIAGo so customers grab the tings
        self.turn()

        # Wait for keyword detection porcupine
        self.keywordDetected('all set')
    
    def waitUnload(self):
        # Turn TIAGo so customers grab the tings
        self.turn()

        # Ask them very politely to take the tings off
        self.talk('Please collect your order and say we dont remember the keyword just now')

        # Wait for keyword detection porcupine
        self.keywordDetected('order collected')

        # Set the table to be already served
        current_table = rospy.get_param('/current_table')
        rospy.set_param('/tables/' + current_table + '/status', 'Already served')
