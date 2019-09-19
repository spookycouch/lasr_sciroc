#!/usr/bin/python

import rospy
import rospkg
from datetime import datetime

import tf

from SciRocServer import SciRocServer
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Quaternion
from collections import defaultdict
from math import pi as PI
from utilities import MKHubBridge
from utilities.srv import RobotStatus, RobotStatusResponse
from utilities import TheGlobalClass

import numpy as np
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
                    next_table_id = tables[table]['id']
                elif tables[table]['id'] < next_table_id:
                    next_table_id = tables[table]['id']
        
        if needServing_exist and not rospy.get_param('/served_table'):
            rospy.set_param('/current_table', 'table' + str(next_table_id))
            print "\033[1;33m" + "The next table that needs serving is " + str(next_table_id) + "\033[0m"

            # Update the RobotStatus on the hub using the service
            rospy.wait_for_service('/robot_status')
            try:
                robot_status = rospy.ServiceProxy('/robot_status', RobotStatus)
                response = robot_status('Taking order of table' + str(next_table_id), 'EPISODE3')
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        else:
            # if all tables have been served, counting is done
            self._result.condition_event = ['doneServing']
        # log the param server
        self.logText()

    def updateHubTableOrder(self, status):
        current_table = rospy.get_param('/current_table')

        # Fetch the order from the parameter server set by dialogflow
        order = rospy.get_param('/tables/' + current_table + '/order')

        # MKHub bridge object
        bridge = MKHubBridge('https://api.mksmart.org/sciroc-competition', 'leedsasr', 'sciroc-episode3-order')

        # PUT the order of the table to the data hub
        payload = bridge.constructOrderPayload(current_table, order, status)
        print('PRINTING ORDER PAYLOAD')
        print(payload)
        response = bridge.put(current_table, payload)

        # Get the update to check (log)
        got = bridge.get(current_table)
        print(got)

    def get_order_string(self, items_array):
        items_dict = defaultdict(int)
        for item in items_array:
            items_dict[item] += 1

        string_array = []
        for item in items_dict:
            if items_dict[item] == 1:
                string_array.append(str(items_dict[item]) + ' ' + item)
            else:
                string_array.append(str(items_dict[item]) + ' ' + item + 's')
        
        items_count = len(string_array)
        if items_count == 0:
            return "nothing"
        elif items_count == 1:
            return string_array[0]
        elif items_count == 2:
            return " and ".join(string_array)
        else:
            return ", ".join(string_array[:-1]) + " and " + string_array[-1]
    
    def orderConfirm(self):
        current_table = rospy.get_param('/current_table')

        # Fetch the order from the parameter server
        order = rospy.get_param('/tables/' + current_table + '/order')
        rospy.loginfo('The order fetched from the parameter server is ')
        print(order)  

        # Update the RobotStatus on the hub using the service
        rospy.wait_for_service('/robot_status')
        try:
            robot_status = rospy.ServiceProxy('/robot_status', RobotStatus)
            response = robot_status('Relaying order of ' + current_table + 'to the employee', 'EPISODE3')
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # Say the order
        self.talk('The order of {0} is {1}'.format(current_table, self.get_order_string(order)))
        rospy.sleep(2)

    
    def checkOrder(self):
        current_table = rospy.get_param('/current_table')

        # Fetch the order from the parameter server
        order = rospy.get_param('/tables/' + current_table + '/order')
        rospy.loginfo('The order fetched from the parameter server in checkOrder is ')
        print(order)  

        # Update the RobotStatus on the hub using the service
        rospy.wait_for_service('/robot_status')
        try:
            robot_status = rospy.ServiceProxy('/robot_status', RobotStatus)
            response = robot_status('Checking order of ' + str(current_table), 'EPISODE3')
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # wait for the keyword
        self.talk('Could you please place the order on the counter and say \"check the items\" when you are done')
        self.keywordDetected('check the items')
        self.talk('Checking the order.')
        rospy.loginfo('keyword got!')

        # Look down to see the items on the counter
        self.playMotion('check_table')

        for x in range(4):
            # Run the object detection client on the items
            depth_points, image_raw = self.getPcl2AndImage()
            
            cuboid = rospy.get_param('/Bar/cuboid')
            mask_msg = self.getDepthNanMask(depth_points, cuboid['min_xyz'], cuboid['max_xyz'])
            image_masked = self.applyDepthMask(image_raw, mask_msg.mask, 175)
            result = self.detectObject(image_masked, "costa", 0.5, 0.3)

            order_count = defaultdict(int)
            for item in order:
                order_count[item] += 1
            
            object_count = defaultdict(int)
            for detection in result.detected_objects:
                if(detection.name == 'coffee'):
                    self.setCupSize(detection, depth_points, image_raw)
                object_count[detection.name] += 1
            
            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(result.image_bb, "bgr8")

            # Save img to img dir for logging
            rospack = rospkg.RosPack()
            savedir = rospack.get_path('lasr_sciroc') + '/images/'
            now = datetime.now()
            cv2.imwrite(savedir + now.strftime("%Y-%m-%d-%H:%M:%S") + '.png', frame)

            for count in object_count:
                print('I see ' + str(object_count[count]) + ' of ' + str(count))

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
            if 'coffee' in object_count:
                coffee_count = object_count['coffee']
                if coffee_count == 1:
                    suffix = ''
                else:
                    suffix = 's'
                speech_out = 'I could not get {} coffee size{}.'.format(coffee_count, suffix)
                speech_out += ' Could you please ensure all cups are in full view?'.format(coffee_count, suffix)
                self.talk(speech_out)
            elif len(missing_items) or len(excess_items):
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
                self.talk('Order is correct.')
                break
            rospy.sleep(2)
        self.talk('Please place the items on my back, and say "all set" when you are done.')
        self.playMotion('back_to_default')

    def waitLoad(self):
        # Turn TIAGo so customers grab the tings
        rospy.loginfo('Still did not turn yet')
        TheGlobalClass.turn_radians(PI, self.move_base_client)
        rospy.loginfo('Done turning')

        # Wait for keyword detection porcupine
        self.keywordDetected('all set')

        # Update the RobotStatus on the hub using the service
        rospy.wait_for_service('/robot_status')
        try:
            robot_status = rospy.ServiceProxy('/robot_status', RobotStatus)
            response = robot_status('Delivering order to ' + str(current_table), 'EPISODE3')
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def waitUnload(self):
        # Turn TIAGo so customers grab the tings
        TheGlobalClass.turn_radians(PI, self.move_base_client)

        # Ask them very politely to take the tings off
        self.talk('Please collect your order and say the phrase "items collected" when you are done')

        # Wait for keyword detection porcupine
        self.keywordDetected('items collected')

        # Set the table to be already served
        current_table = rospy.get_param('/current_table')
        rospy.set_param('/tables/' + current_table + '/status', 'Already served')

        # MKHub bridge object
        bridge = MKHubBridge('https://api.mksmart.org/sciroc-competition', 'leedsasr', 'sciroc-episode3-table')

        # Post the status of the table to the data hub
        person_count = rospy.get_param('/tables/' + current_table + '/person_count')
        payload = bridge.constructTablePayload(current_table, person_count, 'Already served')
        print('PRINTING PAYLOAD')
        print(payload)
        response = bridge.post(current_table, payload)

        # Get the update to check (log)
        got = bridge.get(current_table)
        print(got)

        # Update order status on the hub
        self.updateHubTableOrder('Complete')

        rospy.set_param('/served_table', True)

        # Update the RobotStatus on the hub using the service
        rospy.wait_for_service('/robot_status')
        try:
            robot_status = rospy.ServiceProxy('/robot_status', RobotStatus)
            response = robot_status('Order of ' + str(current_table) + ' delivered', 'EPISODE3')
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
