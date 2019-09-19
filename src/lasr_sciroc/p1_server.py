#!/usr/bin/python

import rospy
import rosnode
import rospkg
import numpy as np
from datetime import datetime
from SciRocServer import SciRocServer
from sensor_msgs.msg import Image
from collections import defaultdict
from utilities import MKHubBridge
from utilities.srv import RobotStatusResponse, RobotStatus

# for debug
import cv2
from cv_bridge import CvBridge, CvBridgeError

class P1Server(SciRocServer):
    def __init__(self, server_name):
        SciRocServer.__init__(self, server_name)

    def initialise(self):
        pass

    def countPeople(self):
        # Get the current table from the parameter server
        current_table = rospy.get_param('/current_table')
        object_count = defaultdict(int)
        
        # Get Cuboid for Min and Max points of the table
        cuboid = rospy.get_param('/tables/' + current_table + '/cuboid')

        tables = rospy.get_param('/tables')
        locations = tables[current_table]['locations']
        rospy.loginfo('locations {}'.format(locations))

        # Get Left and Right points of the sides of the table
        side_points = rospy.get_param('/tables/' + current_table + '/lookLR')

        # Take a picture using the depth mask and feed it to the detection
        for i in range(len(side_points)):
            self.lookAt(side_points[i])
            rospy.loginfo('Getting the image..')
            depth_points, image = self.getPcl2AndImage()
            rospy.loginfo('GOT THE IMAGE GOT EEEM ')
            mask_msg = self.getDepthMask(depth_points, cuboid['min_xyz'], cuboid['max_xyz'])
            rospy.loginfo('FINISHED FROM DE MASK BRODA')
            image_masked = self.applyDepthMask(image, mask_msg.mask, 175)
            count_objects_result = self.detectObject(image_masked, "coco", 0.5, 0.3)

            # update dictionary
            for detection in count_objects_result.detected_objects:
                object_count[detection.name] += 1

            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(count_objects_result.image_bb, "bgr8")

            # Save img to img dir for logging
            rospack = rospkg.RosPack()
            savedir = rospack.get_path('lasr_sciroc') + '/images/'
            now = datetime.now()
            cv2.imwrite(savedir + now.strftime("%Y-%m-%d-%H:%M:%S") + '.png', frame)

        # RETURN TO DEFAULT POSE
        self.playMotion('back_to_default')

        # calculate and output result
        person_count = object_count['person']
        speech_out = 'I see ' + str(person_count) + ' person'
        if not person_count == 1:
            speech_out += 's'
        self.talk(speech_out)

        # Update the number of people in the parameter server
        rospy.loginfo('Updating the number of people found at %s' % current_table)
        rospy.set_param('/tables/' + current_table + '/person_count', person_count)
        rospy.loginfo('Updated the person counter successfully')

        # TODO: Make an updateHubTableStatus function that does the below since this block of code is duplicated a lot
        # MKHub bridge object
        bridge = MKHubBridge('https://api.mksmart.org/sciroc-competition', 'leedsasr', 'sciroc-episode3-table')

        # Post the people count of the table to the data hub
        status = rospy.get_param('/tables/' + current_table + '/status')
        payload = bridge.constructTablePayload(current_table, person_count, status)
        print('PRINTING PAYLOAD')
        print(payload)
        response = bridge.post(current_table, payload)

        # Get the update to check (log)
        got = bridge.get(current_table)
        print(got)


    def identifyStatus(self):
        current_table = rospy.get_param('/current_table')
        rospy.loginfo('Identifying the status of: %s' % current_table)

        # Step 1: Look down to see the table
        self.playMotion('check_table')

        # Step 2: YOLOv3 object detection
        image_raw = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        count_objects_result = self.detectObject(image_raw, "costa", 0.7, 0.3)
        
        # dictionary of results
        object_count = defaultdict(int)
        for detection in count_objects_result.detected_objects:
            object_count[detection.name] += 1

        # output results
        if len(object_count):
            speech_out = 'I see '
            for costa_object in object_count:
                speech_out += ', ' + str(object_count[costa_object]) + ' ' + str(costa_object)
                if not object_count[costa_object] == 1:
                    speech_out += 's'
            self.talk(speech_out)
        else:
            self.talk('no objects found')

        # Step 3: Get head and torso back to default
        self.playMotion('back_to_default')

        # Step 4: Decide on table status and send tts goal to the sound server
        foundPerson = rospy.get_param('/tables/' + current_table + '/person_count')
        foundConsumable = len(object_count)

        if foundPerson and foundConsumable:
            status = 'Already served'
        elif foundPerson and not foundConsumable:
            status = 'Needs serving'
        elif not foundPerson and foundConsumable:
            status = 'Needs cleaning'
        else:
            status = 'Ready'
        
        # Update the status of the table in the parameter server
        rospy.loginfo('Updating the status of %s', current_table)
        rospy.set_param('/tables/' + current_table + '/status', status)
        rospy.loginfo('Updated the status successfully')
        # output status
        self.talk('Status of {0} is {1}'.format(current_table, status))
        
        # MKHub bridge object
        bridge = MKHubBridge('https://api.mksmart.org/sciroc-competition', 'leedsasr', 'sciroc-episode3-table')

        # Post the status of the table to the data hub
        person_count = rospy.get_param('/tables/' + current_table + '/person_count')
        payload = bridge.constructTablePayload(current_table, person_count, status)
        print('PRINTING PAYLOAD')
        print(payload)
        response = bridge.post(current_table, payload)

        # Get the update to check (log)
        got = bridge.get(current_table)
        print(got)

    # TODO: rename please to determineNextUnknownTable
    def count(self):
        rospy.loginfo('Counting all the tables')

        # Fetch tables dictionary from the parameter server
        tables = rospy.get_param("/tables")

        # get an unknown table of the lowest id
        next_table_id = None
        for table in tables:
            print table
            if tables[table]['status'] == 'Unknown':
                if next_table_id is None or tables[table]['id'] < next_table_id:
                    next_table_id = tables[table]['id']

        if next_table_id is not None:
            rospy.set_param('/current_table', 'table' + str(next_table_id))
            print "\033[1;33m" + "The next table is " + str(next_table_id) + "\033[0m"

            # Update the RobotStatus on the hub using the service
            rospy.wait_for_service('/robot_status')
            try:
                robot_status = rospy.ServiceProxy('/robot_status', RobotStatus)
                response = robot_status('Checking table' + str(next_table_id), 'EPISODE3')
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        else:
            # if all tables have been identified, counting is done
            self._result.condition_event = ['doneCounting']

            # Update the RobotStatus on the hub using the service
            rospy.wait_for_service('/robot_status')
            try:
                robot_status = rospy.ServiceProxy('/robot_status', RobotStatus)
                response = robot_status('Done checking tables', 'EPISODE3')
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        # log the param server
        self.logText()

        







