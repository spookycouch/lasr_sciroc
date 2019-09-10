#!/usr/bin/python

import rospy
import rosnode
from SciRocServer import SciRocServer
from sensor_msgs.msg import Image
from collections import defaultdict

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
        
        # Get Cuboid for Min and Max points of the table
        cuboid = rospy.get_param('/tables/' + current_table + '/cuboid')

        # Get Left and Right points of the sides of the table
        side_points = rospy.get_param('/tables/' + current_table + '/lookLR')
        object_count = defaultdict(int)

        # Take a picture using the depth mask and feed it to the detection
        for i in range(2):
            self.lookAt(side_points[i])
            depth_points, image = self.getPcl2AndImage()
            mask_msg = self.getDepthMask(depth_points, cuboid['min_xyz'], cuboid['max_xyz'])
            image_masked = self.applyDepthMask(image, mask_msg.mask, 175)
            count_objects_result = self.detectObject(image_masked, "coco", 0.3, 0.3)

            # update dictionary
            for detection in count_objects_result.detected_objects:
                object_count[detection.name] += 1
            
            # view the image - debug
            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(count_objects_result.image_bb, "bgr8")
            cv2.imshow('image_masked', frame)
            cv2.waitKey(0)

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


    def identifyStatus(self):
        current_table = rospy.get_param('/current_table')
        rospy.loginfo('Identifying the status of: %s' % current_table)

        # Step 1: Look down to see the table
        self.playMotion('check_table')

        # Step 2: YOLOv3 object detection
        image_raw = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        count_objects_result = self.detectObject(image_raw, "costa", 0.3, 0.3)
        
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
            result = 'Already served'
        elif foundPerson and not foundConsumable:
            result = 'Needs serving'
        elif not foundPerson and foundConsumable:
            result = 'Dirty'
        else:
            result = 'Clean'
        
        # Update the status of the table in the parameter server
        rospy.loginfo('Updating the table status of %s', current_table)
        rospy.set_param('/tables/' + current_table + '/status', result)
        rospy.loginfo('Updated the table status successfully')
        # output result
        self.talk('Status of {0} is {1}'.format(current_table, result))
        rospy.sleep(1)

    # TODO: rename please to determineNextUnknownTable
    def count(self):
        rospy.loginfo('Counting all the tables')

        # if any table status is unknown, set it to the robot's current table
        tables = rospy.get_param("/tables")
        # get an unknown table of the lowest id

        next_table_id = None
        for table in tables:
            if tables[table]['status'] == 'unknown':
                if next_table_id is None or tables[table]['id'] < next_table_id:
                    next_table_id = tables[table]['id']

        if next_table_id is not None:
            rospy.set_param('/current_table', 'table' + str(next_table_id))
            print "\033[1;33m" + "The next table is " + str(next_table_id) + "\033[0m"
        else:
            # if all tables have been identified, counting is done
            self._result.condition_event = ['doneCounting']

        







