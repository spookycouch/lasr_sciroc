#!/usr/bin/python

import rospy
import rosnode
from SciRocServer import SciRocServer
from sensor_msgs.msg import Image
from collections import defaultdict

class P1Server(SciRocServer):
    def __init__(self, server_name):
        SciRocServer.__init__(self, server_name)

    def initialise(self):
        # # Restart move basse with a lower yaw threshold for more accurate turning
        # rospy.loginfo('killing move base server')
        # rospy.set_param("/move_base/PalLocalPlanner/yaw_goal_tolerance", 0.05)
        # rosnode.kill_nodes(['/move_base'])

        # if self.move_base_client.wait_for_server(rospy.Duration(10)):
        #     rospy.loginfo('Move base server is back up')
        # else:
        #     rospy.loginfo("Failed to connect to move base")
        pass
        
    
    def countPeople(self):
        table_index = rospy.get_param('/current_table')
        cuboid = rospy.get_param('/tables/table' + str(table_index) + '/cuboid')
        x_left = float((cuboid['max_xyz'])[0])
        x_right = float((cuboid['min_xyz'])[0])
        y = float(((cuboid['max_xyz'])[1]) + float((cuboid['min_xyz'])[1])) / 2
        points = [(x_left, y), (x_right, y)]
        object_count = defaultdict(int)

        # Take a picture using the depth mask and feed it to the detection
        for i in range(2):
            self.lookAt(points[i])
            depth_points = self.getRecentPcl()
            image = self.pclToImage(depth_points)
            mask = self.getDepthMask(depth_points, cuboid['min_xyz'], cuboid['max_xyz'])
            image_masked = self.applyDepthMask(image, mask, 150)
            count_objects_result = self.detectObject(mask_result.img_mask, "coco", 0.3, 0.3)

            # update dictionary
            for detection in count_objects_result.detected_objects:
                object_count[detection.name] += 1
            
            # view the image - debug
            # bridge = CvBridge()
            # frame = bridge.imgmsg_to_cv2(count_objects_result.image_bb, "bgr8")
            # cv2.imshow('image_masked', frame)
            # cv2.waitKey(0)

        # RETURN TO DEFAULT POSE
        self.playMotion('back_to_default')

        # calculate and output result
        person_count = object_count['person']
        speech_out = 'I see ' + str(person_count) + ' person'
        if not person_count == 1:
            speech_out += 's'
        self.talk(speech_out)

        # Update the number of people in the parameter server
        rospy.loginfo('Updating the number of people found at table %d' % table_index)
        rospy.set_param('/tables/table' + str(table_index) + '/person_count', person_count)
        rospy.loginfo('Updated the person counter successfully')



    # Sleeps are required to avoid Tiago's busy status from body motions controllers
    def identifyStatus(self):
        table_index = rospy.get_param('/current_table')
        rospy.loginfo('Identifying the status of: %d' % table_index)

        # Step 1: Look down to see the table
        self.playMotion('check_table')

        # Step 2: Take a picture of the table surface
        image_raw = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        count_objects_result = self.detectObject(image_raw, "coco", 0.3, 0.3)
        
        # dictionary of results
        object_count = defaultdict(int)
        for detection in count_objects_result.detected_objects:
            if(detection.name == 'cup'):
                self.setCupSize(detection)
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
        foundPerson = rospy.get_param('/tables/table' + str(table_index) + '/person_count')
        foundConsumable = len(object_count)

        if foundPerson:
            if foundConsumable:
                result = 'Already served'
            else:
                result = 'Needs serving'
        else:
            if foundConsumable:
                result = 'Dirty'
            else:
                result = 'Clean'
        
        # Update the status of the table in the parameter server
        rospy.loginfo('Updating the table status of table %d', table_index)
        rospy.set_param('/tables/table' + str(table_index) + '/status', result)
        rospy.loginfo('Updated the table status successfully')
        # output result
        self.talk('Status of table {0} is {1}'.format(table_index, result))
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
            rospy.set_param('/current_table', next_table_id)
            print "\033[1;33m" + "The next table is " + str(next_table_id) + "\033[0m"
        else:
            # if all tables have been identified, counting is done
            self._result.condition_event = ['doneCounting']

        







