#!/usr/bin/python

import yaml
from os import path

import rospy
import actionlib
import rosparam

# Actionlib messages
import lasr_pnp_bridge.msg as lpb_msg
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, PointCloud2
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Quaternion, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from lasr_img_depth_mask.msg import DepthMaskAction, DepthMaskGoal
from lasr_object_detection_yolo.msg import yolo_detectionAction, yolo_detectionGoal
from pal_interaction_msgs.msg import TtsGoal, TtsAction
from collections import defaultdict

import cv2
from cv_bridge import CvBridge, CvBridgeError

class P1Server(object):
    _feedback = lpb_msg.BridgeFeedback()
    _result = lpb_msg.BridgeResult()

    def __init__(self, server_name):
        # bridge server
        self._bridge_server = actionlib.SimpleActionServer(server_name, lpb_msg.BridgeAction, execute_cb=self.execute_cb, auto_start=False)
        self._bridge_server.start()

        # Initialising clients: move_base, playmotion, objectRecognition and table_status
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        self.depth_mask_client = actionlib.SimpleActionClient('/depth_mask', DepthMaskAction)
        self.object_recognition_client = actionlib.SimpleActionClient('/yolo_detection', yolo_detectionAction)
        self.speech_client = actionlib.SimpleActionClient('/tts', TtsAction)

        # Bool variable and wake_word subscriber for voice plan activation
        self.running = False
        rospy.Subscriber('/wake_word/wake_word_detected', String, self.handle_wake_word_detected)
        self.plan_publisher = rospy.Publisher('/p1/planToExec', String, queue_size=1)

        # Get the Tables Dictionary from the parameter server
        self.tables = rospy.get_param("/tables")
    
    def handle_wake_word_detected(self, data):
        wake_word = data.data
        if not self.running and wake_word == 'start the demo':
            self.running = True
            self.plan_publisher.publish('p1PlanNew')
  
    def execute_cb(self, goal):
        rospy.loginfo("----------ExternalServer start----------")

        # log action and parameters
        rospy.loginfo("action is: " + str(goal.action))
        rospy.loginfo("params are: " + str(goal.params))

        # reset result 
        self._result = lpb_msg.BridgeResult()

        # call the action
        getattr(self, goal.action)(*goal.params)

        self._bridge_server.set_succeeded(self._result)

        # end callback
        rospy.loginfo("----------ExternalServer end----------")

    def initialise(self):
        # initialise PNP variables
        rospy.set_param('/HAL9000/current_table', 3)
        rospy.set_param('/HAL9000/current_pose', 0)

    def gotoHome(self):
        rospy.loginfo('Going to home')
        home = rospy.get_param('/Home')

        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = Pose(position = Point(**home['loc']['position']),
            orientation = Quaternion(**home['loc']['orientation']))

        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) #waits forever
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")

    def goto(self):
        table_index = rospy.get_param('/HAL9000/current_table')
        pose_index = rospy.get_param('/HAL9000/current_pose')
        print table_index
        # TODO: move to individual action file
        rospy.loginfo('Going to: %d with pose %d', table_index, pose_index)

        self.move_base_client.wait_for_server(rospy.Duration(15.0))

        # For now
        if pose_index == 0:
            pose = 'far_pose'
        else:
            pose = 'close_pose'

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
        goal.target_pose.pose = Pose(position = Point(**self.tables['table' + str(table_index)]['loc'][pose_index][pose]['position']),
            orientation = Quaternion(**self.tables['table' + str(table_index)]['loc'][pose_index][pose]['orientation']))

        rospy.loginfo('Sending goal location ...')
        self.move_base_client.send_goal(goal) #waits forever
        if self.move_base_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
        else:
            rospy.logwarn("Couldn't reach the goal!")

        # If we moved close to the table means we have already taken the picture
        if pose_index == 1:
           self._result.condition_event = ['pictureDone']
           rospy.set_param('/HAL9000/current_pose', 0)

    def talk(self, speech_in):
        print('\033[1;36mTIAGO: ' + speech_in + '\033[0m')
        tts_goal = TtsGoal()
        tts_goal.rawtext.lang_id = 'en_GB'
        tts_goal.rawtext.text = speech_in
        self.speech_client.send_goal(tts_goal)


    def countPeople(self):
        table_index = rospy.get_param('/HAL9000/current_table')
        # TODO: move to individual action file
        # Take a picture of the table from afar
        # Wait for recognition action server to come up and send goal

        # DEPTH MASK
        self.depth_mask_client.wait_for_server(rospy.Duration(15.0))
        mask_goal = DepthMaskGoal()
        mask_goal.depth_points = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
        mask_goal.filter_left = 1
        mask_goal.filter_right = 1
        mask_goal.filter_front = 3.5
        # send goal and wait for result
        self.depth_mask_client.send_goal(mask_goal)
        rospy.loginfo('Depth mask goal sent')
        rospy.loginfo('Waiting for the depth mask result...')
        self.depth_mask_client.wait_for_result()
        mask_result = self.depth_mask_client.get_result()

        # COCO DETECTION
        #TODO VIEW RESULTS BECAUSE INVISIBLE PERSON APPEARED
        self.object_recognition_client.wait_for_server(rospy.Duration(15.0))
        # create goal
        recognition_goal = yolo_detectionGoal()
        recognition_goal.image_raw = mask_result.img_mask
        recognition_goal.dataset = "coco"
        recognition_goal.confidence = 0.3
        recognition_goal.nms = 0.3
        # send goal and wait for result
        self.object_recognition_client.send_goal(recognition_goal)
        rospy.loginfo('Recognition goal sent')
        rospy.loginfo('Waiting for the detection result...')
        self.object_recognition_client.wait_for_result()
        count_objects_result = self.object_recognition_client.get_result()

        # dictionary of results
        object_count = defaultdict(int)
        for detection in count_objects_result.detected_objects:
            object_count[detection.name] += 1
        person_count = object_count['person']
        speech_out = 'I see ' + str(person_count) + ' person'
        if not person_count == 1:
            speech_out += 's'
        
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(count_objects_result.image_bb, "bgr8")

        cv2.imshow('image_masked', frame)
        cv2.waitKey(0)


        # output result
        self.talk(speech_out)

        # Update the number of people in the parameter server
        rospy.loginfo('Updating the number of people found at table %d' % table_index)
        rospy.set_param('/tables/table' + str(table_index) + '/person_count', person_count)
        rospy.loginfo('Updated the person counter successfully')

        # Switch to the pose closer to the table
        rospy.loginfo('Switching to the second pose')
        rospy.set_param('/HAL9000/current_pose', 1)



    # Sleeps are required to avoid Tiago's busy status from body motions controllers
    def identifyStatus(self):
        # TODO: move to individual action file
        table_index = rospy.get_param('/HAL9000/current_table')
        rospy.loginfo('Identifying the status of: %d' % table_index)

        # Step 1: Look down to see the table
        # Wait for the play motion server to come up and send goal
        self.play_motion_client.wait_for_server(rospy.Duration(15.0))
        pose_goal = PlayMotionGoal()
        pose_goal.motion_name = "look_down"
        pose_goal.skip_planning = True
        self.play_motion_client.send_goal(pose_goal)
        rospy.loginfo('Looking down goal sent')
        rospy.sleep(3)

        # Step 2: Take a picture of the table surface
        # Wait for recognition action server to come up and send goal
        # COSTA DETECTION
        self.object_recognition_client.wait_for_server(rospy.Duration(15.0))
        # create goal
        recognition_goal = yolo_detectionGoal()
        recognition_goal.image_raw = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        recognition_goal.dataset = "costa"
        recognition_goal.confidence = 0.3
        recognition_goal.nms = 0.3
        # send goal and wait for result
        self.object_recognition_client.send_goal(recognition_goal)
        rospy.loginfo('Recognition goal sent')
        rospy.loginfo('Waiting for the detection result...')
        self.object_recognition_client.wait_for_result()
        count_objects_result = self.object_recognition_client.get_result()

        # dictionary of results
        object_count = defaultdict(int)
        for detection in count_objects_result.detected_objects:
            object_count[detection.name] += 1
        if len(object_count):
            speech_out = 'I see '
            for costa_object in object_count:
                speech_out += ', ' + str(object_count[costa_object]) + ' ' + str(costa_object)
                if not object_count[costa_object] == 1:
                    speech_out += 's'
            self.talk(speech_out)
        else:
            self.talk('no objects found')

        # Step 4: Get head and torso back to default
        pose_goal.motion_name = "back_to_default"
        self.play_motion_client.send_goal(pose_goal)
        rospy.loginfo('Default head position goal sent')
        rospy.sleep(3)

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

    def count(self):
        # TODO: move to individual action file
        rospy.loginfo('Counting all the tables')

        # if any table status is unknown, set it to the robot's current table
        self.tables = rospy.get_param("/tables")
        print(self.tables)
        unknown_exist = False
        for table in self.tables:
            print(table)
            if (self.tables[table])['status'] == 'unknown':
                if not unknown_exist:
                    unknown_exist = True
                    next_table = self.tables[table]['id']
                elif self.tables[table]['id'] < next_table:
                    next_table = self.tables[table]['id']

        if unknown_exist:
            rospy.set_param('/HAL9000/current_table', next_table)
            print "\033[1;33m" + "The next table is " + str(next_table) + "\033[0m"
        else:
            # if all tables have been identified, counting is done
            self._result.condition_event = ['doneCounting']

        







