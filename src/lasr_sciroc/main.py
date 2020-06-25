#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import smach
import math
import actionlib
import sys
import subprocess
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
from get_closer_to_person import get_closer_to_person
from openpose_actions import get_waving_bbox
from location import getLocation, lookAt
from serve import serve
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

def talk(text, wait=True):
    print('\033[1;36mTIAGO: ' + text + '\033[0m')
    tts_proc = subprocess.Popen(['echo "{}" | festival --tts'.format(text)], shell=True)
    if wait:
        tts_proc.wait()

def playMotion(motion_name):
    # Wait for the play motion server to come up and send goal
    play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
    play_motion_client.wait_for_server(rospy.Duration(15.0))

    # Create the play_motion goal and send it
    pose_goal = PlayMotionGoal()
    pose_goal.motion_name = motion_name
    pose_goal.skip_planning = True
    play_motion_client.send_goal(pose_goal)
    rospy.loginfo('Play motion goal sent')
    play_motion_client.wait_for_result()


def go_to_bar():
    movebase_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    movebase_client.wait_for_server()
    bar_location = rospy.get_param('/Bar/location')

    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose = Pose(position = Point(**bar_location['position']),
                                orientation = Quaternion(**bar_location['orientation']))
    movebase_client.send_goal(goal)

    if movebase_client.wait_for_result():
        rospy.loginfo('Goal location achieved!')
    else:
        rospy.logwarn("Couldn't reach goal")


class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
    def execute(self, userdata):
        rospy.loginfo('Executing state init')
        playMotion('back_to_default')

        # initialise cv2
        frame = np.zeros((480,640,3))
        cv2.putText(frame, 'waiting for output...', (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1)
        cv2.imshow('robocup', frame)
        cv2.waitKey(1)
        rospy.sleep(0.5)
        cv2.imshow('robocup', frame)
        cv2.waitKey(1)
        rospy.sleep(0.5)
        cv2.imshow('robocup', frame)
        cv2.waitKey(1)
        rospy.sleep(0.5)

        return 'outcome1'


class InspectRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
    def execute(self, userdata):
        trform = tf.TransformListener()
        rospy.loginfo('Executing state InspectRoom')
        movebase_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        movebase_client.wait_for_server()
        locations = rospy.get_param('/locations')

        talk('time to search for some customers.', wait=False)

        for location in locations:
            look = rospy.get_param('/locations/' + location + '/look')

            # GO TO THE VANTAGE POINT
            goal = MoveBaseGoal()
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose = Pose(position = Point(**locations[location]['position']),
                                        orientation = Quaternion(**locations[location]['orientation']))
            movebase_client.send_goal(goal)
            rospy.loginfo('GOAL SENT! o: ' + location)
            # waits for the server to finish performing the action
            if movebase_client.wait_for_result():
                rospy.loginfo('Goal point achieved!')
            else:
                rospy.logwarn("Couldn't reach the goal!")
            

            # DETECT WAVING PERSON
            for point in look:
                lookAt(point)

                bbox = get_waving_bbox()
                if bbox is not None:
                    loc = getLocation(bbox, trform)
                    playMotion('back_to_default')
                    get_closer_to_person(loc)
                    lookAt([loc.point.x, loc.point.y])
                    print loc.point.x, loc.point.y
                    return 'outcome1'

            playMotion('back_to_default')
            talk('looks like nobody wants to buy my coffee here.')
            talk('business is going great.', wait=False)
        
        go_to_bar()
        return 'outcome2'


class SpeakToPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
    def execute(self, userdata):
        rospy.loginfo('Executing state SpeakToPerson')
        serve()
        return 'outcome1'

def main():
    rospy.init_node('cleanup_state_machine')
    #State machine
    sm = smach.StateMachine(outcomes=['outcome1', 'end'])
    with sm:
        smach.StateMachine.add('Initialise', Init(), transitions={'outcome1': 'InspectRoom'})
        smach.StateMachine.add('InspectRoom', InspectRoom(), transitions={'outcome1': 'SpeakToPerson', 'outcome2' : 'Initialise'})
        smach.StateMachine.add('SpeakToPerson', SpeakToPerson(), transitions={'outcome1':'Initialise'})
    outcome = sm.execute()


if __name__=='__main__':
    try:
        main()
    #wait for keyboard interrupt
    except rospy.ROSInterruptException:
        rospy.loginfo('State Machine terminated.')