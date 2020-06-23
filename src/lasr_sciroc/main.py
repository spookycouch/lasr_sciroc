#!/usr/bin/env python
import rospy
import smach
import math
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
from rotate import rotate
from get_closer_to_person import get_closer_to_person
from openpose_actions import get_waving_bbox
from location import getLocation, lookAt
from serve import serve

class World:
    def __init__(self):
        self.locations = []

world = World()
class InspectRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
    def execute(self, userdata):
        rospy.loginfo('Executing state InspectRoom')
        movebase_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        movebase_client.wait_for_server()
        locations = rospy.get_param('/locations')  
        for location in locations:
            status = rospy.get_param('/locations/' + location + '/status')
            look = rospy.get_param('/locations/' + location + '/look')


            # GO TO THE VANTAGE POINT
            if status == 'unchecked':
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
                    rospy.set_param('/locations/' + location + '/status', 'checked')
                else:
                    rospy.logwarn("Couldn't reach the goal!")
                

                # DETECT WAVING PERSON
                for point in look:
                    lookAt(point)

                    bbox = get_waving_bbox()
                    if bbox is not None:
                        loc = getLocation(bbox)
                        get_closer_to_person(loc)
                        world.locations.append(loc)
                        return 'outcome1'

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
        smach.StateMachine.add('InspectRoom', InspectRoom(), transitions={'outcome1': 'SpeakToPerson', 'outcome2' : 'end'})
        smach.StateMachine.add('SpeakToPerson', SpeakToPerson(), transitions={'outcome1':'InspectRoom'})
    outcome = sm.execute()



if __name__=='__main__':
    try:
        main()
    #wait for keyboard interrupt
    except rospy.ROSInterruptException:
        rospy.loginfo('State Machine terminated.')