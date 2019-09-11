#!/usr/bin/env python

import rospy
from lasr_sciroc.srv import RobotStatus, RobotStatusResponse
from geometry_msgs.msg import PoseWithCovarianceStamped
from mkhub_bridge import MKHubBridge

def handle_robot_status(req):
    # Get an amcl_pose msg
    pose_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)

    # Get the position and the timestamp from the callback message
    position = pose_msg.pose.pose.position

    # Construct a RobotStatus payload using the position
    bridge = MKHubBridge('https://api.mksmart.org/sciroc-competition', 'leedsasr', 'sciroc-robot-status')
    payload = bridge.constructRobotStatusPayload(req.status_message, req.episode, position.x, position.y, position.z)

    response =bridge.put('Tiago', payload)
    print('I HAVE JUST PUUUUUUUUUUUT TIAGO STATUS ON DE HUB MAN')

    # During development get the payload from the server to check that it has been posted on there
    got = bridge.get('Tiago')
    print('GOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOT EEEM')
    print(got)

    # Status code 201 and 204 are for 'Object created' and 'Object replaced'
    if response.status_code == 201 or response.status_code == 204:
        return RobotStatusResponse(True)
    else:
        return RobotStatusResponse(False)

def robot_status_server():
    rospy.init_node('robot_status_server')
    s = rospy.Service('robot_status', RobotStatus, handle_robot_status)
    print "Ready to PUT RobotStatus objects."
    rospy.spin()

if __name__ == "__main__":
    robot_status_server()