#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from mkhub_bridge import MKHubBridge
    
def StartLocationPublishing():
    rospy.init_node('RobotLocationHubPublisher', anonymous=True)

    while not rospy.is_shutdown():
        # Get an amcl_pose msg
        pose_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)

        # Get the position and the timestamp from the callback message
        position = pose_msg.pose.pose.position

        # Construct a RobotLocation payload using the position
        bridge = MKHubBridge('https://api.mksmart.org/sciroc-competition', 'leedsasr', 'sciroc-robot-location')
        payload = bridge.constructRobotLocationPayload(position.x, position.y, position.z)

        # PUT the RobotLocation on the MKHub if the object is not there already otherwise updated it using POST
        dictionary, get_response = bridge.get('Tiago')
        if get_response.status_code == 404: # Error code 404 is for 'object not found'
            bridge.put('Tiago', payload) # Create the RobotLocation object
            print('I HAVE JUST PUUUUUUUUUUUT TIAGO ON DE HUB MAN')
        else:
            bridge.post('Tiago', payload) # Update the RobotLocation object
            print('I updated Tiago on the hub :)')

        # # During development get the payload from the server to check that it has been posted on there
        # got = bridge.get('Tiago')
        # print('GOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOT EEEM')
        # print(got)

        # Publish to the hub every one second as suggested in the rulebook
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        StartLocationPublishing()
    except rospy.ROSInterruptException:
        pass