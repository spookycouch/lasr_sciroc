#!/usr/bin/python
import rospy
import actionlib
import sys
from sensor_msgs.msg import PointCloud2, PointCloud
import numpy
import tf
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import Point, PointStamped
import warnings
from control_msgs.msg import PointHeadAction, PointHeadGoal

def lookAt(point):
    # Create tf transformer for z point transformation to keep TIAGo's head on the same level
    tfBuffer = tf2_ros.Buffer()
    tf = tf2_ros.TransformListener(tfBuffer)

    # Wait for the server to come up
    point_head_client = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)
    rospy.loginfo('Waiting for the point head server to come up')
    point_head_client.wait_for_server(rospy.Duration(10.0))

    # Create the goal 
    print('Looking at: ', point)
    ph_goal = PointHeadGoal()
    ph_goal.target.header.frame_id = 'map'
    ph_goal.max_velocity = 1
    ph_goal.min_duration = rospy.Duration(0.5)
    ph_goal.target.point.x = point[0]
    ph_goal.target.point.y = point[1]

    ph_goal.pointing_frame = 'head_2_link'
    ph_goal.pointing_axis.x = 1
    ph_goal.pointing_axis.y = 0
    ph_goal.pointing_axis.z = 0

    ps = PointStamped()
    ps.header.stamp = rospy.Time(0)
    ps.header.frame_id = 'head_2_link'
    transform_ok = False
    while not transform_ok and not rospy.is_shutdown():
        try:
            transform = tfBuffer.lookup_transform('base_link', 'head_2_link', rospy.Time(0))
            get_z_ps = do_transform_point(ps, transform)
            transform_ok = True
        # This usually happens only on startup
        except tf2_ros.ExtrapolationException as e:
            rospy.sleep(1.0/4)
            ps.header.stamp = rospy.Time(0)
            rospy.logwarn("Exception on transforming point... trying again \n(" +
                                str(e) + ") at time " + str(ps.header.stamp))
        except tf2_ros.LookupException:
            pass
        except tf2_ros.ConnectivityException:
            pass

    ph_goal.target.point.z = get_z_ps.point.z
    print(get_z_ps.point.z)

    # Send the goal
    rospy.loginfo("Sending the goal...")
    point_head_client.send_goal(ph_goal)
    rospy.loginfo("Goal sent!!")

    rospy.sleep(3)
 

def getLocation(xywh):
    trform = tf.TransformListener()
    rospy.sleep(2)
    # print('got person')
    pcl2 = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
    
    #print('cloud')
    cloud = numpy.fromstring(pcl2.data, numpy.float32)                      
    c_x = int(xywh[0] + xywh[2]/2)
    c_y = int(xywh[1] + xywh[3]/2)
    c_y = c_y*pcl2.width*8
    c_x = c_x*8
    c_index = c_y + c_x
    (x,y,z) = cloud[c_index : c_index +3 ]
    p = Point(x, y, z)
    loc = PointStamped()
    loc.header = pcl2.header
    loc.point = p
    trform.waitForTransform('xtion_rgb_optical_frame', 'map', pcl2.header.stamp, rospy.Duration(4.0))
    person_loc = trform.transformPoint('map', loc)
    return person_loc
              
def main(args):
    rospy.init_node('location_getter', anonymous=True)
    # print('LOADED')
    getLocation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__ == '__main__':
    main(sys.argv)

