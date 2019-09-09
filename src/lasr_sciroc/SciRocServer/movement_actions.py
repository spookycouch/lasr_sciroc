
import rospy
import tf2_ros

# Actionlib messages
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Pose, PointStamped
from move_base_msgs.msg import MoveBaseGoal
from play_motion_msgs.msg import PlayMotionGoal
from control_msgs.msg import PointHeadGoal
from tf2_geometry_msgs import do_transform_point

# TODO: handle not reaching goal and bypass rest of count loop
def gotoTable(self):
    # Get the current table from the parameter server
    current_table = rospy.get_param('/current_table')
    rospy.loginfo('Going to: %s ', current_table)

    # Get a fresh updated copy of the tables dictionary from the parameter server
    tables = rospy.get_param('/tables')

    # Wait for the action server to come up
    self.move_base_client.wait_for_server(rospy.Duration(15.0))

    # Create the move_base goal and send it
    goal = MoveBaseGoal()
    goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
    goal.target_pose.pose = Pose(position = Point(**tables[current_table]['location']['position']),
        orientation = Quaternion(**tables[current_table]['location']['orientation']))

    rospy.loginfo('Sending goal location ...')
    self.move_base_client.send_goal(goal) 
    if self.move_base_client.wait_for_result():
        rospy.loginfo('Goal location achieved!')
    else:
        rospy.logwarn("Couldn't reach the goal!")

    
def gotoLocation(self, location):
    rospy.loginfo('Going to %s', location)
    location = rospy.get_param('/' + location)

    # Wait for the action server to come up
    self.move_base_client.wait_for_server(rospy.Duration(15.0))

    # Create the move_base goal and send it
    goal = MoveBaseGoal()
    goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
    goal.target_pose.pose = Pose(position = Point(**location['location']['position']),
        orientation = Quaternion(**location['location']['orientation']))

    rospy.loginfo('Sending goal location ...')
    self.move_base_client.send_goal(goal) 
    if self.move_base_client.wait_for_result():
        rospy.loginfo('Goal location achieved!')
    else:
        rospy.logwarn("Couldn't reach the goal!")

def playMotion(self, motion_name):
    # Wait for the play motion server to come up and send goal
    self.play_motion_client.wait_for_server(rospy.Duration(15.0))

    # Create the play_motion goal and send it
    pose_goal = PlayMotionGoal()
    pose_goal.motion_name = motion_name
    pose_goal.skip_planning = True
    self.play_motion_client.send_goal(pose_goal)
    rospy.loginfo('Play motion goal sent')
    self.play_motion_client.wait_for_result()

def lookAt(self, point):
    # Create tf transformer for z point transformation to keep TIAGo's head on the same level
    tfBuffer = tf2_ros.Buffer()
    tf = tf2_ros.TransformListener(tfBuffer)

    # Wait for the server to come up
    rospy.loginfo('Waiting for the point head server to come up')
    self.point_head_client.wait_for_server(rospy.Duration(10.0))

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
    self.point_head_client.send_goal(ph_goal)

    self.point_head_client.send_goal(ph_goal) 
    if self.point_head_client.wait_for_result():
        rospy.loginfo('Head goal achieved!')
    else:
        rospy.logwarn("Couldn't reach the head goal!")

def shiftQuaternion(self, orientation, radians):
        theEuler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        if(theEuler[2] > 0):
            newEuler = theEuler[0], theEuler[1], theEuler[2] - radians
        else:
            newEuler = theEuler[0], theEuler[1], theEuler[2] + radians
        theFakeQuaternion = tf.transformations.quaternion_from_euler(newEuler[0], newEuler[1], newEuler[2])
        theRealQuaternion = Quaternion(theFakeQuaternion[0], theFakeQuaternion[1], theFakeQuaternion[2], theFakeQuaternion[3])
        return theRealQuaternion
    
def turn(self):
    # Get his current pose
    current_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped).pose.pose

    # Change orientation to turn TIAGo 180 degrees
    self.move_base_client.wait_for_server(rospy.Duration(15.0))
    goal = MoveBaseGoal()
    goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
    goal.target_pose.pose = Pose(position=current_pose.position, orientation=self.shiftQuaternion(current_pose.orientation, PI))

    # Send the move_base goal
    rospy.loginfo('Sending goal location ...')
    self.move_base_client.send_goal(goal) 
    if self.move_base_client.wait_for_result():
        rospy.loginfo('Goal location achieved!')
    else:
        rospy.logwarn("Couldn't reach the goal!")