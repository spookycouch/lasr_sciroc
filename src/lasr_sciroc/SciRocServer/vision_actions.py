import rospy
import cv2
import numpy as np
import tf
from cv_bridge import CvBridge, CvBridgeError
from six.moves import queue

# Actionlib messages
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import PointCloud2
from lasr_img_depth_mask.msg import DepthMaskGoal
from lasr_object_detection_yolo.srv import YoloDetection, Pcl2ToImage

def getRecentPcl(self):
    pcl_queue = queue.Queue()

    # subscribes to topic until a recent depth cloud image (less than 2 seconds ago) is taken
    def pclCallback(data):
        print('Time now: ' + str(rospy.Time.now().secs) + '. Time of pcl: ' + str(data.header.stamp.secs))
        if((rospy.Time.now().secs - data.header.stamp.secs) < 2):
            pcl_queue.put(data)
            depth_sub.unregister()
    
    # once a recent pcl image has been captured, return it
    depth_sub = rospy.Subscriber('/xtion/depth_registered/points', PointCloud2, pclCallback)
    # return pcl_queue.get()
    while True:
        try:
            return pcl_queue.get(block=False)
        except queue.Empty:
            rospy.sleep(0.1)

def pclToImage(self, depth_points):
    rospy.wait_for_service('/pcl2_to_image')
    try:
        extract_image = rospy.ServiceProxy('/pcl2_to_image', Pcl2ToImage)
        return extract_image(depth_points).image_bgr
    except rospy.ServiceException as e:
        print "Service call failed: %s"%e

def depthMask(self, depth_points, filter_left, filter_right, filter_front):
    # create goal
    mask_goal = DepthMaskGoal()
    mask_goal.depth_points = depth_points
    mask_goal.filter_left = filter_left
    mask_goal.filter_right = filter_right
    mask_goal.filter_front = filter_front
    # send goal and wait for result
    self.depth_mask_client.send_goal(mask_goal)
    rospy.loginfo('Depth mask goal sent')
    rospy.loginfo('Waiting for the depth mask result...')
    self.depth_mask_client.wait_for_result()
    return self.depth_mask_client.get_result()

def detectObject(self, image_raw, dataset, confidence, nms):
    # wait for the service to come up
    rospy.wait_for_service('/yolo_detection')
    # call the service
    try:
        detect_objects = rospy.ServiceProxy('/yolo_detection', YoloDetection)
        return detect_objects(image_raw, dataset, confidence, nms)
    except rospy.ServiceException as e:
        print "Service call failed: %s"%e


# transform point to map frame
def getTransformedPoint(self, header, point, frame):
    current_point = PointStamped()
    current_point.header = header
    current_point.point = Point(*point)
    return self.transformer.transformPoint(frame, current_point)

# add size to the name field of a cup Detection
# for cups of invalid or indeterminate height, name remains unchanged
def setCupSize(self, cup, depth_points):

    # validate bounding box
    if cup.xywh[2] < 3 or cup.xywh[3] < 6:
        rospy.loginfo('cup too small to determine size')
        return
    
    # set ranges and get points of interest
    x_range = 2
    y_range = 5
    mid_x = int((cup.xywh[0] + cup.xywh[2]/2) - 1)
    top_y = cup.xywh[1]
    btm_y = cup.xywh[1] + cup.xywh[3]
    
    # get pcl and reshape to image dimensions
    header = depth_points.header
    height = depth_points.height
    width = depth_points.width
    cloud = np.fromstring(depth_points.data, np.float32)
    cloud = cloud.reshape(height, width, 8)

    # extract xyz values along points of interest
    top_cluster    = cloud[mid_x : mid_x + x_range, top_y : top_y + y_range, 0:3]
    bottom_cluster = cloud[mid_x : mid_x + x_range, btm_y - y_range : btm_y, 0:3]
    
    # transform points to map frame and get y values
    top_cluster_y = []
    bottom_cluster_y = []
    for axes in top_cluster:
        for point in axes:
            print(point)
            current_point = self.getTransformedPoint(header, point, 'map')
            top_cluster_y.append(current_point.point.z)
    for axes in bottom_cluster:
        for point in axes:
            current_point = self.getTransformedPoint(header, point, 'map')
            bottom_cluster_y.append(current_point.point.z)


    
    # get max and min y-coords
    cup_top    = np.nanmax(top_cluster_y)
    cup_bottom = np.nanmin(bottom_cluster_y)

    # determine and set size
    if (not np.isnan(cup_top)) and (not np.isnan(cup_bottom)):
        cup_height = cup_top - cup_bottom
        
        print(cup_height)

        # if cup_height > 0.23:
        #     pass
        # elif cup_height > 0.17:
        #     cup.name = 'large coffee'
        # elif cup_height > 0.13:
        #     cup.name = 'medium coffee'
        # elif cup_height > 0.9:
        #     cup.name = 'small coffee'
        if cup_height > 0.3:
            pass
        elif cup_height > 0.2:
            cup.name = 'large coffee'
        elif cup_height > 0.1:
            cup.name = 'medium coffee'
        elif cup_height > 0.5:
            cup.name = 'small coffee'


def locateCustomer(self, person, cloud):
    # Get center point of the person bounding box
    point1 = (person.xywh[0], person.xywh[1])
    point2 = (person.xywh[0] + person.xywh[2], person.xywh[1] + person.xywh[3])
    center_point = ( (point1[0] + point2[0])/2, (point1[1] + point2[1])/2 )

        # Converts a pixel representing the centre point of a detected
        # person to a real world coordinate with respect to the robot base.

        # Params:
        #     center_point[0]: x coordinate of the detection centre point
        #     center_point[1]: y coordinate of the detection centre point
        #     cloud: PointCloud2 from depth camera topic

        # Returns:
        #     PointStamped: PointStamped object with the real world
        #     coordinates relative to the robot base
    
    # Initialise PointCloud2 properties
    region_size = 2
    width = cloud.width
    height = cloud.height
    point_step = cloud.point_step
    row_step = cloud.row_step

    while True:
        # Get a region of points around the centre points
        x_centres = [center_point[0]]
        y_centres = [center_point[1]]
        for i in range(1, region_size):
            x_centres.append(center_point[0]-i)
            y_centres.append(center_point[1]-i)
        for i in range(1, region_size):
            x_centres.append(center_point[0]+i)
            y_centres.append(center_point[1]+i)

        x_array = []
        y_array = []
        z_array = []

        # Extract point in PointCloud corresponding to the detection centre
        for x,y in zip(x_centres,y_centres):
            array_pos = y*row_step + x*point_step
            # extract x,y,z as separate arrays in order of pixel.
            x_bytes = [ord(x) for x in cloud.data[array_pos:array_pos+4]]
            y_bytes = [ord(x) for x in cloud.data[array_pos+4: array_pos+8]]
            z_bytes = [ord(x) for x in cloud.data[array_pos+8:array_pos+12]]
            # pack the bytes into
            byte_format=struct.pack('4B', *x_bytes)
            X = struct.unpack('f', byte_format)[0]

            byte_format=struct.pack('4B', *y_bytes)
            Y = struct.unpack('f', byte_format)[0]

            byte_format=struct.pack('4B', *z_bytes)
            Z = struct.unpack('f', byte_format)[0]

            # Filter out NaN values from PointCloud as it is not dense
            if not math.isnan(X) and not math.isnan(Y) and not math.isnan(Z):
                x_array.append(X)
                y_array.append(Y)
                z_array.append(Z)

        # Check at least 3 valid points exist to calculate average
        if len(x_array) >= 3:
            break

        # Increase region of points around the centre points
        region_size += 5

    x_mean = numpy.mean(x_array)
    y_mean = numpy.mean(y_array)
    z_mean = numpy.mean(z_array)

    transformer = tf.TransformListener()
    transformer.waitForTransform("/xtion_rgb_optical_frame", "/map", rospy.Time(0), rospy.Duration(4.0))
    depth_point = PointStamped()
    depth_point.header.frame_id="/xtion_rgb_optical_frame"
    depth_point.header.stamp=cloud.header.stamp
    depth_point.point.x=x_mean
    depth_point.point.y=y_mean
    depth_point.point.z=z_mean
    try:
        person_position = transformer.transformPoint("map", depth_point)
        print(person_position)
        # return goal
    except:
        pass

    # Set a move base goal to our found point
        # Wait for the action server to come up
    # self.move_base_client.wait_for_server(rospy.Duration(15.0))

    # Create the move_base goal and send it
    # goal = MoveBaseGoal()
    # goal.target_pose.header = Header(frame_id="map", stamp=rospy.Time.now())
    # goal.target_pose.pose.x = person_position.point.x
    # goal.target_pose.pose.y = person_position.y
    # goal.target_pose.pose.z = 0

    # rospy.loginfo('Sending goal location ...')
    # self.move_base_client.send_goal(goal) 
    # if self.move_base_client.wait_for_result():
    #     rospy.loginfo('Goal location achieved!')
    # else:
    #     rospy.logwarn("Couldn't reach the goal!")
