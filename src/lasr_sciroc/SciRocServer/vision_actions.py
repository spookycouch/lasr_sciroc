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
    
    # SEE WHERE THE 
    # image_raw = self.pclToImage(depth_points)
    # bridge = CvBridge()
    # frame = bridge.imgmsg_to_cv2(image_raw, "bgr8")
    # frame[top_y : top_y + y_range, mid_x : mid_x + x_range] = (0,0,255)
    # frame[btm_y - y_range : btm_y, mid_x : mid_x + x_range] = (0,0,255)
    # cv2.imshow('image_masked', frame)
    # cv2.waitKey(0)

    # get pcl and reshape to image dimensions
    header = depth_points.header
    height = depth_points.height
    width = depth_points.width
    cloud = np.fromstring(depth_points.data, np.float32)
    cloud = cloud.reshape(height, width, 8)

    # extract xyz values along points of interest
    top_cluster    = cloud[top_y : top_y + y_range, mid_x : mid_x + x_range, 0:3]
    bottom_cluster = cloud[btm_y - y_range : btm_y, mid_x : mid_x + x_range, 0:3]
    
    # transform points to map frame and get y values
    top_cluster_z = []
    bottom_cluster_z = []
    for axes in top_cluster:
        for point in axes:
            current_point = self.getTransformedPoint(header, point, 'map')
            top_cluster_z.append(current_point.point.z)
    for axes in bottom_cluster:
        for point in axes:
            current_point = self.getTransformedPoint(header, point, 'map')
            bottom_cluster_z.append(current_point.point.z)
    
    # get max and min y-coords
    cup_top    = np.nanmax(top_cluster_z)
    cup_bottom = np.nanmin(bottom_cluster_z)

    # determine and set size
    if (not np.isnan(cup_top)) and (not np.isnan(cup_bottom)):
        cup_height = cup_top - cup_bottom
        
        print(cup_height)

        if cup_height > 0.23:
            pass
        elif cup_height > 0.17:
            cup.name = 'large coffee'
        elif cup_height > 0.13:
            cup.name = 'medium coffee'
        elif cup_height > 0.04:
            cup.name = 'small coffee'


def locateCustomer(self, person, depth_points):
    header = depth_points.header
    height = depth_points.height
    width = depth_points.width
    cloud = np.fromstring(depth_points.data, np.float32)
    cloud = cloud.reshape(height, width, 8)

    region_size = 2
    while True:
        # calculate centre points
        centre_x = int((person.xywh[0] + person.xywh[2]/2) - region_size)
        centre_y = int((person.xywh[1] + person.xywh[3]/2) - region_size)
        # extract xyz values along points of interest
        centre_cluster = cloud[centre_y  : centre_y + region_size, centre_x : centre_x + region_size, 0:3]
        not_nan_count = 0

        for axes in centre_cluster:
            for point in axes:
                if not (np.isnan(point[0]) or np.isnan(point[1]) or np.isnan(point[2])):
                    not_nan_count += 1

        if not_nan_count >= 3:
            break
        
        region_size += 2

    mean = np.nanmean(centre_cluster, axis=1)
    mean = np.nanmean(mean, axis=0)
    centre_point = PointStamped()
    centre_point.header = depth_points.header
    centre_point.point = Point(*mean)

    self.transformer.waitForTransform('xtion_rgb_optical_frame', 'map', depth_points.header.stamp, rospy.Duration(4.0))
    person_point = self.transformer.transformPoint('map', centre_point)
    print person_point
    return person_point
