import rospy
import cv2
import numpy as np
import tf
from cv_bridge import CvBridge, CvBridgeError
from six.moves import queue

# Actionlib messages
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image, PointCloud2
from lasr_img_depth_mask.msg import DepthMaskGoal
from lasr_pcl.srv import DepthCropMask
from lasr_object_detection_yolo.srv import YoloDetection, Pcl2ToImage

import message_filters

# NEW IMPLEMENTATION
# def getPcl2AndImage(self):
#     vision_queue = queue.Queue()

#     # return the stuff
#     def pcl2_and_image_callback(pcl2, image):
#         vision_queue.put((pcl2, image))
    
#     image_sub = message_filters.Subscriber('/xtion/rgb/image_rect_color', Image)
#     pcl2_sub = message_filters.Subscriber('/xtion/depth_registered/points_throttle', PointCloud2)
#     ts = message_filters.ApproximateTimeSynchronizer([pcl2_sub, image_sub], 10, 0.2)
#     ts.registerCallback(pcl2_and_image_callback)

#     while True:
#         try:
#             result = vision_queue.get(block=False)
#             break
#         except queue.Empty:
#             rospy.sleep(0.1)
    
#     image_sub.unregister()
#     pcl2_sub.unregister()
#     return result

# OLD IMPLEMENTATION
def getPcl2AndImage(self):
    pcl2 = self.getRecentPcl()
    image = self.pclToImage()
    return pcl2, image

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


def getDepthMask(self, depth_points, point_min, point_max):
    # time and cloud
    current_time = rospy.Time.now()
    # min pointstamped
    pointStamped_min = PointStamped()
    # pointStamped_min.header.stamp = current_time
    pointStamped_min.header.frame_id = 'map'
    pointStamped_min.point = Point(*point_min)
    # max pointstamped
    pointStamped_max =  PointStamped()
    # pointStamped_max.header.stamp = current_time
    pointStamped_max.header.frame_id = 'map'
    pointStamped_max.point = Point(*point_max)
    # send goal and wait for result
    print('waiting for server')
    rospy.wait_for_service('/depth_crop_mask')
    print('waited for server')
    try:
        get_crop_mask = rospy.ServiceProxy('/depth_crop_mask', DepthCropMask)
        return get_crop_mask(depth_points, pointStamped_min, pointStamped_max)
    except rospy.ServiceException as e:
        print "Service call failed: %s"%e



def applyDepthMask(self, image_msg, mask_msg, blur):
    # height and width
    height = image_msg.height
    width = image_msg.width
    # get images
    image_raw = np.fromstring(image_msg.data, np.uint8)
    image_blur = cv2.blur(image_raw, (blur, blur))
    mask = np.fromstring(mask_msg.data, np.uint8)
    # reshape arrrays (fromstring) to matrices
    mask = mask.reshape(height * width, 1)
    image_raw = image_raw.reshape(height * width * 3, 1)
    # for index i in images,
    # select image_raw[i] when mask[i] true
    # select image_blur[i] when mask[i] false
    image_masked = np.empty(image_raw.shape)
    for i in range(3):
        image_masked[i::3] = np.where(mask, image_raw[i::3], image_blur[i::3])


    # create sensor_msgs image
    image_msg_out = Image()
    image_msg_out.header.stamp = image_msg.header.stamp
    image_msg_out.header.frame_id = image_msg.header.frame_id
    image_msg_out.height = image_msg.height
    image_msg_out.width = image_msg.width
    image_msg_out.encoding = image_msg.encoding
    image_msg_out.is_bigendian = image_msg.is_bigendian
    image_msg_out.step = image_msg.step
    image_msg_out.data = list(image_masked)

    # result
    return image_msg_out

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
def setCupSize(self, cup, depth_points, image_raw):
    # validate bounding box
    if cup.xywh[2] < 3 or cup.xywh[3] < 6:
        rospy.loginfo('cup too small to determine size')
        return
    
    # set ranges and get points of interest
    x_range = 4
    y_range = 15
    mid_x = int((cup.xywh[0] + cup.xywh[2]/2) - x_range/2)
    top_y = int(cup.xywh[1])
    btm_y = int(cup.xywh[1] + cup.xywh[3])
    
    # SEE WHERE THE 
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(image_raw, "bgr8")
    frame[top_y : top_y + y_range, mid_x : mid_x + x_range] = (0,0,255)
    frame[btm_y - y_range : btm_y, mid_x : mid_x + x_range] = (0,0,255)
    cv2.imshow('image_masked', frame)
    cv2.waitKey(0)

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

    if np.isnan(cup_top):
        return 'top nan'
    elif np.isnan(cup_bottom):
        return 'bottom nan'
    # determine and set size
    else:
        cup_height = cup_top - cup_bottom
        
        print(cup_height)

        if cup_height > 0.23:
            pass
        elif cup_height > 0.14:
            cup.name = 'large coffee'
        elif cup_height > 0.12:
            cup.name = 'medium coffee'
        elif cup_height > 0.09:
            cup.name = 'small coffee'
    
    return 'success'


def locateCustomer(self, person, depth_points):
    header = depth_points.header
    height = depth_points.height
    width = depth_points.width
    cloud = np.fromstring(depth_points.data, np.float32)
    cloud = cloud.reshape(height, width, 8)

    image_raw = self.pclToImage(depth_points)
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(image_raw, "bgr8")
    cv2.imshow('image_masked', frame)
    cv2.waitKey(0)

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
