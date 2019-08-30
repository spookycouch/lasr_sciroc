import rospy

# Actionlib messages
from sensor_msgs.msg import PointCloud2
from lasr_img_depth_mask.msg import DepthMaskGoal
from lasr_object_detection_yolo.msg import yolo_detectionGoal

# Depth Mask
# subscribes to topic until a recent depth cloud image (less than 2 seconds ago) is taken
def maskCallback(self, data):
    print('Time now: ' + str(rospy.Time.now().secs) + '. Time of pcl: ' + str(data.header.stamp.secs))
    if((rospy.Time.now().secs - data.header.stamp.secs) < 2):
        self.depth_points = data
        self.depth_sub.unregister()
    
def depthMask(self, filter_left, filter_right, filter_front):
    # create depth cloud subscriber, wait for depth_points to be updated
    self.depth_points = None
    self.depth_sub = rospy.Subscriber('/xtion/depth_registered/points', PointCloud2, self.maskCallback)
    while True:
        if self.depth_points != None:
            break
    # create goal
    mask_goal = DepthMaskGoal()
    mask_goal.depth_points = self.depth_points
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
    # Wait for the action server to come up
    self.object_recognition_client.wait_for_server(rospy.Duration(15.0))

    # Create the recognition goal
    recognition_goal = yolo_detectionGoal()
    recognition_goal.image_raw = image_raw
    recognition_goal.dataset = dataset
    recognition_goal.confidence = confidence
    recognition_goal.nms = nms

    # send goal and wait for result
    self.object_recognition_client.send_goal(recognition_goal)
    rospy.loginfo('Recognition goal sent')
    rospy.loginfo('Waiting for the detection result...')
    self.object_recognition_client.wait_for_result()
    return self.object_recognition_client.get_result()