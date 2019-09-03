#!/usr/bin/python

import rospy

from SciRocServer import SciRocServer

class P3Server(SciRocServer):
    def __init__(self, server_name):
        SciRocServer.__init__(self, server_name)
    
    def detectAndLocateCustomer(self):
        depth_points = self.getRecentPcl()
        image_bgr = self.pclToImage(depth_points)
        detection_result = self.detectObject(image_bgr, 'coco', 0.3, 0.5)
        detected_objects = detection_result.detected_objects
        for detection in detected_objects:
            if detection.name == 'person':
                location = self.locateCustomer(detection, depth_points)
                print(location)
