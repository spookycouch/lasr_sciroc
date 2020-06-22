import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import cv2
import sys
sys.path.append('/home/jeff/openpose/build/python')

from openpose import pyopenpose as op
from math import atan2, pi
import time
import numpy as np

# cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

def get_waving_bbox():
    # start openpose
    params = {}
    params['model_folder'] = '/home/jeff/openpose/models/'
    params['hand'] = False
    params['net_resolution'] = '320x176'

    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()

    box = None

    # time.sleep(1)
    # print 'TAKING THE PICTURE IN:'
    # for x in range(3,-1,-1):
    #     print x
    #     time.sleep(1)

    try:
        # grabbed, frame = cap.read()
        # frame = cv2.imread('juan_wave.jpg')
        img_msg = rospy.wait_for_message('/xtion/rgb/image_raw',Image)
        frame = CvBridge().imgmsg_to_cv2(img_msg, "bgr8")

        datum = op.Datum()
        datum.cvInputData = frame
        opWrapper.emplaceAndPop([datum])
        image = datum.cvOutputData

        no_people = 0
        try:
            no_people = len(datum.poseKeypoints)
        except:
            pass

        for i in range(no_people):
            person = datum.poseKeypoints[i]

            # get angles via opposite (dy) and adjacent (dx)
            dy = person[4][1] - person[3][1], person[7][1] - person[6][1]
            dx = person[4][0] - person[3][0], person[7][0] - person[6][0]
            angles = np.arctan2(dy, dx)

            # normalise the probability of each arm
            # 1 if both points >= 0.7 confidence, 0 otherwise
            probs = np.array([person[3:5, 2], person[6:8, 2]])
            print probs
            probs = np.min(probs > 0, axis = 1)

            # mask range of angles from image onto prob
            margin = 0.25
            probs *= angles < -pi * margin
            probs *= angles > -pi * (1 - margin)

            # multiply angle of each arm by the normalised probability
            angles = angles * probs

            # number of hands up is number of non-zero entries
            handsup = np.argwhere(angles).size
            print angles, handsup

            # draw a bounding box for the face
            if handsup:
                x1, x2 = person[15][0], person[16][0]
                y1, y2 = person[0][1], person[1][1]
                cv2.rectangle(image, (x1, y1), (x2, y2), (0,255,0), 2)


                box = (x1, y1, x2-x1, y2-y1)
                break
    
    except Exception as e:
        print e
    
    # show image
    cv2.imshow('test', image)
    cv2.waitKey(0)

    opWrapper.stop()
    return box

if __name__ == '__main__':
    rospy.init_node('hi')
    print get_waving_bbox()