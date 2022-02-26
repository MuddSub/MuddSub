#!/usr/bin/env python
import numpy as np
import rospy
from vision.msg import BoundingBox
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Range_Estimation():
    CAMERA_FOV = 2 # check with our camera
    CAMERA_XRES = 416 # check with our camera
    NUM_SET_POINTS = 72
    FORWARD_STEP_SIZE = 1

    def __init__(self):
        rospy.init_node('range_estimation')
        self.depthMapSub  = rospy.Subscriber('/zed2/zed_node/depth/depth_registered', Image, self.callback)
        self.bridge = CvBridge()

    def callback(self, image):
        image_message = self.bridge.imgmsg_to_cv2(image, "32FC1")
        input_img = np.array(image_message,dtype = np.dtype('f8'))
        print(input_img[720/2][1280/2])
        print(input_img.size)
        

if __name__ == "__main__":
    range = Range_Estimation()
    rospy.spin()


