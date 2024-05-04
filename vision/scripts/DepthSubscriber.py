#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import pdb

bridge = CvBridge()
def saveDepthImage(image):
    image_message = bridge.imgmsg_to_cv2(image, "32FC1")
    pdb.set_trace()



if __name__== '__main__':
    rospy.init_node('DepthSubscriber')
    rospy.Subscriber('/zed2/zed_node/depth/depth_registered', Image, saveDepthImage)
    rospy.spin()

