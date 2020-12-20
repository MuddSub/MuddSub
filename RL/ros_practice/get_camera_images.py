import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageListener:
    left_image = None
    right_image = None

def displayImages():
    imageGen = ImageListener()
    bridge = CvBridge()
    def get_image_left(msg):
        imageGen.left_image = np.array(bridge.imgmsg_to_cv2(msg))
    def get_image_right(msg):
        imageGen.right_image = np.array(bridge.imgmsg_to_cv2(msg))
    rospy.init_node('image_listener', anonymous=True)
    sub_left = rospy.Subscriber('/cameras/front_left/raw', Image, get_image_left)    
    sub_right = rospy.Subscriber('/cameras/front_right/raw', Image, get_image_right)
    loopRate = rospy.Rate(1) #Hz
    while not rospy.is_shutdown():
        if imageGen.left_image is None or imageGen.right_image is None:
            continue
        left_img = imageGen.left_image
        right_img = imageGen.right_image
        print(left_img.shape)
        print(right_img.shape)
        cv2.imshow('Left Image', left_img)
        cv2.imshow('Right Image', right_img)
        cv2.waitKey()
        cv2.destroyAllWindows()
        loopRate.sleep()


if __name__ == '__main__':
    displayImages()
