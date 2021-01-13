import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class ImageListener:
    left_image = None
    right_image = None

def setImageSubscriber(imageGen):
    bridge = CvBridge()
    def get_image_left(msg):
        imageGen.left_image = np.array(bridge.imgmsg_to_cv2(msg))
        img = imageGen.left_image.copy()
        noise = cv2.randn(img,(0,0,0),(50,50,50))
        #imageGen.left_image+=noise
    def get_image_right(msg):
        imageGen.right_image = np.array(bridge.imgmsg_to_cv2(msg))
        img = imageGen.right_image.copy()
        noise = cv2.randn(img,(0,0,0),(0,0,0))
        #imageGen.right_image+=noise
    rospy.init_node('image_listener', anonymous=True)
    sub_left = rospy.Subscriber('/cameras/front_left/raw', Image, get_image_left)
    sub_right = rospy.Subscriber('/cameras/front_right/raw', Image, get_image_right)

def setStatePublisher(init):
    publisher = rospy.Publisher('/robot_state',Odometry,queue_size=10)
    return publisher
