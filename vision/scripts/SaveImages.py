#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from torchvision import transforms
from torch.autograd import Variable
import numpy as np
import cv2
from std_msgs.msg import Header, String
import os
import sys

bridge = CvBridge()

def callback(image):
    image_message = bridge.imgmsg_to_cv2(image, "bgr8")
    print("Current working directory: {0}".format(os.getcwd()))
    now = rospy.get_rostime() # current date and time
    image_name = f"{image_storage_path}/{now.secs},{now.nsecs}.jpg"
    print(f"The image name is {image_name}")

    cv2.imwrite(image_name,image_message)
    # input_img = np.copy(image_message).astype(float)
    rospy.sleep(5)

if __name__ == '__main__':
    rospy.loginfo("SaveImages is launched")
    rospy.init_node('vision_subscriber')
    sys.argv[1]
    image_storage_path = rospy.get_param("image_storage_path")
    rospy.Subscriber(f"/{sys.argv[1]}/image_raw", Image, callback)
    # rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    rospy.spin()