#!/usr/bin/env python3
from xmlrpc.client import DateTime
from datetime import datetime
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from torchvision import transforms
from torch.autograd import Variable
import numpy as np
import cv2
from std_msgs.msg import Header, String
import os

bridge = CvBridge()

def callback(image):
    image_message = bridge.imgmsg_to_cv2(image, "bgr8")
    print("Current working directory: {0}".format(os.getcwd()))
    now = datetime.now() # current date and time

    year = now.strftime("%Y")
    print("year:", year)

    month = now.strftime("%m")
    print("month:", month)

    day = now.strftime("%d")
    print("day:", day)

    time = now.strftime("%H:%M:%S")
    print("time:", time)

    date_time = now.strftime("%m/%d/%Y,%H:%M:%S")
    print("date and time:",date_time)	
    print("name is " + str(date_time)+".jpg")
    image_name = str(date_time) + ".jpg"

    cv2.imwrite("hi.jpg",image_message)
    # input_img = np.copy(image_message).astype(float)

if __name__ == '__main__':
    rospy.loginfo("SaveImages is launched")
    rospy.init_node('vision_subscriber')
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    rospy.spin()
