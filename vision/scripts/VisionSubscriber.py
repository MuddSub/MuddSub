#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import torch
from torch.autograd import Variable
import numpy as np

bridge = CvBridge()

def callback(image):
    image_message = bridge.imgmsg_to_cv2(image, "rgb8")
    input_img = np.copy(image_message).astype(float)
    input_img = torch.from_numpy(input_img).float()
    print(type(input_img))
    input_img = Variable(input_img.type(torch.FloatTensor))
    print(input_img)

def videofeed():
    rospy.init_node('vision_subscriber')

    rospy.Subscriber("/usb_cam/image_raw", Image, callback)

    rospy.spin()

if __name__ == '__main__':
    videofeed()