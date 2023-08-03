#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
import os
from cv_bridge import CvBridge


class RosbagExtractor:
    def __init__(self, name, save_dir):
        self.bridge = CvBridge()
        self.name = name
        self.image_subscriber = rospy.Subscriber(f'/{name}/image_raw', Image, self.save_images)
        self.save_dir = save_dir
        self.counter = 0

    def save_images(self, image):
        image_message = self.bridge.imgmsg_to_cv2(image, "bgr8") # because cv2 will save in bgr format
        cv2.imwrite(os.path.join(save_dir, f"frame{self.counter:06d}.jpg"), image_message)
        print(f"finished the {self.counter}th image")
        self.counter += 1

if __name__ == "__main__":
    rospy.init_node('rosbag_extractor', anonymous=True)
    name = "left_camera"
    save_dir = "/media/muddsub/3338-3831/pool_run_images1"
    rosbag_extractor = RosbagExtractor(name, save_dir)
    rospy.spin()
