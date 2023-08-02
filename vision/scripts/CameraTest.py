#!/usr/bin/env python3
from vision.Camera import Camera
import rospy

if __name__ == '__main__':
    rospy.loginfo("Hello, we started")
    obstacle_names = rospy.get_param("names")
    obstacle_names = [n.strip() for n in obstacle_names.split('\n')]
    test_image_path = rospy.get_param("test_image_path")

    model_path = rospy.get_param("model_path")
    model_config_path = rospy.get_param("model_config_path")

    camera_name = rospy.get_param("camera_name")
    test = Camera(camera_name, obstacle_names, model_path, model_config_path, test_image_path)
    test.main()

    # camera_name = "test_cam"
    # obstacle_names = ""
    # weights = "./models/7_16_2023/best_small.pt"
    # model_config_path = None
    # test_camera = Camera(camera_name, obstacle_names, weights, model_config_path)
