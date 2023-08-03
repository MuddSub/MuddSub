#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import torch
from torchvision import transforms
from torch.autograd import Variable
import numpy as np

from std_msgs.msg import Header, String, Float32
from vision.VisionPublisher import VisionPublisher
from vision.msg import Detection, DetectionArray, BoundingBoxArray, BoundingBox
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Vector3, Pose2D

from vision.YoloV5.yolov5.models.common import DetectMultiBackend
from vision.YoloV5.yolov5.utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)

from vision.Camera_Utils import detect_lines

from cv_bridge import CvBridge

bridge = CvBridge()
class Camera:

    def __init__(self, name, obstacle_names, weights, model_config_path, test_image_path):
        self.name = name
        self.camera_topic = "/" + self.name + "/image_raw"
        self.obstacle_names = obstacle_names
        self.weights = weights

        rospy.loginfo(self.weights)
        self.test_image = torch.tensor(cv2.imread(test_image_path)).permute(2,0,1).unsqueeze(0)
        print("test_image: ", self.test_image.shape)


    def main(self):
        rospy.init_node('vision_subscriber')

        self.visionPublisher = VisionPublisher(self.name)

        self.obstacle_names = rospy.get_param("names")
        self.obstacle_names = [n.strip() for n in self.obstacle_names.split('\n')]

        self.model_path = rospy.get_param("model_path")
        self.model_config_path = rospy.get_param("model_config_path")
        rospy.Subscriber(self.camera_topic, Image, self.callback)
        rospy.search_param
        rospy.spin()

    def callback(self, image):
        image_message = bridge.imgmsg_to_cv2(image, "bgr8")
        (H,W,C) = image_message.shape
        line_img, all_x = detect_lines(image_message, threshold=4, min_line_length=0)
        if len(all_x) >= 2: 
            center_all_x = sum(all_x) / len(all_x)
            normalized_x_center = center_all_x/W
            # print(normalized_x_center)
            self.visionPublisher.publishGateCenter(Float32(normalized_x_center))
            line_img = cv2.line(line_img, (int(center_all_x), 0), (int(center_all_x), H-1), (0, 255, 0), 2)  # Draw red lines
        else:
            print(f"not enough data. length of x's is {len(center_all_x)}")
        line_img = bridge.cv2_to_imgmsg(line_img, "bgr8")
        self.visionPublisher.publishModelOutput(line_img)


    def boundingBoxPublish(self, detection_output_list):
        """
        detection_output_list = list of detections, on (n,6) tensor per image [xyxy, conf, cls]
        """
        list_of_bounding_boxes = []
        for i in range(len(detection_output_list)):
            name = self.obstacle_names[int(detection_output_list[i][5])]
            xy = detection_output_list[i][0:4]
            object_xmin = xy[0]
            object_ymin = xy[1]
            object_xmax = xy[2]
            object_ymax = xy[3]
            object_center = Pose2D((object_xmax+object_xmin)/2, (object_ymax+object_ymin)/2, 0) #theta assumed be zero
            bbox = BoundingBox2D(object_center, (object_xmax-object_xmin)/2, (object_ymax-object_ymin)/2)
            confidence = detection_output_list[i, 4]
            bounding_box = BoundingBox(Header(), name, confidence, bbox)
            list_of_bounding_boxes.append(bounding_box)
        for bounding_box in list_of_bounding_boxes:
            self.visionPublisher.publishBoundingBox(bounding_box)
        bounding_box_array = BoundingBoxArray(Header(), list_of_bounding_boxes)
        self.visionPublisher.publishBoundingBoxArray(bounding_box_array)

if __name__ == "__main__":
    name = "test_cam"
    obstacle_names = ""
    weights = "./models/7_16_2023/best_small.pt"
    model_config_path = None
    test_camera = Camera(name, obstacle_names, weights, model_config_path)