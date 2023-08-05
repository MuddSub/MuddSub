#!/usr/bin/env python
from vision.MSCV2.network.util import nms_prediction
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import torch
from torchvision import transforms
from torch.autograd import Variable
import numpy as np

from std_msgs.msg import Header, String
from vision.VisionPublisher import VisionPublisher
from vision.msg import Detection, DetectionArray, BoundingBoxArray, BoundingBox
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Vector3, Pose2D

from vision.MSCV2.network.detector import Detector
from vision.MSCV2.network.model import NetworkModel

bridge = CvBridge()

class Camera:

    def __init__(self, name, obstacle_names, model_path, model_config_path):
        self.name = name
        self.camera_topic = "/" + self.name + "/image_raw"
        self.obstacle_names = obstacle_names
        self.model_path = model_path
        self.model_config_path = model_config_path

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
        image_message = bridge.imgmsg_to_cv2(image, "rgb8")
        input_img = np.copy(image_message).astype(float)
        input_img = torch.from_numpy(input_img).float()
        # print(type(input_img))
        input_img = Variable(input_img.type(torch.FloatTensor))
        # print(input_img)
        # print("input_img.size(): ", input_img.size())
        input_img = input_img.permute(2,0,1)
        # print(input_img.size())
    # ______________________________________________________
        det = Detector(self.model_path, self.model_config_path)
        transform = transforms.Compose([transforms.Resize((256,256)),
                                    transforms.Normalize(0.5,0.5)])
        input_img = transform(input_img)
        input_img = input_img.unsqueeze(0)
        output, _ = det.get_detections(input_img, conf_thresh=0.5, nms_thresh=0.3)
        if len(output) == 0:
            rospy.loginfo("No object.")
        # sample_output = [[1, 15, 51, 14, 41, 0.6, 0.9, 3] ,     # 3 = Bins (2018)
        #                 [2, 10, 20, 30, 40, 0,   0,   11],     # 11 = Gate (2018)
        #                 [3, 5,  6,  35, 55, 0.7, 0.9, 17]]     # 17 = Torpedo - Board (2019)
        # sample_output = torch.FloatTensor(sample_output)
        # self.boundingBoxPublish(sample_output)
        self.boundingBoxPublish(output)

    def boundingBoxPublish(self, detection_output_list):
        list_of_bounding_boxes = []
        for i in range(len(detection_output_list)):
            name = self.obstacle_names[int(detection_output_list[i, 7])]
            xy = detection_output_list[i,1:5]
            object_xmin = xy[0]
            object_ymin = xy[1]
            object_xmax = xy[2]
            object_ymax = xy[3]
            object_center = Pose2D((object_xmax+object_xmin)/2, (object_ymax+object_ymin)/2, 0) #theta assumed be zero
            bbox = BoundingBox2D(object_center, (object_xmax-object_xmin)/2, (object_ymax-object_ymin)/2)
            confidence = detection_output_list[i, 6]
            bounding_box = BoundingBox(Header(), name, confidence, bbox)
            list_of_bounding_boxes.append(bounding_box)
        for bounding_box in list_of_bounding_boxes:
            self.visionPublisher.publishBoundingBox(bounding_box)
        bounding_box_array = BoundingBoxArray(Header(), list_of_bounding_boxes)
        self.visionPublisher.publishBoundingBoxArray(bounding_box_array)

