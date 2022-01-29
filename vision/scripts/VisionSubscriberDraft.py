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

def callback(image):
    image_message = bridge.imgmsg_to_cv2(image, "rgb8")
    input_img = np.copy(image_message).astype(float)
    input_img = torch.from_numpy(input_img).float()
    print(type(input_img))
    input_img = Variable(input_img.type(torch.FloatTensor))
    print(input_img)
    print("input_img.size(): ", input_img.size())
    input_img = input_img.permute(2,0,1)
    print(input_img.size())
# ______________________________________________________
    det = Detector(model_path, model_config_path)
    transform = transforms.Compose([transforms.Resize((256,256)),
                                transforms.Normalize(0.5,0.5)])
    input_img = transform(input_img)
    input_img = input_img.unsqueeze(0)
    output, _ = det.get_detections(input_img, conf_thresh=.99, nms_thresh=0)
    
    sample_output = [[1, 15, 51, 14, 41, 0.6, 0.9, 3] ,     # 3 = Bins (2018)
                     [2, 10, 20, 30, 40, 0,   0,   11],     # 11 = Gate (2018)
                     [3, 5,  6,  35, 55, 0.7, 0.9, 17]]     # 17 = Torpedo - Board (2019)
    sample_output = torch.FloatTensor(sample_output)
    boundingBoxPublish(output)

def boundingBoxPublish(detection_output_list):
    list_of_bounding_boxes = []
    for i in range(len(detection_output_list)):
        name = names[int(detection_output_list[i, 7])]
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
        visionPublisher.publishBoundingBox(bounding_box)
    bounding_box_array = BoundingBoxArray(Header(), list_of_bounding_boxes)
    visionPublisher.publishBoundingBoxArray(bounding_box_array)

# def videofeed():
#     rospy.init_node('vision_subscriber')
#     visionPublisher = VisionPublisher("test_camera")

#     rospy.Subscriber("/usb_cam/image_raw", Image, callback)

#     rospy.spin()

if __name__ == '__main__':
    rospy.init_node('vision_subscriber')

    visionPublisher = VisionPublisher("test_camera")

    names = rospy.get_param("names")
    names = [n.strip() for n in names.split('\n')]

    model_path = rospy.get_param("model_path")
    model_config_path = rospy.get_param("model_config_path")
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    rospy.search_param
    rospy.spin()
