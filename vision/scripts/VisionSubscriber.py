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
from vision.msg import Detection, DetectionArray, BoundingBox2DArray
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
    
    # name = names[int(output[-1])]
    # get object names
    list_of_names = [names[int(i)] for i in output[:,-1]] 
    print("name is", list_of_names)
    rospy.loginfo(names)

    # get    0          1     2     3     4     5         6           7
    #       [image num, xmin, ymin, xmax, ymax, box conf, class conf, class num]

    list_of_xy = output[:,1:5]
    first_object_xmin = list_of_xy[0][0]
    first_object_ymin = list_of_xy[0][1]
    first_object_xmax = list_of_xy[0][2]
    first_object_ymax = list_of_xy[0][3]

    first_object_center = Pose2D((first_object_xmax+first_object_xmin)/2, (first_object_ymax+first_object_ymin)/2, 0)
    first_object_bounding_box = BoundingBox2D(first_object_center, (first_object_xmax-first_object_xmin)/2, (first_object_ymax-first_object_ymin)/2)

    

    # publish("-".join(list_of_names))
    # publish(str(len(detections)))
    # publish("-".join(output))
    publish(list_of_names[0], first_object_bounding_box)


def publish(obstacle_name, bounding_box): 
    obstacle1_header = Header()
    obstacle1_name = obstacle_name
    obstacle1_range = 1.0
    obstacle1_theta = 1.56
    obstacle1_phi = 0.2
    obstacle1_confidence = 0.8 #where 1 is 100%
    # bounding_box = BoundingBox2D(Pose2D(21.0, 21.0, 0.11), 51.0, 31.0)
    detection1 = Detection(obstacle1_header, obstacle1_name, obstacle1_range, obstacle1_theta, obstacle1_phi, obstacle1_confidence, bounding_box)
    visionPublisher.publishDetection(detection1)

def videofeed():
    rospy.init_node('vision_subscriber')
    # rospy.init_node('vision_example_node')
    visionPublisher = VisionPublisher("test_camera")

    rospy.Subscriber("/usb_cam/image_raw", Image, callback)

    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('vision_subscriber')
    # rospy.init_node('vision_example_node')
    visionPublisher = VisionPublisher("test_camera")
    # with open("~/catkin_ws/src/MuddSub/vision/scripts/class-names.txt") as f:
    #     names = f.read().split("\n")
    names = rospy.get_param("names")
    names = [n.strip() for n in names.split('\n')]

    model_path = rospy.get_param("model_path")
    model_config_path = rospy.get_param("model_config_path")
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    rospy.search_param
    rospy.spin()
