#!/usr/bin/env python3
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

from vision.YoloV5.yolov5.models.common import DetectMultiBackend
from vision.YoloV5.yolov5.utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)

bridge = CvBridge()
class Camera:

    def __init__(self, name, obstacle_names, weights, model_config_path, test_image_path):
        self.name = name
        self.camera_topic = "/" + self.name + "/image_raw"
        self.obstacle_names = obstacle_names
        self.weights = weights
        imgsz = (640, 640)

        device = torch.device("cuda:0" if torch.cuda.is_available else "cpu")

        # self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        rospy.loginfo(self.weights)
        self.model = DetectMultiBackend(weights, device=device, dnn=False, data=False, fp16=False)
        print(self.model)
        stride, names, pt = self.model.stride, self.model.names, self.model.pt # stride = 32
        imgsz = check_img_size(imgsz, s=stride)  # check image size
        bs = 1
        self.model.warmup(imgsz=(1 if pt or self.model.triton else bs, 3, *imgsz))  # warmup move this up
        seen, windows, self.dt = 0, [], (Profile(), Profile(), Profile())
        self.conf_thres = 0.25
        self.iou_thres = 0.45
        self.classes = None
        self.agnostic_nms = False
        self.max_det = 1000
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
        image_message = bridge.imgmsg_to_cv2(image, "rgb8")
        input_img = np.copy(image_message).astype(float)
        input_img = torch.from_numpy(input_img).float()
        input_img = Variable(input_img.type(torch.FloatTensor))

        input_img = input_img.permute(2,0,1)

        with self.dt[0]:
            # input_img = self.test_image
            im = input_img.to(self.model.device)
            im = im.half() if self.model.fp16 else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim

        # Inference
        with self.dt[1]:
            # visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False # False
            pred = self.model(im, augment=False, visualize=False) # False, False

        # NMS
        with self.dt[2]:
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)
        if len(pred) == 0:
            rospy.loginfo("No object.")
        else:
            print(pred)
            self.boundingBoxPublish(pred[0])

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