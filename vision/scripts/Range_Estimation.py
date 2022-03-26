#!/usr/bin/env python3
import numpy as np
import rospy
from vision.msg import BoundingBox
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from std_msgs.msg import Header, String
from vision.msg import Detection, DetectionArray, BoundingBoxArray, BoundingBox
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Vector3, Pose2D


class Range_Estimation():
    def __init__(self):
        rospy.init_node('range_estimation')
        self.depthMapSub  = rospy.Subscriber('/zed2/zed_node/depth/depth_registered', Image, self.callback)
        self.bridge = CvBridge()
        self.CAMERA_WIDTH = 1280
        self.CAMERA_HEIGHT = 720
        self.depth_map = None
    #    self.sample_depth_map = np.random.rand(self.CAMERA_HEIGHT, self.CAMERA_WIDTH)
        self.sample_depth_map = np.ones((self.CAMERA_HEIGHT, self.CAMERA_WIDTH))

        #making a testing bounding box

        # creating the name of the detection
        name = "Gate"

        # creating the confidance value
        confidence = 0.80

        # creating the bounding box
        # the left top corner I think
        object_xmin = 0.1 #these values go from 0 to 1
        object_ymin = 0.1
        # the right bot corner I think
        object_xmax = 0.9 
        object_ymax = 0.9
        object_center = Pose2D((object_xmax+object_xmin)/2, (object_ymax+object_ymin)/2, 0)
        box = BoundingBox2D(object_center, (object_xmax-object_xmin)/2, (object_ymax-object_ymin)/2)
        
        # creating the final BoundingBox message from the name, confidance, and box
        self.boundingBox = BoundingBox(Header(), name, confidence, box)


    def callback(self, image):
        image_message = self.bridge.imgmsg_to_cv2(image, "32FC1")
        input_img = np.array(image_message,dtype = np.dtype('f8'))
        self.depth_map = input_img
        print(input_img[self.CAMERA_WIDTH/2][self.CAMERA_HEIGHT/2])
        print(input_img.shape)

    def estimate_range(self, boundingBox):
        """Estimates range to an obstacle.

    Args:
        boundingBox: the boundingBox of the obstacle in the form of a BoundingBox.msg.

    Return:
        The distance to the obstacle in meters.
        """
        # 1. extract the bounding box from the boundingBox parameter
        data = boundingBox.boundingBox
        center = data.center
        x_len = data.size_x
        y_len = data.size_y
        dimensions = np.array((self.CAMERA_HEIGHT, self.CAMERA_WIDTH))
        top_left_corner  = np.array((center.y - y_len, center.x - x_len)) * dimensions
        top_right_corner = np.array((center.y - y_len, center.x + x_len)) * dimensions
        bottom_left_corner = np.array((center.y + y_len, center.x - x_len)) * dimensions
        bottom_right_corner = np.array((center.y + y_len, center.x + x_len)) * dimensions

        x_min = int((center.x - x_len)* dimensions[1])
        x_max = int((center.x + x_len) * dimensions[1])
        y_min = int((center.y - y_len) * dimensions[0])
        y_max = int((center.y + y_len) * dimensions[0])
        # 2. using indexing to pull out all the elements of the depth map that
        #    falls within the bounding box
        bounding_box_with_depth = self.sample_depth_map[y_min: y_max + 1, x_min: x_max + 1]

        # 3. Find the num_element min points and take average
        num_elements = int(bounding_box_with_depth.size * 0.1)
        print(bounding_box_with_depth)
        print(len(bounding_box_with_depth.flatten()))
        print(num_elements)
        min_indices = np.argpartition(bounding_box_with_depth.flatten(), num_elements)[:num_elements]
        print(min_indices)
        print(len(min_indices))
        min_points = bounding_box_with_depth.flatten()[min_indices]
        # 4. return
        return np.mean(min_points)
    
    def estimate_angles(boundingBox):
        return 10, 20   

if __name__ == "__main__":
    range = Range_Estimation()
    print(range.estimate_range(range.boundingBox))
    rospy.spin()


