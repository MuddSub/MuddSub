#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from std_msgs.msg import Header, String
from vision.VisionPublisher import VisionPublisher
from vision.msg import Detection, DetectionArray, BoundingBoxArray, BoundingBox
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Vector3, Pose2D


def sampleDetectionPublisher():
    visionPublisher = VisionPublisher("left_camera")
    rospy.init_node('sample_bbox_publisher')
    while not rospy.is_shutdown():

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
        boundingBox = BoundingBox(Header(), name, confidence, box)

        # publishes to the topic called 'vision/left_camera/bounding_box'
        visionPublisher.publishBoundingBox(boundingBox)

        # A second BoundingBox message
        name2 = "Buoy"
        confidence2 = 0.50
        object_xmin2 = 0.5
        object_ymin2 = 0.4
        object_xmax2 = 0.8
        object_ymax2 = 0.7
        object_center2 = Pose2D((object_xmax2+object_xmin2)/2, (object_ymax2+object_ymin2)/2, 0)
        box2 = BoundingBox2D(object_center2, (object_xmax2-object_xmin2)/2, (object_ymax2-object_ymin2)/2)
        boundingBox2 = BoundingBox(Header(), name2, confidence2, box2)
        visionPublisher.publishBoundingBox(boundingBox2)

        # An array of boundingboxes. This is what you should parse to find range to each object.
        boundingBoxArray = BoundingBoxArray(Header(), [boundingBox, boundingBox2])
        visionPublisher.publishBoundingBoxArray(boundingBoxArray)


if __name__ == '__main__':
    sampleDetectionPublisher()