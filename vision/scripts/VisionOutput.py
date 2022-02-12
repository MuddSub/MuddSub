#!/usr/bin/env python3
import rospy
import sys
from vision.msg import Detection, DetectionArray, BoundingBoxArray, BoundingBox
from vision.VisionPublisher import VisionPublisher
from std_msgs.msg import Header, String


#range publishing template

def callback(data):
    """Adds range estimation data to obstacle detection and publishes DetectionArray.

    Args:
        data: obstacle detection in the form of a BoundingBoxArray.msg.
    """
    detectionList = []
    for currentBoundingBox in data.bounding_boxes:
        range = estimate_range(currentBoundingBox)
        theta, phi = estimate_angles(currentBoundingBox)
        detectionList += [Detection(Header(), 
                            currentBoundingBox.name, 
                            range, 
                            theta, 
                            phi, 
                            currentBoundingBox.confidence, 
                            currentBoundingBox.boundingBox)]
    
    detectionArray = DetectionArray(Header(), detectionList)

    visionPublisher.publishDetectionArray(detectionArray)

def estimate_range(boundingBox):
    """Estimates range to an obstacle.

    Args:
        boundingBox: the boundingBox of the obstacle in the form of a BoundingBox.msg.

    Return:
        The distance to the obstacle in meters.
    """
    return 15

def estimate_angles(boundingBox):
    """Estimates theta and phi of an obstacle relative to the camera.
    
    Args:
        boundingBox: the boundingBox of the obstacle in the form of a BoundingBox.msg.

    Return:
        Theta is the horizontal, then phi is the vertical angle in radians.
    """
    return 10, 20
    
if __name__ == '__main__':
    camera_name = sys.argv[0]
    rospy.init_node('vision_output')
    visionPublisher = VisionPublisher(camera_name)
    # rospy.Subscriber("/vision/test_camera/bounding_box_array", BoundingBoxArray, callback)
    rospy.Subscriber(f"/vision/{camera_name}/bounding_box_array", BoundingBoxArray, callback)
    rospy.spin()
    
    
