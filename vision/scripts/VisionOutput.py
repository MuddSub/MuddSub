#!/usr/bin/env python3
import rospy
from vision.msg import Detection, DetectionArray, BoundingBoxArray, BoundingBox
from vision.VisionPublisher import VisionPublisher
from std_msgs.msg import Header, String


#range publishing template


'''
Header header
string name
float64 range
float64 theta
float64 phi
float64 confidence
vision_msgs/BoundingBox2D boundingBox
'''
'''
obstacle1_header = Header()
obstacle1_name = obstacle_name
obstacle1_range = 1.0
obstacle1_theta = 1.56
obstacle1_phi = 0.2
obstacle1_confidence = 0.8 #where 1 is 100%
# bounding_box = BoundingBox2D(Pose2D(21.0, 21.0, 0.11), 51.0, 31.0)
detection1 = Detection(obstacle1_header, obstacle1_name, obstacle1_range, obstacle1_theta, obstacle1_phi, obstacle1_confidence, bounding_box)
visionPublisher.publishDetection(detection1)'''
def callback(data):
    #range estimation stuff happens
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
    return 15

def estimate_angles(boundingBox):
    '''returns theta, then returns phi in radians'''
    return 10, 20
    
if __name__ == '__main__':
    rospy.init_node('vision_output')
    visionPublisher = VisionPublisher("test_camera")
    rospy.Subscriber("/vision/test_camera/bounding_box_array", BoundingBoxArray, callback)
    rospy.spin()
    
    
