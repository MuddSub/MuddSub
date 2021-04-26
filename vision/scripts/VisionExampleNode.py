#!/usr/bin/env python

import rospy
from std_msgs.msg import Header, String
from VisionPublisher import VisionPublisher
from vision.msg import Detection, DetectionArray, BoundingBox2DArray
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Vector3, Pose2D
import dynamic_reconfigure.client

# params is a dictionary that you can use to set up parameter values
# params = {"double_param": 0.0}

params = {} #usually the default starting params is just {} for our purposes

def update(config):
    global params
    rospy.loginfo("""Config set to {example_param}""".format(**config))
    params = config

def visionPubNode():
    rospy.init_node('vision_example_node', anonymous=True)
    visionPublisher = VisionPublisher("test_camera")
    client = dynamic_reconfigure.client.Client("vision_server", timeout=30, config_callback=update)
    client.update_configuration({})

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        #Create Header message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'vision'

        #you can access values set in param like this
        print(len(params))
        example_param = params["example_param"]
        print(example_param)

        #Create BoundingBox2DArray message
        box1_center = Pose2D(1.0, 2.0, 0) # in the order of x, y, and theta (in radian)
        box1 = BoundingBox2D(box1_center, 4.0, 5.0)
        box2_center = Pose2D(2.0, 3.0, 0)
        box2 = BoundingBox2D(box2_center, 3.0, 5.0)
        all_bounding_boxes = [box1, box2]
        boundingBoxes = BoundingBox2DArray(header, all_bounding_boxes)

        #Create 2 Detection messages
        obstacle1_header = Header()
        obstacle1_header.frame_id = "gate"
        obstacle1_range = 1.0
        obstacle1_theta = 1.56
        obstacle1_phi = 0.2
        obstacle1_confidence = 0.8 #where 1 is 100%
        detection1 = Detection(obstacle1_header, obstacle1_range, obstacle1_theta, obstacle1_phi, obstacle1_confidence)

        obstacle2_header = Header()
        obstacle2_header.frame_id = "ball"
        obstacle2_range = 2.0
        obstacle2_theta = 1.23
        obstacle2_phi = 3.13
        obstacle2_confidence = 0.7 #where 1 is 100%
        detection2 = Detection(obstacle2_header, obstacle2_range, obstacle2_theta, obstacle2_phi, obstacle2_confidence)

        #Create DetectionArray messages
        allDetections = [detection1, detection2]
        detectionArray = DetectionArray(header, allDetections)

        # Publish messages
        visionPublisher.publishBoundingBoxes(boundingBoxes)
        visionPublisher.publishDetection(detection1)
        visionPublisher.publishDetection(detection2)
        visionPublisher.publishDetectionArray(detectionArray)
        rate.sleep()

if __name__ == '__main__':
    try:
        visionPubNode()
    except rospy.ROSInterruptException:
        pass
