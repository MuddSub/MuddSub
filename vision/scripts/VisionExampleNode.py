#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header, String
from vision.VisionPublisher import VisionPublisher
from vision.msg import Detection, DetectionArray
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Vector3, Pose2D
import dynamic_reconfigure.client

def update(config):
    #Add whatever needs to be updated when configuration parameters change here
    rospy.loginfo("""Config set to {example_param}""".format(**config))

def visionPubNode():
    rospy.init_node('vision_example_node', anonymous=True)
    visionPublisher = VisionPublisher("test_camera")
    client = dynamic_reconfigure.client.Client("vision_server", timeout=30, config_callback=update)
    client.update_configuration({})

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        #sets up params which is a dictionary filled with values obtained from the dynamic reconfigure gui
        params = client.get_configuration(timeout=10)

        #Create Header message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'vision'

        #you can access values set in param like this
        example_param = params["example_param"]
        print(example_param)

        # Create Detection message

        name = "gate"
        range = 10.0
        theta = 0.05  #radians or degrees
        phi = 1.0
        confidence = 0.7

        boundingBox = BoundingBox2D()

        #making center message
        boundingBox.center = Pose2D()
        boundingBox.center.x = 2.0
        boundingBox.center.y = 2.0
        boundingBox.center.theta = 0.1

        boundingBox.size_x = 5.0
        boundingBox.size_y = 3.0

        detection = Detection(header, name, range, theta, phi, confidence, boundingBox)
        visionPublisher.publishDetection(detection)

        # Create second Detection message

        name = "dragon"
        range = 100.0
        theta = 0.055  #radians or degrees
        phi = 11.0
        confidence = 0.17

        boundingBox = BoundingBox2D()

        #making center message
        boundingBox.center = Pose2D()
        boundingBox.center.x = 21.0
        boundingBox.center.y = 21.0
        boundingBox.center.theta = 0.11

        boundingBox.size_x = 51.0
        boundingBox.size_y = 31.0

        detection2 = Detection(header, name, range, theta, phi, confidence, boundingBox)

        list_of_detections = [detection, detection2]

        detectionArray = DetectionArray(header, list_of_detections)
        visionPublisher.publishDetectionArray(detectionArray)


        # #Create BoundingBox2DArray message
        # box1_center = Pose2D(1.0, 2.0, 0) # in the order of x, y, and theta (in radian)
        # box1 = BoundingBox2D(box1_center, 4.0, 5.0)
        # box2_center = Pose2D(2.0, 3.0, 0)
        # box2 = BoundingBox2D(box2_center, 3.0, 5.0)
        # all_bounding_boxes = [box1, box2]
        # boundingBoxes = BoundingBox2DArray(header, all_bounding_boxes)

        # #Create 2 Detection messages
        # obstacle1_header = Header()
        # obstacle1_header.frame_id = "gate"
        # obstacle1_range = 1.0
        # obstacle1_theta = 1.56
        # obstacle1_phi = 0.2
        # obstacle1_confidence = 0.8 #where 1 is 100%
        # detection1 = Detection(obstacle1_header, obstacle1_range, obstacle1_theta, obstacle1_phi, obstacle1_confidence)

        # obstacle2_header = Header()
        # obstacle2_header.frame_id = "ball"
        # obstacle2_range = 2.0
        # obstacle2_theta = 1.23
        # obstacle2_phi = 3.13
        # obstacle2_confidence = 0.7 #where 1 is 100%
        # detection2 = Detection(obstacle2_header, obstacle2_range, obstacle2_theta, obstacle2_phi, obstacle2_confidence)

        # #Create DetectionArray messages
        # allDetections = [detection1, detection2]
        # detectionArray = DetectionArray(header, allDetections)

        # # Publish messages
        # visionPublisher.publishBoundingBoxes(boundingBoxes)
        # visionPublisher.publishDetection(detection1)
        # visionPublisher.publishDetection(detection2)
        # visionPublisher.publishDetectionArray(detectionArray)
        rate.sleep()

    # def create_yaml():
    #     f = open("hi.yaml", w)
        
    # rospy.on_shutdown(create_yaml)

if __name__ == '__main__':
    try:
        visionPubNode()
    except rospy.ROSInterruptException:
        pass
