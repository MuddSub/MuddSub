#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from vision.msg import Detection, DetectionArray, BoundingBox2DArray


class VisionPublisher:
    def __init__(self, cameraName):
        self.cameraName = cameraName
        self.camera_info_pub = rospy.Publisher('vision/' + cameraName + '/info', CameraInfo, queue_size=10)
        self.raw_image_pub = rospy.Publisher('vision/' + cameraName + '/image/raw', Image, queue_size=10)
        self.greyscale_image_pub = rospy.Publisher('vision/' + cameraName + '/image/raw', Image, queue_size=10)
        self.compressed_image_pub = rospy.Publisher('vision/' + cameraName + '/image/compressed', CompressedImage, queue_size=10)
        self.bboxes_pub = rospy.Publisher('vision/' + cameraName + '/bounding_boxes', BoundingBox2DArray, queue_size=10)
        self.detection_pub = rospy.Publisher('vision/' + cameraName + '/detection', Detection, queue_size=10)
        self.detection_array_pub = rospy.Publisher('vision/detection_array', DetectionArray, queue_size=10)

    def publishCameraInfo(self, cameraInfo):
        self.camera_info_pub.publish(cameraInfo)

    def publishRawImage(self, rawImage):
        self.raw_image_pub.publish(rawImage)

    def publishGreyscaleImage(self, greyscaleImage):
        self.greyscale_image_pub.publish(greyscaleImage)

    def publishCompressedImage(self, compressedImage):
        self.compressed_image_pub.publish(compressedImage)

    def publishBoundingBox(self, boundingBox):
        self.bboxes_pub.publish(boundingBox)

    def publishDetection(self, detection):
        self.detection_pub.publish(detection)

    def publishDetectionArray(self, detectionArray):
        self.detection_array_pub.publish(detectionArray)
