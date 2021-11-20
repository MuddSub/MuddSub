#!/usr/bin/env python3
import rospy
#from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from vision.msg import Detection, DetectionArray, BoundingBox2DArray

class VisionPublisher:
    """
    Publishes vision messages describing obstacles detected by the computer vision algorithm.

    The collection of all bounding boxes surrounding their respective objects are described by vision/BoundingBox2DArray messages published to the vision/{cameraName}/bounding_boxes topic.

    The label, range, bearing,and confidence of a single detected obstacle is described by vision/Detection messages published to the vision/{cameraName}/detection topic.

    All collection of all vision/Detection messages is described by the vision/DetectionArray messages published to the vision/detection_array topic.

    Attributes:
        cameraName: The name of the camera that captured the obstacle
        bboxesPub: Publisher to vision/{cameraName}/bounding_boxes topic. Publishes vision/BoundingBox2DArray messages.
        detectionPub: Publisher to vision/{cameraName}/detection topic. Publishes vision/Detection messages.
        detectionArrayPub: Publisher to vision/detection_array topic. Publishes vision/DetectionArray messages.
    """

    def __init__(self, cameraName):
        """Initialize VisionPublisher instance."""
        self._cameraName = cameraName
        self._bboxesPub = rospy.Publisher('vision/' + cameraName + '/bounding_boxes', BoundingBox2DArray, queue_size=10)
        self._detectionPub = rospy.Publisher('vision/' + cameraName + '/detection', Detection, queue_size=10)
        self._detectionArrayPub = rospy.Publisher('vision/detection_array', DetectionArray, queue_size=10)

    def publishBoundingBoxes(self, boundingBoxes):
        """
        Publish a vision/BoundingBox2DArray message.

        Uses the bboxesPub Publisher to publish to vision/{cameraName}/bounding_boxes topic.

        Args:
            boundingBoxes: The vision/BoundingBox2DArray message.
        """
        self._bboxesPub.publish(boundingBoxes)

    def publishDetection(self, detection):
        """
        Publishes a vision/Detection message.

        Uses the detectionPub Publisher to publish to vision/{cameraName}/detection topic.

        Args:
            detection: The vision/Detection message.
        """
        self._detectionPub.publish(detection)

    def publishDetectionArray(self, detectionArray):
        """
        Publishes a vision/DetectionArray message.

        Uses the detection_array Publisher to publish to vision/detection_array topic.

        Args:
            detectionArray: The vision/DetectionArray message.
        """
        self._detectionArrayPub.publish(detectionArray)
