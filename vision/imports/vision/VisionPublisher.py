#!/usr/bin/env python3
import rospy
#from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from vision.msg import Detection, DetectionArray, BoundingBoxArray, BoundingBox
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
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
        # self._bboxesPub = rospy.Publisher('vision/' + cameraName + '/bounding_boxes', BoundingBox2DArray, queue_size=10)
        self._detectionPub = rospy.Publisher('vision/' + cameraName + '/detection', Detection, queue_size=2)
        self._detectionArrayPub = rospy.Publisher('vision/' + cameraName + '/detection_array', DetectionArray, queue_size=2)
        self._boundingBoxPub = rospy.Publisher('vision/' + cameraName + '/bounding_box', BoundingBox, queue_size=2)
        self._boundingBoxArrayPub = rospy.Publisher('vision/' + cameraName + '/bounding_box_array', BoundingBoxArray, queue_size=2)
        self._gateCenterPub = rospy.Publisher('vision/' + cameraName + '/gate_center', Float32, queue_size=2)
        self._modelOutputPub = rospy.Publisher('vision/' + cameraName + '/model_output', Image, queue_size=2)
    # def publishBoundingBoxes(self, boundingBoxes):
    #     """
    #     Publish a vision/BoundingBox2DArray message.

    #     Uses the bboxesPub Publisher to publish to vision/{cameraName}/bounding_boxes topic.

    #     Args:
    #         boundingBoxes: The vision/BoundingBox2DArray message.
    #     """
    #     self._bboxesPub.publish(boundingBoxes)

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

    def publishBoundingBox(self, boundingBox):
        self._boundingBoxPub.publish(boundingBox)

    def publishBoundingBoxArray(self, boundingBoxArray):
        self._boundingBoxArrayPub.publish(boundingBoxArray)
    
    def publishGateCenter(self, normalized_x_center):
        """
        Publishes a std_msgs/Float32 representing a normalized value for the horizontal center of the gate.

        Args:
            normalized_x_center: a float between 0 and 1
        """
        self._gateCenterPub.publish(normalized_x_center)
    
    def publishModelOutput(self, model_output):
        """
        Publishes a sensor_msgs/Image showing the output of the vision model

        Args:
            model_output: any outputs of type sensor_msgs/Image
        """
        self._modelOutputPub.publish(model_output)

