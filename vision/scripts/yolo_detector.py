#!/usr/bin/env python3
"""YOLO26s detection node for MuddSub AUV.

Subscribes to a camera image topic, runs YOLO26s inference,
and publishes detections using the existing VisionPublisher.
"""
import traceback
import rospy
from ultralytics import YOLO
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from vision.VisionPublisher import VisionPublisher
from vision.msg import Detection, DetectionArray, BoundingBox, BoundingBoxArray
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D


class YoloDetector:
    def __init__(self):
        rospy.init_node('yolo_detector')

        # Load parameters
        self.camera_name = rospy.get_param('~camera_name', 'left_camera')
        model_path = rospy.get_param('~model_path', 'yolo26s.pt')
        self.conf_threshold = rospy.get_param('~confidence_threshold', 0.25)
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.45)
        self.publish_debug = rospy.get_param('~publish_debug_image', True)

        # Initialize model
        rospy.loginfo(f"Loading YOLO model from {model_path}")
        self.model = YOLO(model_path)
        rospy.loginfo("YOLO model loaded successfully")

        # Initialize publisher and bridge
        self.vision_pub = VisionPublisher(self.camera_name)
        self.bridge = CvBridge()

        # Subscribe to camera — queue_size=1 drops stale frames,
        # large buff_size prevents buffering delays
        camera_topic = f"/{self.camera_name}/image_raw"
        rospy.Subscriber(
            camera_topic, Image, self.image_callback,
            queue_size=1, buff_size=2**24
        )
        rospy.loginfo(f"Subscribed to {camera_topic}")

    def image_callback(self, msg):
        """Process incoming camera frame through YOLO and publish detections."""
        try:
            # Convert ROS Image to BGR numpy array
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Run inference
            results = self.model.predict(
                source=frame,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                verbose=False
            )

            result = results[0]
            stamp = msg.header.stamp
            bounding_boxes = []
            detections = []

            if result.boxes is not None and len(result.boxes) > 0:
                xyxy = result.boxes.xyxy.cpu().numpy()
                xyxyn = result.boxes.xyxyn.cpu().numpy()
                confs = result.boxes.conf.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy().astype(int)

                for i in range(len(xyxy)):
                    class_name = result.names[classes[i]]
                    confidence = float(confs[i])

                    # Normalized center (0-1 range) for downstream mission nodes
                    # Uses xyxyn (already normalized by ultralytics)
                    center_x = (xyxyn[i][0] + xyxyn[i][2]) / 2.0
                    center_y = (xyxyn[i][1] + xyxyn[i][3]) / 2.0

                    # Half-width/height in pixels, matching existing Camera.py convention
                    size_x = (xyxy[i][2] - xyxy[i][0]) / 2.0
                    size_y = (xyxy[i][3] - xyxy[i][1]) / 2.0

                    bbox2d = BoundingBox2D(
                        Pose2D(center_x, center_y, 0),
                        size_x,
                        size_y
                    )

                    header = Header(stamp=stamp, frame_id=self.camera_name)

                    # Build and publish individual BoundingBox
                    bbox_msg = BoundingBox(header, class_name, confidence, bbox2d)
                    bounding_boxes.append(bbox_msg)
                    self.vision_pub.publishBoundingBox(bbox_msg)

                    # Build and publish individual Detection
                    # Range/angle estimation are stubs (same as current VisionOutput.py)
                    detection_msg = Detection(
                        header, class_name,
                        1.0,   # range — stub
                        0.0,   # theta — stub
                        0.0,   # phi — stub
                        confidence, bbox2d
                    )
                    detections.append(detection_msg)
                    self.vision_pub.publishDetection(detection_msg)

            # Publish arrays (even if empty — downstream may check for empty arrays)
            array_header = Header(stamp=stamp, frame_id=self.camera_name)
            self.vision_pub.publishBoundingBoxArray(
                BoundingBoxArray(array_header, bounding_boxes)
            )
            self.vision_pub.publishDetectionArray(
                DetectionArray(array_header, detections)
            )

            # Publish annotated debug image
            if self.publish_debug:
                annotated = result.plot()  # BGR numpy array with boxes drawn
                debug_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
                debug_msg.header = array_header
                self.vision_pub.publishModelOutput(debug_msg)

        except Exception as e:
            rospy.logerr(f"Error processing frame: {e}\n{traceback.format_exc()}")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = YoloDetector()
    node.run()
