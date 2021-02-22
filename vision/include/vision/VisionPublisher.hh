#pragma once

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "vision/BoundingBox2DArray.h"
#include "vision/Detection.h"
#include "vision/DetectionArray.h"

namespace MuddSub::Vision
{
    class VisionPublisher
    {
        private:

      /**
       * @brief Publishes vision/BoundingBox2DArray messages to vision/bounding_boxes topic
       *
       */
      ros::Publisher bboxesPub_;

      /**
       * @brief Publishes vision/Detection messages to vision/detection topic
       *
       */
      ros::Publisher detectionPub_;

      /**
       * @brief Publishes vision/DetectionArray messages to vision/detection_array topic
       *
       */
      ros::Publisher detectionArrayPub_;

      /**
       * @brief Node handle is the main access point to communications with the ROS system.
       */
      ros::NodeHandle n_;

      /**
       * @brief The name of the camera that took the image that will be used for the detection and bounding boxes.
       *
       */
      std::string cameraName_;

    public:
      /**
       * @brief Creates a VisionPublisher instance
       *
       * @param n NodeHandle to create the Publishers with
       * @param cameraName The name of the camera to use in the topic names
       *
       * @return ControlsPublisher instance
       */
      VisionPublisher(ros::NodeHandle n, std::string cameraName);

      /**
      * @brief Explicitely deteleted constructor
      */
      VisionPublisher() = delete;

      /**
       * @brief Use default copy constructor
       */
      VisionPublisher(const VisionPublisher&) = default;

      /**
       * @brief Publishes a vision/BoundingBox2DArray message
       *
       * Uses the bboxes_pub Publisher to publish to vision/bounding_boxes
       *
       * @param boundingbox The BoundingBox2DArray message to publish
       */
      void publishBoundingBox(vision::BoundingBox2DArray& boundingBox);

      /**
       * @brief Publishes a vision/Detection message
       *
       * Uses the detection_pub Publisher to publish to vision/detection
       *
       * @param detection The Detection message to publish
       */
      void publishDetection(vision::Detection& detection);

      /**
       * @brief Publishes a vision/DetectionArray message
       *
       * Uses the detection_array_pub Publisher to publish to vision/detection_array
       *
       * @param detectionArray The DetectionArray message to publish
       */
      void publishDetectionArray(vision::DetectionArray& detectionArray);
  };
} // namespace MuddSub::Vision
