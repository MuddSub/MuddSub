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
       * @brief Publisher to camera/info topic
       *
       * Publishes sensor_msgs/CameraInfo messages
       */
       ros::Publisher camera_info_pub;

      /**
       * @brief Publisher to camera/image/raw topic
       *
       * Publishes sensor_msgs/Image messages
       */
      ros::Publisher raw_image_pub;

      /**
       * @brief Publisher to camera/image/greyscale topic
       *
       * Publishes sensor_msgs/Image messages
       */
      ros::Publisher greyscale_image_pub;

      /**
       * @brief Publisher to camera/image/compressed topic
       *
       * Publishes sensor_msgs/CompressedImage messages
       */
      ros::Publisher compressed_image_pub;

      /**
       * @brief Publisher to vision/bounding_boxes topic
       *
       * Publishes vision/BoundingBox2DArray messages
       */
      ros::Publisher bboxes_pub;

      /**
       * @brief Publisher to vision/detection topic
       *
       * Publishes vision/Detection messages
       */
      ros::Publisher detection_pub;

      /**
       * @brief Publisher to vision/detection_array topic
       *
       * Publishes vision/DetectionArray messages
       */
      ros::Publisher detection_array_pub;

      /**
       * @brief Node handle
       *
       * NodeHandle is the main access point to communications with the ROS system.
       */
      ros::NodeHandle n_;

    public:
      /**
       * @brief Creates a VisionPublisher instance
       *
       * @param n NodeHandle to create the Publishers with
       *
       * @return ControlsPublisher instance
       */
      VisionPublisher(ros::NodeHandle n);

      /**
       * @brief Use default copy constructor
       */
      VisionPublisher(const VisionPublisher&) = default;

      /**
       * @brief Publishes a sensor_msgs/CameraInfo message
       *
       * Uses the camera_info_pub Publisher to publish to camera/info
       *
       * @param cameraInfo The CameraInfo message to publish
       */
      void publishCameraInfo(sensor_msgs::CameraInfo& cameraInfo);

      /**
       * @brief Publishes a sensor_msgs/Image message
       *
       * Uses the raw_image_pub Publisher to publish to camera/image/raw
       *
       * @param rawImage The Image message to publish
       */
      void publishRawImage(sensor_msgs::Image& rawImage);

      /**
       * @brief Publishes a sensor_msgs/Image message
       *
       * Uses the greyscale_image_pub Publisher to publish to camera/image/greyscale
       *
       * @param greyscaleImage The Image message to publish
       */
      void publishGreyscaleImage(sensor_msgs::Image& greyscaleImage);

      /**
       * @brief Publishes a sensor_msgs/CompressedImage message
       *
       * Uses the compressed_image_pub Publisher to publish to camera/image/compressed
       *
       * @param compressedImage The CompressedImage message to publish
       */
      void publishCompressedImage(sensor_msgs::CompressedImage& compressedImage);

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

