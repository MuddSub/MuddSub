#include "vision/VisionPublisher.hh"

namespace MuddSub::Vision
{
  VisionPublisher::VisionPublisher(ros::NodeHandle n, std::string cameraName):
    n_(n), cameraName_(cameraName)
  {
    camera_info_pub = n_.advertise<sensor_msgs::CameraInfo>("vision/" + cameraName_ + "/info", 1000);
    raw_image_pub = n_.advertise<sensor_msgs::Image>("vision/" + cameraName_ + "/image/raw", 1000);
    greyscale_image_pub = n_.advertise<sensor_msgs::Image>("vision/" + cameraName_ + "/image/greyscale", 1000);
    compressed_image_pub = n_.advertise<sensor_msgs::CompressedImage>("vision/" + cameraName_ + "/image/compressed", 1000);
    bboxes_pub = n_.advertise<vision::BoundingBox2DArray>("vision/" + cameraName_ + "/bounding_boxes", 1000);
    detection_pub = n_.advertise<vision::Detection>("vision/" + cameraName_ + "/detection", 1000);
    detection_array_pub = n_.advertise<vision::DetectionArray>("vision/detection_array", 1000);
  }

  void VisionPublisher::publishCameraInfo(sensor_msgs::CameraInfo& cameraInfo)
  {
    camera_info_pub.publish(cameraInfo);
  }
  void VisionPublisher::publishRawImage(sensor_msgs::Image& rawImage)
  {
    raw_image_pub.publish(rawImage);
  }
  void VisionPublisher::publishGreyscaleImage(sensor_msgs::Image& greyscaleImage)
  {
    greyscale_image_pub.publish(greyscaleImage);
  }
  void VisionPublisher::publishCompressedImage(sensor_msgs::CompressedImage& compressedImage)
  {
    compressed_image_pub.publish(compressedImage);
  }
  void VisionPublisher::publishBoundingBox(vision::BoundingBox2DArray& boundingBox)
  {
    bboxes_pub.publish(boundingBox);
  }
  void VisionPublisher::publishDetection(vision::Detection& detection)
  {
    detection_pub.publish(detection);
  }
  void VisionPublisher::publishDetectionArray(vision::DetectionArray& detectionArray)
  {
    detection_array_pub.publish(detectionArray);
  }
}
