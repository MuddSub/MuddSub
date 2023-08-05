#include "vision/VisionPublisher.hh"

namespace MuddSub::Vision
{
  VisionPublisher::VisionPublisher(ros::NodeHandle n, std::string cameraName):
    n_(n), cameraName_(cameraName)
  {
    // bboxesPub_ = n_.advertise<vision::BoundingBox2DArray>("vision/" + cameraName_ + "/bounding_boxes", 1000);
    detectionPub_ = n_.advertise<vision::Detection>("vision/" + cameraName_ + "/detection", 1000);
    detectionArrayPub_ = n_.advertise<vision::DetectionArray>("vision/detection_array", 1000);
  }

  // void VisionPublisher::publishBoundingBox(vision::BoundingBox2DArray& boundingBox)
  // {
  //   bboxesPub_.publish(boundingBox);
  // }
  void VisionPublisher::publishDetection(vision::Detection& detection)
  {
    detectionPub_.publish(detection);
  }
  void VisionPublisher::publishDetectionArray(vision::DetectionArray& detectionArray)
  {
    detectionArrayPub_.publish(detectionArray);
  }
}
