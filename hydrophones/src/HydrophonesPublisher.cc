#include "hydrophones/HydrophonesPublisher.hh"

namespace MuddSub::Hydrophones
{
  HydrophonesPublisher::HydrophonesPublisher(ros::NodeHandle n):
    n_(n)
  {
    hydrophones_data_pub = n_.advertise<hydrophones::PingerData>("hydrophones/data", 1000);
  }

  void HydrophonesPublisher::publishData(hydrophones::PingerData& data)
  {
    hydrophones_data_pub.publish(data);
  }
}
