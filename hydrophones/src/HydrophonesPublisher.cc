#include "hydrophones/HydrophonesPublisher.hh"

namespace MuddSub::Hydrophones
{
  HydrophonesPublisher::HydrophonesPublisher(ros::NodeHandle n):
    n_(n)
  {
    hydrophonesDataPub_ = n_.advertise<hydrophones::PingerData>("hydrophones/data", 1000);
  }

  void HydrophonesPublisher::publishData(hydrophones::PingerData& data)
  {
    hydrophonesDataPub_.publish(data);
  }
}
