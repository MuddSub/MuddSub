#pragma once

#include "ros/ros.h"
#include "hydrophones/PingerData.h"

namespace MuddSub::Hydrophones
{
    class HydrophonesPublisher
    {
        private:
      /**
       * @brief Publisher hydrophones/PingerData messages to hydrophones/data topic
       *
       */
       ros::Publisher hydrophonesDataPub_;

     /**
       * @brief Node handle, the main access point to communications with the ROS system.
       */
      ros::NodeHandle n_;

    public:
      /**
       * @brief Creates a HydrophonesPublisher instance
       *
       * @param n NodeHandle to create the Publishers with
       *
       * @return HydrophonesPublisher instance
       */
      HydrophonesPublisher(ros::NodeHandle n);

      /**
       * @brief Use default copy constructor
       */
      HydrophonesPublisher(const HydrophonesPublisher&) = default;

      /**
      * @brief Explicitly deleted constructor
      */
      HydrophonesPublisher() = delete;

      /**
       * @brief Publishes a hydrophones/PingerData message
       *
       * Uses the hydrophones_data_pub Publisher to publish to hydrophones/data
       *
       * @param data The PingerData message to publish
       */
      void publishData(hydrophones::PingerData& data);
    };
}// namespace MuddSub::Hydrophones
