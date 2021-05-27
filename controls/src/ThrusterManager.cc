#include "controls/ThrusterManager.hh"

/**
 * This is merely a way to test the ThrusterManager.inl implementation
 */
int main (int argc, char** argv)
{
    ros::init(argc, argv, "ThrusterManager");

    MuddSub::Controls::ThrusterManager<6, 8> manager;
    MuddSub::Controls::ThrusterManager<6, 8>::ForceVector_t force;
    // Just an example
    force << 300, -50, 10, 0, 0, 0;
    manager.calculateThrust(force);
    
    ros::Rate loopRate = 30;
    while(ros::ok())
    {
        ros::spinOnce();
        loopRate.sleep();
    }
}