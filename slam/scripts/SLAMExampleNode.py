#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from slam.SLAMPublisher import SLAMPublisher
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped
from slam.msg import Map
import dynamic_reconfigure.client

def update(config):
    #Add whatever needs to be updated when configuration parameters change here
    rospy.loginfo("""Config set to {position_x}, {position_y}, {position_z}""".format(**config))


def slamExampleNode():
    rospy.init_node('slam_example_node', anonymous=True)
    slamPublisher = SLAMPublisher()
    client = dynamic_reconfigure.client.Client("slam_server", timeout=30, config_callback=update)
    client.update_configuration({})

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        #sets up params which is a dictionary filled with values obtained from the dynamic reconfigure gui
        params = client.get_configuration(timeout=10)

        # Create Header message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'slam'

        # Create Odometry message
        #pos = Point(0.0, 1.0, 2.0)
        #below we set the parameters of Point using values from dynamic_reconfigure gui
        pos = Point(params["position_x"], params["position_y"], params["position_z"])
        print("position_x ", params["position_x"])
        print("position_y ", params["position_y"])
        print("position_z ", params["position_z"])

        num_particle = params["number_of_particles"]
        print(num_particle)

        rot = Quaternion(0.4, 0.2, 0.3, 0.5)
        pose = PoseWithCovariance(Pose(pos, rot), [0.8] * 36)
        lin = Vector3(4.0, 2.0, 5.0)
        ang = Vector3(0.4, 0.8, 0.2)
        twist = TwistWithCovariance(Twist(lin, ang), [0.2] * 36)
        state = Odometry(header, '', pose, twist)

        # Create Map message
        landmarkHeader = Header()
        landmarkHeader.stamp = rospy.Time.now()
        landmarkHeader.frame_id = 'gate'
        map = Map(header, [PoseWithCovarianceStamped(landmarkHeader, pose)])

        # Publish messages
        slamPublisher.publishState(state)
        slamPublisher.publishMap(map)
        rate.sleep()

if __name__ == '__main__':
    try:
        slamExampleNode()
    except rospy.ROSInterruptException:
        pass
