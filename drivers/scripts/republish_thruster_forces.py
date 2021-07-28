#!/usr/bin/env python

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from std_msgs.msg import Float64
import rospy

def publish(msg, args):
    publisher, name = args
    newMsg = Float64()
    newMsg.data = msg.data
    rospy.loginfo("Name: {}, Value: {}".format(name, newMsg.data))
    publisher.publish(newMsg)

if __name__ == "__main__":
    rospy.init_node("republish_thruster_forces") 
    subscribers = []
    publishers = []

    thrusterNames = ["hfl", "hfr", "hbl", "hbr", "vfl", "vfr", "vbl", "vbr"]

    for i, name in enumerate(thrusterNames):
        publishers.append(rospy.Publisher("/controls/thruster_forces/{}".format(name), Float64, queue_size=1))
        subscribers.append(rospy.Subscriber("/alfie/thrusters/{}/input".format(i), FloatStamped, publish, (publishers[i], name), queue_size=1))

    rospy.spin()
