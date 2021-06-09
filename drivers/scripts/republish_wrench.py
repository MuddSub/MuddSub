#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Wrench, WrenchStamped

rospy.init_node("republish_wrench")

publisher = rospy.Publisher("/alfie/thruster_manager/input", Wrench, queue_size=1)

def pubWrench(msg):
    publisher.publish(msg.wrench)

wrenchSub = rospy.Subscriber("/controls/robot/wrench", WrenchStamped, pubWrench, queue_size=1)

rospy.spin()
