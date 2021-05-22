#/usr/bin/env python

import rospy
from geometry_msgs.msg import Wrench, WrenchStamped

wrenchPub = rospy.Publisher("/alfie/thruster_manager/input", Wrench, queue_size=1)
wrenchSub = rospy.Subscriber("/controls/robot/wrench", WrenchStamped, lambda msg: wrenchPub.publish(msg.wrench))
