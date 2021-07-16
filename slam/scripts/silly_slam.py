#!/usr/bin/env python
import rospy
from drivers.msg import Depth, DVL
from sensor_msgs.msg import Imu
from SLAMPublisher import SLAMPublisher
from nav_msgs.msg import Odometry
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import numpy as np

class SillySlam:

		def __init__(self):
			self.depthSub = rospy.Subscriber("/drivers/depth_sensor/depth", Depth, self.depthCB, queue_size=1)
			self.imuSub = rospy.Subscriber("/imu/imu", Imu, self.imuCB, queue_size=1)
			self.dvlSub = rospy.Subscriber("/drivers/dvl", DVL, self.dvlCB, queue_size=1)
			self.prevTime = None
			self.x_pos = 0
			self.y_pos = 0
			self.z_pos = 0
			self.depth = 0
			self.slamPub = SLAMPublisher()
			self.orientation = None
			self.time = None
			self.x_velocity = 0
			self.y_velocity = 0
			self.z_velocity = 0
			self.dt = 0

		def depthCB(self, msg):
			self.depth = msg.depth

		def imuCB(self, msg):
			self.orientation = msg.orientation

		def dvlCB(self, msg):
			self.prevTime = self.time
			self.time = msg.header.stamp.to_sec()
			# left side dvl flips
			# self.x_velocity = -msg.velocity.y # this is on purpose it needs to be flipped
			# self.y_velocity = msg.velocity.x
			# right side dvl flips
			self.x_velocity = msg.velocity.y
			self.y_velocity = -msg.velocity.x
			self.z_velocity = msg.velocity.z
			self.fom = msg.fom
			self.altitude = msg.altitude
			self.valid = msg.valid
			self.status = msg.status

		def iterate(self):
			msg = Odometry()

			if self.orientation is None:
				msg.pose.pose.orientation.w = 1
			else:
				msg.pose.pose.orientation = self.orientation

			msg.pose.pose.position.x = self.x_pos
			msg.pose.pose.position.y = self.y_pos

			msg.pose.pose.position.z = self.depth
			# msg.pose.pose.position.z = self.z_pos

			msg.twist.twist.linear.x = self.x_velocity
			msg.twist.twist.linear.y = self.y_velocity
			msg.twist.twist.linear.z = self.z_velocity

			self.integrateToPos()

			self.slamPub.publishState(msg)

		def integrateToPos(self):
			if self.time is not None and self.prevTime is not None:
				self.dt = self.time - self.prevTime

				self.x_pos += self.x_velocity * self.dt
				self.y_pos += self.y_velocity * self.dt
				self.z_pos += self.z_velocity * self.dt

if __name__ == '__main__':
	rospy.init_node("silly_slam")
	ss = SillySlam()

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		ss.iterate()
		rate.sleep()
