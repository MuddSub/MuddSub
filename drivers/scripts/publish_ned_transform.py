#!/usr/bin/env python
import rospy

import tf_conversions
import tf2_ros
import geometry_msgs.msg



if __name__ == "__main__":
	rospy.init_node("publish_ned_transform")
	br = tf2_ros.TransformBroadcaster()
	t = geometry_msgs.msg.TransformStamped()

	t.header.frame_id="alfie/base_link"
	t.child_frame_id="alfie/base_link_ned"

	t.transform.rotation.w = 1

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		sendT = t
		sendT.header.stamp = rospy.Time.now()
		br.sendTransform(t)
		rate.sleep()
