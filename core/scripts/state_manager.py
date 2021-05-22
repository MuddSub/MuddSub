#!/usr/bin/env python

from nav_msgs.msg import Odometry
import rospy
from controls.msg import State
from tf.transformations import euler_from_quaternion

rospy.init_node("state_manager")

stateVectorPub = rospy.Publisher("/slam/robot/state", State, queue_size=1)


def handleState(msg):
    p = msg.pose.pose.position
    x,y,z = p.x, p.y, p.z

    q = msg.pose.pose.orientation
    qList = [q.x, q.y, q.z, q.w]
    roll, pitch, yaw = euler_from_quaternion(qList)

    v = msg.twist.twist.linear
    a = msg.twist.twist.angular

    velocity = [v.x, v.y, v.z, a.x, a.y, a.z]

    result = [x,y,z,roll,pitch,yaw] + velocity

    state = State()
    state.state = result

    stateVectorPub.publish(state)

    outstring =   "State: \n"
    outstring +=  "   Position: [{:0.3f},{:0.3f},{:0.3f}] \n".format(x,y,z)
    outstring +=  "   Velocity: [{:0.3f},{:0.3f},{:0.3f}] \n".format(v.x, v.y, v.z)
    outstring +=  "   Orientation: [{:0.3f},{:0.3f},{:0.3f}] \n".format(roll, pitch, yaw)
    outstring +=  "   Ang Vel: [{:0.3f},{:0.3f},{:0.3f}] \n".format(a.x, a.y, a.z)

    rospy.loginfo(outstring)

stateSub = rospy.Subscriber("/slam/robot/pose", Odometry, handleState)

rospy.spin()
