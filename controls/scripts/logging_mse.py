import rospy
import numpy as np
import pandas
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


def getData():
    """"""
    def get_z(msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        roll = msg.pose.pose.orientation.x
        pitch = msg.pose.pose.orientation.y
        yaw = msg.pose.pose.orientation.z
    rospy.init_node('position_listener', anonymous=True)
    position = rospy.Subscriber('/robot_state', Odometry, get_z)
    loopRate = rospy.Rate(1) #Hz
    while not rospy.is_shutdown():
        loopRate.sleep()

if __name__ == '__main__':
    getData()
