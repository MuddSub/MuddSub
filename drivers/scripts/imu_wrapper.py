#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from drivers.msg import EulerOrientation, ImuAcceleration

import math
class Imu_Wrapper():
    def __init__(self):
        self.raw_imu = rospy.Subscriber("/vectornav/IMU", Imu, self.convert_to_euler)
        self.euler_publisher = rospy.Publisher("/drivers/IMU/euler_orientation", EulerOrientation, queue_size=1)

        self.raw_imu_acc = rospy.Subscriber("/vectornav/IMU", Imu, self.get_imu_acceleration)
        self.acceleration_publisher = rospy.Publisher("/drivers/IMU/imu_acceleration", ImuAcceleration, queue_size=1) # Is queue of 1 necessary?

    def get_imu_acceleration(self, msg):
        x_a = msg.linear_acceleration.x
        y_a = msg.linear_acceleration.y
        z_a = msg.linear_acceleration.z
        
        acceleration_msg = ImuAcceleration(Header(), x_a, y_a, z_a)
        self.acceleration_publisher.publish(acceleration_msg)
        

    
    # for the vectornav imu
    # the range is from -pi/2 to pi/2
    def convert_to_euler(self, msg):
        quaternion = msg.orientation
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(x,y,z,w)
        euler_msg = EulerOrientation(Header(), roll_x, pitch_y, yaw_z)
        self.euler_publisher.publish(euler_msg)

    def euler_from_quaternion(self, x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z # in radians


if __name__ == "__main__":
    rospy.init_node('imu_wrapper', anonymous=True)
    imu_wrapper = Imu_Wrapper()
    rospy.spin()
