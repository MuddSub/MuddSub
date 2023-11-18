#! /usr/bin/python3
# This module contains the ROS node for state estimation
from typing import Tuple
import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovariance, Point, Pose, Quaternion
from state_estimation.scripts.kalman import KinematicKalmanFilter
from sensor_msgs.msg import Imu
from drivers.msg import DVL


DRIFT = 0  # m^2/s^3
DVL_PREC = 0  # s^2/m^2
RATE = 50  # Hz


def q_mult(a: float, x: np.ndarray, b: float, y: np.ndarray) -> Tuple[float, np.ndarray]:
    """Multiplies two quaternions

    Args:
        a: The scalar part of the first (left) quaternion. (Required)
        x: The vector part of the first (left) quaternion. (Required)
        b: The scalar part of the second (right) quaternion. (Required)
        y: The vector part of the second (right) quaternion. (Required)
    
    Returns:
        A tuple `(c, z)` where `c` is the scalar part of the product and
        `z` is the vector part of the product quaternion.
    """
    return a * b - x @ y, np.cross(x, y) + a * y + b * x


def rotate(vector: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    """Rotates a 3D vector by a rotation given by a quaternion.
    
    Args:
        vector: A 3D vector to be rotated. (Required)
        orientation: A 4D vector representation of a quaternion encoding
            the rotation to apply to the vector. (Required)
    
    Returns:
        The 3D vector rotated by the given rotation.
    """
    return q_mult(rotation[0], rotation[1:],
                  *q_mult(0, vector, rotation[0], rotation[1:]))[1]


def parse_dvl(dvl: DVL, orientation: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    return (
        rotate(np.array([dvl.velocity.x, dvl.velocity.y, dvl.velocity.z]), orientation),
        np.eye(3) * DVL_PREC
    )


def exportPoseCov(pos: np.ndarray, cov: np.ndarray, orientation: np.ndarray) -> PoseWithCovariance:
    pose_cov = np.zeros((6, 6))
    pose_cov[:3,:3] = cov
    point = Point(pos[0], pos[1], pos[2])
    quaternion = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
    pose = Pose(point, orientation)
    return PoseWithCovariance(pose, *pose_cov.tolist())


def run_state_estimation():
    orientation = np.array([1, 0, 0, 0])
    def set_orientation(new_orientation: np.ndarray):
        nonlocal orientation
        orientation = new_orientation
    filter = KinematicKalmanFilter(np.eye(3) * DRIFT, 3)
    dvl_sub = rospy.Subscriber("drivers/dvl", DVL, lambda m: filter.add_velocity(*parse_dvl(m)))
    imu_sub = rospy.Subscriber("/vectornav/IMU", Imu,
        lambda m: set_orientation(np.array([m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z])))
    pose_pub = rospy.Publisher("state/pose", PoseWithCovariance, queue_size=1)
    vel_pub = rospy.Publisher("state/velocity", PoseWithCovariance, queue_size=1)
    rospy.init_node("state_estimation", anonymous=False)
    rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        pose_pub.publish(exportPoseCov(filter.get_position(), filter.get_position_cov()))
        vel_pub.publish(exportPoseCov(filter.get_velocity(), filter.get_velocity_cov()))
        filter.step(rate.sleep())


if __name__ == "__main__":
    run_state_estimation()
