import rospy
from nav_msgs.msg import Odometry
from geometry_msgs import Pose, PoseWithCovariance, TwitstWithCovariance, Twist, Point, Quaternion
import numpy as np
from tf.transformations import quaternion_from_euler

def publishOdometry():
    pub = rospy.Publisher('/controls/robot/error',Odometry,self.error_callback,queue_size=10)
    rospy.init_node('SampleOdometry',anonymous=True)
    rate = rospy.Rate(10)

    orientation = [0,0,np.pi/2]
    orientation = quaternion_from_euler(orientation)

    pose = Pose()
    pose.position = Point(0,0,0)
    pose.orientation = Quaternion(orientation[1],orientation[2],orientation[3],orientation[0])

    odometry = Odometry()
    odometry.pose = PoseWithCovariance(pose,[0.0]*36)



