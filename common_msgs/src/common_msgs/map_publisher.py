import rospy
from nav_msgs.msg import Odometry
from slam.msg import Map
import scipy.spatial.transform.Rotation.from_euler
import numpy as np
from util import convert_covs

class SLAMPublisher:
    """
    Publishes SLAM messages describing the states of both the robot and the environment.
    The state of the robot, composed of its coordinates, velocity, and uncertainties are represented by nav_msgs/Odometry messages published to the slam/robot/state topic.
    The map of the environment, composed of landmark coordinates and uncertainties are represented by slam/Map messages published to the slam/map topic.
    Attributes:
        _statePub: Publisher to slam/robot/state. Publishes nav_msgs/Odometry messages.
        _mapPub: Publisher to slam/map. Publishes slam/map messages.
    """

    def __init__(self):
        """Initialize SLAMPublisher instance."""
        self._statePub = rospy.Publisher('slam/robot/state', Odometry, queue_size=10)
        self._mapPub = rospy.Publisher('slam/map', Map, queue_size=10)

    def publishState(self, pos, euler_rot, pos_cov, rot_cov, velocity, angular_velocity, 
                      velocity_cov, angular_velocity_cov):
        """
        Publish a nav_msgs/Odometry message.
        Uses the _statePub Publisher to publish to slam/robot/state.
        Args:
            state: The nav_msgs/Odometry message.
        """
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'slam'

        # Create Odometry message
        pos = Point(*pos)
        quaternion = from_euler(euler_rot)
        rot = Quaternion(*quaternion)
        cov = convert_covs(pos_cov, rot_cov)
        pose = PoseWithCovariance(Pose(pos, rot),cov)
        
        velocity = Vector3(*velocity)
        angular_velocity = Vector3(*angular_velocity)
        twist_cov = convert_covs(velocity_cov, angular_velocity_cov)
        twist = TwistWithCovariance(Twist(velocity, angular_velocity), twist_cov)
        state = Odometry(header, '', pose, twist)
        self._statePub.publish(state)

    def publishMapFromLists(self, obstacle_names, obstacle_pos, obstacle_pos_covs,):
        """
        Publish a slam/Map message.
        Uses the _mapPub Publisher to publish to slam/map.
        Args:
            map: The slam/Map message.
        """
        header = None
        obstacles = []

        for name, pos, pos_cov in zip(obstacle_names, obstacle_pos, obstacle_pos_covs):
          pos = Point(*pos)
          pos_cov = convert_covs(pos_cov)
          obstacles.append(Obstacle(header, name, pos, pos_cov))

        self._mapPub.publish(Map(header, obstacles))