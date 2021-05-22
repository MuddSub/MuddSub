#!/usr/bin/env python
import rospy
import tf2_geometry_msgs
import geometry_msgs.msg
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import ApplyBodyWrench
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
import std_srvs.srv as std_srv
from controls.msg import State
import sys
import tf2_ros
import copy

"""
@brief Provides an interface layer between Gazebo and the Robot's code.
    Ensures that messages that usually do something physical on the robot have the same functionality in sim.
    Responsible for the coordinate transormations between ENU (ROS/Gazebo) and NED (Our code).
"""
class GazeboInterface:
    def __init__(self):

        ## Subscribe to the wrench produced by Controls, bypassing thruster allocation
        self.wrenchSub = rospy.Subscriber("/controls/robot/wrench", geometry_msgs.msg.WrenchStamped, self.wrenchCB)

        ## Get state updates from Gazebo
        self.stateSub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.stateCB)

        ## Publish the sim's view of odometry
        self.odomPub = rospy.Publisher("/sim/ground_truth_pose", Odometry, queue_size=5)

        ## Tell gazebo where to move the robot.
        self.statePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=5)

        self.resetSub = rospy.Subscriber("/reset_simulation", Empty, self.resetSimulation)

        ## If the parameter /slam_use_ground_truth == 1, then also publish the ground truth
        ##  pose as the output of localization.
        # TODO: Move to slam/
        self.slamPub = rospy.Publisher("/slam/robot/pose", Odometry, queue_size=1)

        self.resetPub = rospy.Publisher("/reset_controller", Empty, queue_size=1)

        self.resetClient = rospy.ServiceProxy("gazebo/reset_world", std_srv.Empty)
        self.pauseClient = rospy.ServiceProxy("gazebo/pause_physics", std_srv.Empty)
        self.unpauseClient = rospy.ServiceProxy("gazebo/unpause_physics", std_srv.Empty)

        self.wrenchPub = rospy.Publisher("/alfie/thruster_manager/input", geometry_msgs.msg.Wrench, queue_size=1)

        # Connect to the gazebo body wrench service server
        try:
            rospy.wait_for_service('/gazebo/apply_body_wrench', timeout=10)
        except rospy.ROSException:
            rospy.logerr("Apply body wrench service not available. Closing node.")
            sys.exit(-1)
        self.wrench = geometry_msgs.msg.Wrench()

        try:
            self.applyWrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed, error=', e)
            sys.exit(-1)

        ## TF2 transform broadcaster to update model state in TF
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        # Configure TF2 listener to get transforms from Gazebo
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.tfPrevTime = rospy.get_time()

    ## @brief Set the position of the robot in Gazebo, provided in NED
    ## @param pose: The pose as geometry_msgs Pose or PoseStamped
    def setPoseNED(self, pose):
        state = ModelState()

        # Get the transfomation from World (ENU) to world_ned from TF2
        try:
            trans = self.tfBuffer.lookup_transform("world_ned", "world", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to get transform from World to world_ned")
            return 0
        # Accomodate both Pose and PoseStamped types
        if type(pose) is geometry_msgs.msg.Pose:
            poseStamped = geometry_msgs.msg.PoseStamped()
            poseStamped.pose = pose
            pose = poseStamped

        # Transform the pose from ENU to NED
        state.pose = tf2_geometry_msgs.do_transform_pose(poseStamped, trans).pose

        # pack it up and send it off.
        # Note that many Gazebo topics/services don't obsever reference_frame properly,
        #  hence the manual conversion.
        state.model_name = "alfie"
        state.reference_frame = "world"
        self.statePub.publish(state)

        return 1

    ## @brief Set the state (pose + twist) of the robot in Gazebo, provided in NED
    ## @param odometry: a nav_msgs Odometry message in NED
    def setStateNED(self, odometry):
        state = ModelState()

        # Get the transfomation from World (ENU)  to world_ned from TF2
        try:
            trans = self.tfBuffer.lookup_transform("world_ned", "world", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to get transform from World to world_ned")
            return 0

        # beep boop transform pose
        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.pose = pose

        state.pose = tf2_geometry_msgs.do_transform_pose(poseStamped, trans).pose

        # Just do twist manually since it's easy and there's no do_transform_twist (sad)
        state.twist.linear.x = twist.linear.y
        state.twist.linear.y = twist.linear.x
        state.twist.linear.z = -twist.linear.z

        state.twist.angular.x = twist.angular.y
        state.twist.angular.y = twist.angular.x
        state.twist.angular.z = -twist.angular.z

        state.model_name = "alfie"
        state.reference_frame = "world"
        self.statePub.publish(state)

        return 1

    ## @brief Apply a wrench to the body, expressed in it's local NED frame
    ## @param wrench: A geometry_msgs Wrench() or WrenchStamped() message
    def applyWrenchNED(self, wrench):

        try:
            trans = self.tfBuffer.lookup_transform("alfie/base_link", "world", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to get transform from World to base link")
            return 0


        if type(wrench) is geometry_msgs.msg.Wrench:
            stampedWrench = geometry_msgs.msg.WrenchStamped()
            stampedWrench.wrench = wrench
            wrench = stampedWrench

        wrenchOut = tf2_geometry_msgs.do_transform_wrench(wrench, trans)
        success = self.applyWrench(
                    "alfie/base_link",
                    "world",
                    geometry_msgs.msg.Point(0,0,0),
                    wrenchOut.wrench,
                    rospy.Time.now(),
                    rospy.Duration(1))
        return 1

    ## @brief Handle the incoming
    def wrenchCB(self, wrench):
        self.wrenchPub.publish(wrench.wrench)

    # Get ENU frame from gazebo and re-publish as NED frame for MuddSub
    # Also, publish the TF2 transform in NED
    def stateCB(self, states):
        name = rospy.get_param('/robot_name')
        index = states.name.index(name)

        pose = states.pose[index]
        twist = states.twist[index]
        msg = Odometry()

        try:
            trans = self.tfBuffer.lookup_transform("world_ned", "world", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to get transform from World to world_ned")
            return 0

        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.pose = pose

        msg.pose.pose = tf2_geometry_msgs.do_transform_pose(poseStamped, trans).pose

        linearStamped = geometry_msgs.msg.PointStamped()
        angluarStamped = geometry_msgs.msg.PointStamped()

        # Just do twist manually since it's easy and there's no do_transform_twist (sad)
        msg.twist.twist.linear.x = twist.linear.y
        msg.twist.twist.linear.y = twist.linear.x
        msg.twist.twist.linear.z = -twist.linear.z

        msg.twist.twist.angular.x = twist.angular.y
        msg.twist.twist.angular.y = twist.angular.x
        msg.twist.twist.angular.z = -twist.angular.z

        self.odomPub.publish(msg)

        slamParam = "/slam_use_ground_truth"

        if rospy.has_param(slamParam) and rospy.get_param(slamParam):
            self.slamPub.publish(msg)

        t = geometry_msgs.msg.TransformStamped()

        # Also update the transform
        time = rospy.get_time()
        if time - self.tfPrevTime > .02:
            t.header.stamp = rospy.Time.now()
            self.tfPrevTime = time

            t.header.frame_id = "world"
            t.child_frame_id = "alfie/base_link"
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z

            t.transform.rotation.x = pose.orientation.x
            t.transform.rotation.y = pose.orientation.y
            t.transform.rotation.z = pose.orientation.z
            t.transform.rotation.w = pose.orientation.w

            self.tfBroadcaster.sendTransform(t)

        return 1


    def resetSimulation(self, msg):
        self.pauseClient()

        resetMsg = Empty()
        self.resetPub.publish(resetMsg)

        self.resetClient()

        pose = geometry_msgs.msg.Pose()
        pose.position.z = 0
        pose.orientation.w = 1

        while not interface.setPoseNED(pose):
            rate.sleep()


        self.unpauseClient()

if __name__ == '__main__':
    rospy.init_node('gazebo_interface', anonymous=True)
    interface = GazeboInterface()

    rate = rospy.Rate(10)

    # pose = geometry_msgs.msg.Pose()
    # pose.position.z = 0
    # pose.orientation.w = 1
    #
    # # Initialize the pose of the robot to NED 0,0,0,0,0,0
    # while not interface.setPoseNED(pose):
    #     rate.sleep()

    # Iterate asynchronously (responding to messages only)
    rospy.spin()
