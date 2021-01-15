#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Wrench, Point, Vector3, Pose, Twist
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import ApplyBodyWrench
from nav_msgs.msg import Odometry
import sys

"""
- Subscribe to wrench messages and call Gazebo service
- Subscribe to gazebo pose and re-publish as /slam/robot/pose
"""
class GazeboInterface:
    def __init__(self):

        self.wrenchSub = rospy.Subscriber("/controls/robot/wrench", Wrench, self.wrenchCB)
        self.stateSub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.stateCB)
        self.odomPub = rospy.Publisher("/slam/robot/pose", Odometry, queue_size=5)

        try:
            rospy.wait_for_service('/gazebo/apply_body_wrench', timeout=10)
        except rospy.ROSException:
            rospy.logerr("Apply body wrench service not available. Closing node.")
            sys.exit(-1)
        self.wrench = None

        try:
            self.applyWrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed, error=', e)
            sys.exit(-1)

    def wrenchCB(self, wrench):
        self.wrench = wrench

    def stateCB(self, states):
        name = rospy.get_param('/robot_name')
        index = states.name.index(name)

        print("HERE")
        pose = states.pose[index]
        twist = states.twist[index]
        print("HERE")
        msg = Odometry()
        msg.pose.pose = pose
        msg.twist.twist = twist

        self.odomPub.publish(msg)



    def iterate(self, duration):
        bodyName = '%s/base_link' % rospy.get_param('/robot_name')

        success = self.applyWrench(
                    bodyName,
                    'world',
                    Point(0,0,0),
                    self.wrench,
                    rospy.Time.now(),
                    rospy.Duration(duration))

        if not success:
            rospy.logerr("Body wrench apply service failed")

        else:
            rospy.loginfo("Success!")


if __name__ == '__main__':
    rospy.init_node('gazebo_interface', anonymous=True)
    interface = GazeboInterface()

    updateRate = 20

    rate = rospy.Rate(updateRate)

    while not rospy.is_shutdown():
        interface.iterate(1./float(updateRate))
        rate.sleep()
