#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped
from controls.msg import ThrusterForceArray
from controls.msg import ThrusterPWMArray

class Singleton(object):
    def __new__(cls, *args, **kw):
        if not hasattr(cls, '_instance'):
            orig = super(Singleton, cls)
            cls._instance = orig.__new__(cls, *args, **kw)
        return cls._instance

class ControlsPublisher(Singleton):
    was_initialized = False

    def __init__(self):
        if (not self.was_initialized):
            rospy.init_node('controlsPublisher', anonymous=True)
            self.wrench_pub = rospy.Publisher('controls/robot/wrench', WrenchStamped, queue_size=10)
            self.forces_pub = rospy.Publisher('controls/thruster/forces', ThrusterForceArray, queue_size=10)
            self.pwms_pub = rospy.Publisher('controls/thruster/pwms', ThrusterPWMArray, queue_size=10)
            self.was_initialized = True

    def publishWrench(self, wrenchStamped):
        self.wrench_pub.publish(wrenchStamped)

    def publishForces(self, forces):
        self.forces_pub.publish(forces)

    def publishPWMs(self, pwms):
        self.pwms_pub.publish(pwms)

# if __name__ == '__main__':
#     try:
#         obj = ControlsPublisher()
#         obj2 = ControlsPublisher()
#         assert (obj == obj2)
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
