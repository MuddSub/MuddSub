#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32

class PwmRepublisher:
    def __init__(self):
        rospy.init_node('pwm_listener', anonymous=False)

        # Publisher
        self.pub_combined = rospy.Publisher("/robot/pwm/combined", String, queue_size=10)

        # Storage for latest values
        self.pwm_values = {
            'hfl': None, 'hfr': None, 'hbl': None, 'hbr': None,
            'vfl': None, 'vfr': None, 'vbl': None, 'vbr': None
        }

        # Subscribers for each thruster
        rospy.Subscriber("/robot/pwm/hfl", Int32, self.callback, "hfl")
        rospy.Subscriber("/robot/pwm/hfr", Int32, self.callback, "hfr")
        rospy.Subscriber("/robot/pwm/hbl", Int32, self.callback, "hbl")
        rospy.Subscriber("/robot/pwm/hbr", Int32, self.callback, "hbr")
        rospy.Subscriber("/robot/pwm/vfl", Int32, self.callback, "vfl")
        rospy.Subscriber("/robot/pwm/vfr", Int32, self.callback, "vfr")
        rospy.Subscriber("/robot/pwm/vbl", Int32, self.callback, "vbl")
        rospy.Subscriber("/robot/pwm/vbr", Int32, self.callback, "vbr")

    def callback(self, data, thruster):
        """Stores the latest PWM value for a given thruster and republishes formatted data."""
        self.pwm_values[thruster] = data.data
        self.republish()

    def republish(self):
        """Republishes all thruster PWM values as a formatted string."""
        if all(value is not None for value in self.pwm_values.values()):
            formatted_string = (
                f"HFL: {self.pwm_values['hfl']} | HFR: {self.pwm_values['hfr']} | "
                f"HBL: {self.pwm_values['hbl']} | HBR: {self.pwm_values['hbr']} || "
                f"VFL: {self.pwm_values['vfl']} | VFR: {self.pwm_values['vfr']} | "
                f"VBL: {self.pwm_values['vbl']} | VBR: {self.pwm_values['vbr']}"
            )
            self.pub_combined.publish(formatted_string)

if __name__ == '__main__':
    pwm_republisher = PwmRepublisher()
    rospy.spin()  # Keep the node running
