#!/usr/bin/env python

import rospy
from hydrophones.msg import PingerData

class HydrophonesPublisher:
    """
    Publishes hydrophone messages describing information gathered from the robot's hydrophone.

    The range, bearing, confidence of range, and confidence of bearing of a detected acoustic signal are described by hydrophones/PingerData messages published to the hydrophones/data topic.

    Attributes:
        hydrophones_data_pub: Publisher to hydrophones/data topic. Publishes hydrophones/PingerData messages.
    """

    def __init__(self):
        """Initialize Hydrophones instance."""
        self.hydrophones_data_pub = rospy.Publisher('hydrophones/data', PingerData, queue_size=10)

    def publishData(self, data):
        """
        Publish a hydrophones/PingerData message.

        Uses the hydrophones_data_pub Publisher to publish to hydrophones/data topic.

        Args:
            data: The hydrophones/PingerData message.
        """
        self.hydrophones_data_pub.publish(data)
