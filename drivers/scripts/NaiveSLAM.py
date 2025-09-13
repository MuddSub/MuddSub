#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Header
from drivers.msg import Position
from drivers.msg import DVL

class Vector2:
    def __init__(self, x,y):
        self.x = x
        self.y = y

class TimestampVelocity:
    def __init__(self, x, y, time):
        self.x = x
        self.y = y
        self.time = time 

    def time_elapsed(self):
        """Return the time elapsed since other_time"""
        return rospy.get_time() - self.time

class OdometrySLAM:
    
    def __init__(self):
        self.mission_start_subscriber = rospy.Subscriber("/robot/mission_started", Bool, self.start_callback)
        self.dvl_subscriber = rospy.Subscriber('/drivers/DVL', DVL, self.update_velocity_callback)

        # Position publisher
        self.position_publisher = rospy.Publisher("/robot/position", Position, queue_size=1)

        self.started = False
        # The current robot position
        self.position = Vector2(0.0, 0.0)

        # Save the position at the last updated velocity
        self.last_position = Vector2(0.0, 0.0)
        self.last_velocity = TimestampVelocity(0.0, 0.0, 0.0)

    def start_callback(self, msg):
        self.started = msg.data

    def update_velocity_callback(self, msg):
        if not msg.data.valid:
            return
        if self.started:
            self.update_position()
            self.last_position.x = self.position.x
            self.last_position.y = self.position.y

        self.last_velocity.x = msg.data.velocity.x
        self.last_velocity.y = msg.data.velocity.y
        self.last_position.time = msg.data.header.stamp.to_sec()


    def update_position(self):
        delta_time = self.last_velocity.time_elapsed()
        change_x = delta_time * self.last_velocity.x
        change_y = delta_time * self.last_velocity.y
        self.position.x = self.last_position.x + change_x
        self.position.y = self.last_position.y + change_y

    def publish(self):
        self.position_publisher.publish(Position(Header(), self.position.x, self.position.y))
    
    def update_and_publish(self):
        self.update_position()
        self.publish()

    
    
if __name__ == '__main__':
    rospy.init_node('SLAM', anonymous = True)
    rate = rospy.Rate(50) # 50 Hz
    tracker = OdometrySLAM()

    try:
        while not rospy.is_shutdown():
            if not tracker.started:
                tracker.publish()
            else:
                tracker.update_and_publish()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass