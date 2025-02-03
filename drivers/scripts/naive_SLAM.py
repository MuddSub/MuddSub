#!/usr/bin/env python
import rospy
from std_msgs.msg import Header, Bool
from drivers.msg import DVL, Position


class NaiveSLAM:
    def __init__(self):
        self.dvl_subscriber = rospy.Subscriber('drivers/dvl', DVL, self.velocity_receiver)
        #self.dvl_subscriber = rospy.Subscriber('drivers/dvl_debug', DVL, self.velocity_receiver) # uncomment to test
        self.mission_start = rospy.Subscriber("/robot/mission_started", Bool, self.start_callback)
        self.position_publisher = rospy.Publisher('drivers/position', Position, queue_size=1)

        self.started = False
        #self.started = True #uncomment to test
        self.last_x_pos = 0
        self.last_y_pos = 0
        self.last_vel_read = 0
        self.last_vel_x = 0
        self.last_vel_y = 0
    
    def position(self):
        delta = rospy.get_time() - self.last_vel_read
        return (self.last_x_pos + delta * self.last_vel_x, self.last_y_pos + delta * self.last_vel_y)

    def publish(self, xpos, ypos):
        """Publishes the position"""
        self.position_publisher.publish(Position(Header(), xpos, ypos))

    def velocity_receiver(self, msg):
        """Update when it receives a new velocity"""
        if msg.valid:
            self.last_x_pos, self.last_y_pos = self.position()
            self.last_vel_y = msg.velocity.y
            self.last_vel_x = msg.velocity.x
            self.last_vel_read = msg.header.stamp.to_sec()

    def start_callback(self, msg):
        self.started = msg.data
        if self.started:
            self.last_vel_read = rospy.get_time()

    def update(self):
        """Publishes the current position"""
        rospy.loginfo(self.position())
        self.publish(*self.position())

if __name__ == "__main__":
    try:
        rospy.init_node("naive_SLAM", anonymous = True)
        rate = rospy.Rate(50) # Should loop 50 times per second
        position_reporter = NaiveSLAM()
        while not rospy.is_shutdown():
            if position_reporter.started:
                position_reporter.update()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass