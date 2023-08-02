#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, Float32
from vision.msg import BoundingBoxArray
from drivers.msg import Depth
from collections import deque

DEFAULT_PWM = 1500
MAX_PWM = 3000
MIN_PWM = 0

class DepthController:
    def __init__(self, Kp):
        self.depth_sensor = rospy.Subscriber('/drivers/depth_sensor/depth', Depth, self.depth_sensor_callback)
        # self.camera_detections = rospy.Subscriber(bounding_box_array_topic, BoundingBoxArray)
        # self.hfl_pwm_publisher = rospy.Publisher('/robot/pwm/hfl', Int32, queue_size=1)
        # self.hfr_pwm_publisher = rospy.Publisher('/robot/pwm/hfr', Int32, queue_size=1)
        # self.hbl_pwm_publisher = rospy.Publisher('/robot/pwm/hbl', Int32, queue_size=1)
        # self.hbr_pwm_publisher = rospy.Publisher('/robot/pwm/hbr', Int32, queue_size=1)
        self.vfl_pwm_publisher = rospy.Publisher('/robot/pwm/vfl', Int32, queue_size=1)
        self.vfr_pwm_publisher = rospy.Publisher('/robot/pwm/vfr', Int32, queue_size=1)
        self.vbl_pwm_publisher = rospy.Publisher('/robot/pwm/vbl', Int32, queue_size=1)
        self.vbr_pwm_publisher = rospy.Publisher('/robot/pwm/vbr', Int32, queue_size=1)
        self.desired_vertical_pwm = {}
        self.default_pwm = {'vfl':DEFAULT_PWM,
                            'vfr':DEFAULT_PWM,
                            'vbl':DEFAULT_PWM,
                            'vbr':DEFAULT_PWM,
                            'hfl':DEFAULT_PWM,
                            'hfr':DEFAULT_PWM,
                            'hbl':DEFAULT_PWM,
                            'hbr':DEFAULT_PWM}
        self.desired_horizontal_pwm = {}
        self.latest_ten_depth = deque()
        self.Kp = Kp
        
 

    def depth_sensor_callback(self, msg):
        self.depth = msg.depth
        if len(self.latest_ten_depth)>=10:
            self.latest_ten_depth.popleft()
        self.latest_ten_depth.append(self.depth)
        self.avg_depth = sum(self.latest_ten_depth)/len(self.latest_ten_depth)
    
    def depth_controller(self, desired_depth):
        #base: 1500, upward: 1600, downward:1400, max: 3000, min: 0
        delta = self.depth-desired_depth
        out_pwm = self.Kp * delta + DEFAULT_PWM
        # bound inputs to 
        if out_pwm > MAX_PWM:
            out_pwm = MAX_PWM
        if out_pwm < MIN_PWM:
            out_pwm = MIN_PWM
        self.desired_vertical_pwm ={'vfl':out_pwm,
                                    'vfr':out_pwm,
                                    'vbl':out_pwm,
                                    'vbr':out_pwm}
        self.submerge()

    def submerge(self):
        def publish_pwm(pwm_dict):
            # self.hfl_pwm_publisher.publish(Int32(pwm_dict['hfl']))
            # self.hfr_pwm_publisher.publish(Int32(pwm_dict['hfr']))
            # self.hbl_pwm_publisher.publish(Int32(pwm_dict['hbl']))
            # self.hbr_pwm_publisher.publish(Int32(pwm_dict['hbr']))

            self.vfl_pwm_publisher.publish(Int32(pwm_dict['vfl']))
            self.vfr_pwm_publisher.publish(Int32(pwm_dict['vfr']))
            self.vbl_pwm_publisher.publish(Int32(pwm_dict['vbl']))
            self.vbr_pwm_publisher.publish(Int32(pwm_dict['vbr']))
        if self.depth:
            print("up-down-movement")
            publish_pwm(self.desired_vertical_pwm)
            # print(f"sending pwm data {pwm_array} to motors")
            print("currently depth is: "+ self.depth)
        else:
            print("no depth data")
            publish_pwm(self.default_pwm)
            # print(f"sending pwm data {default_pwm_array} to motors")

            

if __name__ == '__main__':
    rospy.init_node('depth_controller', anonymous=True)
    rate = rospy.Rate(50)  # 50Hz
    Kp = 0.5
    depth_controller = DepthController(Kp)
    desired_depth = 2
    acceptable_error = 0.1
    while not rospy.is_shutdown():
        try:
            if abs(depth_controller.avg_depth - desired_depth) < acceptable_error:
                print(f"reached desired depth of {depth_controller.avg_depth}")
            depth_controller.depth_controller()
        except Exception:
            print("Error in depth controller")
        rate.sleep()
