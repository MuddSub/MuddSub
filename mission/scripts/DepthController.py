#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, Float32, Bool
from vision.msg import BoundingBoxArray
from drivers.msg import Depth
from collections import deque
import threading

DEFAULT_PWM = 1500
MAX_PWM = 3000
MIN_PWM = 0
TIMER_LIMIT = 1 #1 sec


class DepthController:
    def __init__(self, Kp, desired_depth):
        self.depth_sensor = rospy.Subscriber('/drivers/depth_sensor/depth', Depth, self.depth_sensor_callback)
        self.desired_depth = rospy.Subscriber('/mission/desired_depth', Float32, self.update_desired_depth_callback)
        self.mission_start = rospy.Subscriber("/robot/mission_started", Bool, self.start_callback)

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
        self.avg_depth = 0
        self.depth = 0
        self.desired_depth = desired_depth
        self.default_desired_depth = 0
        self.timer = threading.Timer(TIMER_LIMIT, self.reset_desired_depth)
        self.start = False
    
    def start_callback(self, msg):
        self.start = msg.data

    def reset_desired_depth(self):
        # self.desired_depth = self.default_desired_depth
        # print(f"Just reset desired_depth to {self.desired_depth}")
        pass

    def depth_sensor_callback(self, msg):
        print(msg.depth)
        self.depth = msg.depth
        if len(self.latest_ten_depth)>=10:
            self.latest_ten_depth.popleft()
        self.latest_ten_depth.append(self.depth)
        # self.avg_depth = sum(self.latest_ten_depth)/len(self.latest_ten_depth)
        self.avg_depth = 10

    
    def update_desired_depth_callback(self, msg):
        self.timer.cancel()
        self.desired_depth = msg.data
        self.timer = threading.Timer(TIMER_LIMIT, self.reset_desired_depth)
        self.timer.start()
    
    def depth_controller(self, desired_depth):
        #base: 1500, upward: 1600, downward:1400, max: 3000, min: 0
        delta = self.depth-desired_depth
        out_pwm = self.Kp * delta + DEFAULT_PWM
        # bound inputs to 
        if out_pwm > MAX_PWM:
            out_pwm = MAX_PWM
        if out_pwm < MIN_PWM:
            out_pwm = MIN_PWM
        self.desired_vertical_pwm ={'vfl':int(out_pwm),
                                    'vfr':int(1500),
                                    'vbl':int(1500),
                                    'vbr':int(out_pwm)}
        self.submerge()

    def submerge(self):
        def publish_pwm(pwm_dict):
            self.vfl_pwm_publisher.publish(Int32(pwm_dict['vfl']))
            self.vfr_pwm_publisher.publish(Int32(pwm_dict['vfr']))
            self.vbl_pwm_publisher.publish(Int32(pwm_dict['vbl']))
            self.vbr_pwm_publisher.publish(Int32(pwm_dict['vbr']))
            # print(pwm_dict)
        if self.depth:
            # print("up-down-movement")
            publish_pwm(self.desired_vertical_pwm)
            # print(f"sending pwm data {pwm_array} to motors")
            print(f"currently depth is: {self.depth}")
        else:
            print("no depth data")
            publish_pwm(self.default_pwm)
            # print(f"sending pwm data {default_pwm_array} to motors")

if __name__ == '__main__':
    rospy.init_node('depth_controller', anonymous=False)
    rate = rospy.Rate(50)  # 50Hz
    Kp = int(rospy.get_param("depth_controller_Kp"))
    desired_depth = float(rospy.get_param("desired_depth"))
    depth_controller = DepthController(Kp, desired_depth)
    depth_controller.timer.start()
    acceptable_error = 0.1
    while not rospy.is_shutdown():
        if depth_controller.start == False:
            continue
        else:
            if not depth_controller.avg_depth:
                print("no depth_controller")
            elif abs(depth_controller.avg_depth - depth_controller.desired_depth) < acceptable_error:
                print(f"reached desired depth of {depth_controller.avg_depth}")
            print(f"desired depth: {depth_controller.desired_depth}")
            depth_controller.depth_controller(depth_controller.desired_depth)
            # except Exception:
            #     print("Error in depth controller")
        rate.sleep()
