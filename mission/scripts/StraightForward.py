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

# /robot/mission_started




class Forward:
    def __init__(self, Kp):
        self.gate_center_subscriber = rospy.Subscriber("vision/left_camera/gate_center", Float32, self.record_center)
        # self.depth_sensor = rospy.Subscriber('/drivers/depth_sensor/depth', Depth, self.depth_sensor_callback)
        # self.desired_depth = rospy.Subscriber('/mission/desired_depth', Float32, self.update_desired_depth_callback,)
        # self.camera_detections = rospy.Subscriber(bounding_box_array_topic, BoundingBoxArray)
        self.mission_start = rospy.Subscriber("/robot/mission_started", Bool, self.start_callback)
        self.hfl_pwm_publisher = rospy.Publisher('/robot/pwm/hfl', Int32, queue_size=1)
        self.hfr_pwm_publisher = rospy.Publisher('/robot/pwm/hfr', Int32, queue_size=1)
        self.hbl_pwm_publisher = rospy.Publisher('/robot/pwm/hbl', Int32, queue_size=1)
        self.hbr_pwm_publisher = rospy.Publisher('/robot/pwm/hbr', Int32, queue_size=1)
        
        self.default_pwm = {'vfl':DEFAULT_PWM,
                            'vfr':DEFAULT_PWM,
                            'vbl':DEFAULT_PWM,
                            'vbr':DEFAULT_PWM,
                            'hfl':DEFAULT_PWM,
                            'hfr':DEFAULT_PWM,
                            'hbl':DEFAULT_PWM,
                            'hbr':DEFAULT_PWM}
        self.desired_horizontal_pwm = {}
        self.Kp = Kp
        self.gate_center = 0.5
        self.start = False
        
    def start_callback(self, msg):
        self.start = msg.data
    
    def record_center(self, msg):
        self.gate_center = msg.data

    # def depth_sensor_callback(self, msg):
    #     print(msg.depth)
    #     self.depth = msg.depth
    #     if len(self.latest_ten_depth)>=10:
    #         self.latest_ten_depth.popleft()
    #     self.latest_ten_depth.append(self.depth)
    #     # self.avg_depth = sum(self.latest_ten_depth)/len(self.latest_ten_depth)
    #     self.avg_depth = 10

    
    def move_forward_rotate(self):
        delta = self.gate_center - 0.5 # negative 

        # 1600 is forward

        rotate_pwm_dict = {"hfl": 1500+self.Kp*delta,
                            "hfr": 1500-self.Kp*delta,
                            "hbl": 1500+self.Kp*delta, 
                            "hbr": 1500-self.Kp*delta}
        print(f"rotating like {str(rotate_pwm_dict)}")
        self.publish_horizontal_pwm(rotate_pwm_dict)

        forward_pwm_dict = {"hfl": 1700,
                            "hfr": 1700,
                            "hbl": 1700, 
                            "hbr": 1700}
        
        self.publish_horizontal_pwm(forward_pwm_dict)
        print(f"moving forward like 1700")


    def move_forward(self):
        pwm_dict = {"hfl": 1600,
                    "hfr": 1600,
                    "hbl": 1600, 
                    "hbr":1600}
        self.publish_horizontal_pwm(pwm_dict)
    
    def rotate(self, direction):
        """positive direction is counterclockwise, turn left"""
        if direction >0:
            print("we rotate towards right")
        elif direction < 0:
            print("we rotate left")
        else:
            print("no rotation. zero")
        

    def publish_horizontal_pwm(self, pwm_dict):
        self.hfl_pwm_publisher.publish(Int32(pwm_dict['hfl']))
        self.hfr_pwm_publisher.publish(Int32(pwm_dict['hfr']))
        self.hbl_pwm_publisher.publish(Int32(pwm_dict['hbl']))
        self.hbr_pwm_publisher.publish(Int32(pwm_dict['hbr']))
        print(pwm_dict)



if __name__ == '__main__':
    rospy.init_node('forward_mover', anonymous=True)
    rate = rospy.Rate(50)  # 50Hz
    Kp = 100
    forward_mover = Forward(Kp)
    acceptable_error = 0.1
    while not rospy.is_shutdown():
        if forward_mover.start == False:
            continue
        else:

            if not forward_mover.gate_center:
                print("no gate_center")
            # elif abs(depth_controller.avg_depth - depth_controller.desired_depth) < acceptable_error:
            #     print(f"reached desired depth of {depth_controller.avg_depth}")
            # print(f"desired depth: {depth_controller.desired_depth}")
            # except Exception:
            #     print("Error in depth controller")
            forward_mover.move_forward_rotate()
        rate.sleep()
