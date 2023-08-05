#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, Float32, Bool
from vision.msg import BoundingBoxArray
from drivers.msg import Depth
from collections import deque
import numpy as np

DEFAULT_PWM = 1500
MAX_PWM = 2000
MIN_PWM = 1000
TIMER_LIMIT = 1 #1 sec
DEFAULT_FORWARD_PWM = 1600
class Forward:
    def __init__(self, Kp, use_vision):
        self.gate_center_subscriber = rospy.Subscriber("vision/left_camera/gate_center", Float32, self.record_center)
        self.mission_start = rospy.Subscriber("/robot/mission_started", Bool, self.start_callback)  # Check whether the mission is going
        self.hfl_pwm_publisher = rospy.Publisher('/robot/pwm/hfl', Int32, queue_size=1)
        self.hfr_pwm_publisher = rospy.Publisher('/robot/pwm/hfr', Int32, queue_size=1)
        self.hbl_pwm_publisher = rospy.Publisher('/robot/pwm/hbl', Int32, queue_size=1)
        self.hbr_pwm_publisher = rospy.Publisher('/robot/pwm/hbr', Int32, queue_size=1)
        self.use_vision_publisher = rospy.Publisher('/robot/use_vision', Bool, queue_size=1)
        
        self.Kp = Kp
        self.forward_effort = 100
        self.gate_center = 0.5  # the current center of the gate could be 0 to 1
        self.center_ff = 0.1  # coefficient for an iir filter on the gate center
        self.start = False
        self.use_vision = use_vision
        print(f"Using Vision: {self.use_vision}")
        self.use_vision_publisher.publish(Bool(self.use_vision))
        
    def start_callback(self, msg):
        self.start = msg.data
    
    def record_center(self, msg):
        # implements an iir filter on the gate center
        self.gate_center = (1 - self.center_ff) * self.gate_center + self.center_ff * msg.data
    
    def move_forward_rotate(self):
        if self.use_vision:
            delta = self.gate_center - 0.5  # negative

            # Above 1500 is forward
            # hfl, hfr, hbl, hbr
            horizontal_pwms = np.array([DEFAULT_PWM + self.forward_effort,
                                        DEFAULT_PWM + self.forward_effort,
                                        DEFAULT_PWM + self.forward_effort,
                                        DEFAULT_PWM + self.forward_effort], dtype='float64')
            angular_effort = self.Kp * delta
            horizontal_pwms += np.array([1., -1., 1., -1.]) * angular_effort
            horizontal_pwms = np.clip(horizontal_pwms, MIN_PWM, MAX_PWM)
            self.publish_horizontal_pwms(horizontal_pwms.astype("int32"))
        else:
            horizontal_pwms = np.array([DEFAULT_FORWARD_PWM]*4)
            self.publish_horizontal_pwms(horizontal_pwms)

    def publish_horizontal_pwms(self, horizontal_pwms):
        if self.mission_start:
            self.hfl_pwm_publisher.publish(Int32(horizontal_pwms[0]))
            self.hfr_pwm_publisher.publish(Int32(horizontal_pwms[1]))
            self.hbl_pwm_publisher.publish(Int32(horizontal_pwms[2]))
            self.hbr_pwm_publisher.publish(Int32(horizontal_pwms[3]))
        else:
            print("mission not started")

if __name__ == '__main__':
    rospy.init_node('straight_forward', anonymous=True)
    rate = rospy.Rate(50)  # 50Hz
    Kp = int(rospy.get_param("straight_forward_Kp"))
    use_vision = rospy.get_param("use_vision")
    forward_mover = Forward(Kp,use_vision)
    acceptable_error = 0.1
    while not rospy.is_shutdown():
        if not forward_mover.start:
            forward_mover.publish_horizontal_pwms([1500,1500,1500,1500])
            # print("Mission not started")
            continue
        else:
            if not forward_mover.gate_center:
                print("no gate_center")
            
            # if we don't recieve a gate_center, will move forward based on the latest recieved gate_center
            # defaults to gate_center = 0.5 at startup
            forward_mover.move_forward_rotate()
        rate.sleep()
# #!/usr/bin/env python3
# import rospy
# from std_msgs.msg import Int32, Float32, Bool
# from vision.msg import BoundingBoxArray
# from drivers.msg import Depth
# from collections import deque
# import numpy as np

# DEFAULT_PWM = 1500
# MAX_PWM = 2000
# MIN_PWM = 1000
# TIMER_LIMIT = 1 #1 sec

# class Forward:
#     def __init__(self, Kp):
#         self.gate_center_subscriber = rospy.Subscriber("vision/left_camera/gate_center", Float32, self.record_center)
#         self.mission_start = rospy.Subscriber("/robot/mission_started", Bool, self.start_callback)  # Check whether the mission is going
#         self.hfl_pwm_publisher = rospy.Publisher('/robot/pwm/hfl', Int32, queue_size=1)
#         self.hfr_pwm_publisher = rospy.Publisher('/robot/pwm/hfr', Int32, queue_size=1)
#         self.hbl_pwm_publisher = rospy.Publisher('/robot/pwm/hbl', Int32, queue_size=1)
#         self.hbr_pwm_publisher = rospy.Publisher('/robot/pwm/hbr', Int32, queue_size=1)
        
#         self.Kp = Kp
#         self.forward_effort = 200
#         self.gate_center = 0.5  # the current center of the gate could be 0 to 1
#         self.center_ff = 0.1  # coefficient for an iir filter on the gate center
#         self.start = False
        
#     def start_callback(self, msg):
#         self.start = msg.data
    
#     def record_center(self, msg):
#         # implements an iir filter on the gate center
#         self.gate_center = (1 - self.center_ff) * self.gate_center + self.center_ff * msg.data
    
#     def move_forward_rotate(self):
#         delta = self.gate_center - 0.5  # negative

#         # Above 1500 is forward
#         # hfl, hfr, hbl, hbr
#         horizontal_pwns = np.array([1500 + self.forward_effort] * 4)
#         angular_effort = self.Kp * delta
#         horizontal_pwms += np.array([1, -1, 1, -1]) * angular_effort
#         horizontal_pwms = np.clip(horizontal_pwms, MIN_PWM, MAX_PWM)
#         self.publish_horizontal_pwms(horizontal_pwms)

#     def publish_horizontal_pwms(self, horizontal_pwms):
#         self.hfl_pwm_publisher.publish(Int32(horizontal_pwms[0]))
#         self.hfr_pwm_publisher.publish(Int32(horizontal_pwms[1]))
#         self.hbl_pwm_publisher.publish(Int32(horizontal_pwms[2]))
#         self.hbr_pwm_publisher.publish(Int32(horizontal_pwms[3]))

# if __name__ == '__main__':
#     rospy.init_node('straight_forward', anonymous=True)
#     rate = rospy.Rate(50)  # 50Hz
#     Kp = 100
#     forward_mover = Forward(Kp)
#     acceptable_error = 0.1
#     while not rospy.is_shutdown():
#         if not forward_mover.start:
#             continue
#             print("Mission not started")
#         else:
#             if not forward_mover.gate_center:
#                 print("no gate_center")
            
#             # if we don't recieve a gate_center, will move forward based on the latest recieved gate_center
#             # defaults to gate_center = 0.5 at startup
#             forward_mover.move_forward_rotate()
#         rate.sleep()
