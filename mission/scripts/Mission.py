#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, Float32, Bool
from StateMachine import State, Sequence, WaitForAll, WaitForAny, Repeat, Lambda, InitWrapper, Timer
from drivers.msg import Depth, EulerOrientation
import numpy as np

def wrap_to_pi(theta):
    return ((theta - np.pi) % (2 * np.pi)) - np.pi

class _MissionSwitchMonitor(State):
    _mission_started = False
    def update_mission_status(msg):
        _MissionSwitchMonitor._mission_started = msg.data
    rospy.Subscriber("/robot/mission_started", Bool, update_mission_status)

    def __init__(self, end_on_mission_start=False):
        super().__init__()
        self._end_on_mission_start = end_on_mission_start

    def run(self):
        if _MissionSwitchMonitor._mission_started:
            if self._end_on_mission_start:
                self.end()
        else:
            if not self._end_on_mission_start:
                self.end()

class WaitForMissionStart(_MissionSwitchMonitor):
    def __init__(self):
        super().__init__(end_on_mission_start=True)

class WaitForMissionEnd(_MissionSwitchMonitor):
    def __init__(self):
        super().__init__(end_on_mission_start=False)

class Submerge(State):
    IDLE_PWM = 1500
    MAX_PWM = 2100
    MIN_PWM = 900

    depth = 0
    def update_depth(msg):
        Submerge.depth = msg.depth
    rospy.Subscriber('/drivers/depth_sensor/depth', Depth, update_depth)

    vfl_pwm_publisher = rospy.Publisher('/robot/pwm/vfl', Int32, queue_size=1)
    vfr_pwm_publisher = rospy.Publisher('/robot/pwm/vfr', Int32, queue_size=1)
    vbl_pwm_publisher = rospy.Publisher('/robot/pwm/vbl', Int32, queue_size=1)
    vbr_pwm_publisher = rospy.Publisher('/robot/pwm/vbr', Int32, queue_size=1)

    def publish_vertical_pwms(pwms):
        Submerge.vfl_pwm_publisher.publish(Int32(pwms[0]))
        Submerge.vfr_pwm_publisher.publish(Int32(pwms[1]))
        Submerge.vbl_pwm_publisher.publish(Int32(pwms[2]))
        Submerge.vbr_pwm_publisher.publish(Int32(pwms[3]))

    def __init__(self, Kp, desired_depth):
        super().__init__()
        self.desired_depth = desired_depth
        self.Kp = Kp
    
    def run(self):
        delta = self.depth - desired_depth
        out_pwm = self.Kp * delta + Submerge.IDLE_PWM
        out_pwm = min(Submerge.MAX_PWM, max(Submerge.MIN_PWM), out_pwm)
        self.publish_vertical_pwms([out_pwm, 0, 0, out_pwm])

    def end(self):
        super().end()
        Submerge.publish_vertical_pwms([Submerge.IDLE_PWM] * 4)

class StraightForward(State):
    IDLE_PWM = 1500
    MAX_PWM = 2100
    MIN_PWM = 900

    yaw = 0
    def update_yaw(msg):
        StraightForward.yaw = msg.yaw_z_radian
    rospy.Subscriber('/drivers/IMU/euler_orientation', EulerOrientation, update_yaw)
    # rospy.Subscriber('/vectornav/IMU')

    hfl_pwm_publisher = rospy.Publisher('/robot/pwm/hfl', Int32, queue_size=1)
    hfr_pwm_publisher = rospy.Publisher('/robot/pwm/hfr', Int32, queue_size=1)
    hbl_pwm_publisher = rospy.Publisher('/robot/pwm/hbl', Int32, queue_size=1)
    hbr_pwm_publisher = rospy.Publisher('/robot/pwm/hbr', Int32, queue_size=1)

    def publish_horizontal_pwms(pwms):
        StraightForward.hfl_pwm_publisher.publish(Int32(pwms[0]))
        StraightForward.hfr_pwm_publisher.publish(Int32(pwms[1]))
        StraightForward.hbl_pwm_publisher.publish(Int32(pwms[2]))
        StraightForward.hbr_pwm_publisher.publish(Int32(pwms[3]))

    def __init__(self, Kp, desired_yaw, yaw_error_threshold=0.05):
        self.yaw_error_threshold = yaw_error_threshold
        self.desired_yaw = desired_yaw
        self.Kp = Kp
    
    def run(self):
        yaw_error = wrap_to_pi(self.desired_yaw - StraightForward.yaw)
        if np.abs(yaw_error) < self.yaw_error_threshold:
            forward_effort = 200
        else:
            forward_effort = 0
        angular_effort = self.Kp * yaw_error
        pwms = np.array([StraightForward.DEFAULT_PWM + forward_effort] * 4, dtype='float64') + np.array([1., -1., 1., -1.]) * angular_effort
        StraightForward.publish_horizontal_pwms(pwms)
    
    def end(self):
        super().end()
        Submerge.publish_vertical_pwms([StraightForward.IDLE_PWM] * 4)

class RotateInPlace(State):
    IDLE_PWM = 1500
    MAX_PWM = 2100
    MIN_PWM = 900

    yaw = 0
    def update_yaw(msg):
        RotateInPlace.yaw = msg.yaw_z_radian
    rospy.Subscriber('/drivers/IMU/euler_orientation', EulerOrientation, update_yaw)

    hfl_pwm_publisher = rospy.Publisher('/robot/pwm/hfl', Int32, queue_size=1)
    hfr_pwm_publisher = rospy.Publisher('/robot/pwm/hfr', Int32, queue_size=1)
    hbl_pwm_publisher = rospy.Publisher('/robot/pwm/hbl', Int32, queue_size=1)
    hbr_pwm_publisher = rospy.Publisher('/robot/pwm/hbr', Int32, queue_size=1)

    def publish_horizontal_pwms(pwms):
        RotateInPlace.hfl_pwm_publisher.publish(Int32(pwms[0]))
        RotateInPlace.hfr_pwm_publisher.publish(Int32(pwms[1]))
        RotateInPlace.hbl_pwm_publisher.publish(Int32(pwms[2]))
        RotateInPlace.hbr_pwm_publisher.publish(Int32(pwms[3]))
    
    def __init__(self, target_delta_yaw, Kp):
        super().__init__()
        self.target_delta_yaw = target_delta_yaw
        self.start_yaw = 0
        self.previous_yaw = 0
        self.delta_yaw = 0
        self.Kp = Kp
    
    def start(self):
        super().start
        start_yaw = RotateInPlace.yaw
    
    def run(self):
        current_yaw = RotateInPlace.yaw
        self.delta_yaw += wrap_to_pi(current_yaw - self.previous_yaw)
        error = self.target_delta_yaw - self.delta_yaw
        angular_effort = self.Kp * error
        pwms = np.array([RotateInPlace.DEFAULT_PWM ] * 4, dtype='float64') + np.array([1., -1., 1., -1.]) * angular_effort
        RotateInPlace.publish_horizontal_pwms(pwms)
        self.previous_yaw = current_yaw

if __name__ == '__main__':
    rospy.init_node('mission', anonymous=False)
    rate = rospy.Rate(50)  # 50Hz
    depth_Kp = int(rospy.get_param("depth_Kp"))
    desired_depth = float(rospy.get_param("desired_depth"))
    yaw_Kp = int(rospy.get_param("yaw_Kp"))
    
    params = {'desired_yaw': 0}
    def record_yaw():
        params['desired_yaw'] = StraightForward.yaw

    # state_machine = Repeat(
    #     Sequence([
    #         WaitForMissionStart(),
    #         Lambda(record_yaw),
    #         WaitForAny([
    #             WaitForMissionEnd(),
    #             WaitForAny([
    #                 Submerge(depth_Kp, desired_depth),
    #                 Sequence([
    #                     Timer(5),  # How long we wait for the robot to submerge
    #                     WaitForAny([
    #                         Timer(25),  # How long we wait for the robot to get to the gate
    #                         InitWrapper(StraightForward, [yaw_Kp], params)
    #                     ]),
    #                     Timer(5),  # How long we wait before starting rotations
    #                     # Add 2 full rotations
    #                     InitWrapper(StraightForward, [yaw_Kp], params)  # Move forward indefinitely
    #                 ])
    #             ])
    #         ])
    #     ])
    # )
    # state_machine.start()

    state_machine = Repeat(
        Sequence([
            WaitForMissionStart(),
            WaitForAny([
                WaitForMissionEnd(),
                RotateInPlace(0, yaw_Kp)
            ])
        ])
    )
    state_machine.start()

    while not rospy.is_shutdown():
        state_machine.run()
        rate.sleep()
