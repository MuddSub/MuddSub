#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, Float32, Bool
from StateMachine import State, Sequence, WaitForAll, WaitForAny, Repeat, Lambda, InitWrapper, Timer
from drivers.msg import Depth, EulerOrientation
import numpy as np

PWM_RANGE_V = 600
PWM_RANGE_H = 100
IDLE_PWM = 1500

def wrap_to_pi(theta):
    # return (theta % (2 * np.pi)) - np.pi
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
    MAX_PWM = IDLE_PWM + PWM_RANGE_V
    MIN_PWM = IDLE_PWM - PWM_RANGE_V
    # MAX_PWM = 2100
    # MIN_PWM = 900
    USE_ALL_PWMS = False

    depth = 0
    def update_depth(msg):
        Submerge.depth = msg.depth
    rospy.Subscriber('/drivers/depth_sensor/depth', Depth, update_depth, queue_size=1)

    vfl_pwm_publisher = rospy.Publisher('/robot/pwm/vfl', Int32, queue_size=1)
    vfr_pwm_publisher = rospy.Publisher('/robot/pwm/vfr', Int32, queue_size=1)
    vbl_pwm_publisher = rospy.Publisher('/robot/pwm/vbl', Int32, queue_size=1)
    vbr_pwm_publisher = rospy.Publisher('/robot/pwm/vbr', Int32, queue_size=1)

    def publish_vertical_pwms(pwms):
        Submerge.vfl_pwm_publisher.publish(Int32(int(pwms[0])))
        Submerge.vfr_pwm_publisher.publish(Int32(int(pwms[1])))
        Submerge.vbl_pwm_publisher.publish(Int32(int(pwms[2])))
        Submerge.vbr_pwm_publisher.publish(Int32(int(pwms[3])))

    def __init__(self, Kp, desired_depth):
        super().__init__()
        self.desired_depth = desired_depth
        self.Kp = Kp

    def run(self):
        delta = self.depth - desired_depth
        out_pwm = self.Kp * delta + IDLE_PWM
        out_pwm = min(Submerge.MAX_PWM, max(Submerge.MIN_PWM, out_pwm))
        if Submerge.USE_ALL_PWMS:
            Submerge.publish_vertical_pwms([out_pwm] * 4)
        else:
            Submerge.publish_vertical_pwms([out_pwm, IDLE_PWM, IDLE_PWM, out_pwm])


    def end(self):
        super().end()
        Submerge.publish_vertical_pwms([IDLE_PWM] * 4)

class WaitUntilSubmerged(State):
    currentDepth = 0
    def update_depth(msg):
        WaitUntilSubmerged.currentDepth = msg.depth
    rospy.Subscriber('/drivers/depth_sensor/depth', Depth, update_depth, queue_size=1)

    def __init__(self, depth_limit):
        super().__init__()
        self.depth_limit = depth_limit

    def run(self):
        if WaitUntilSubmerged.currentDepth > self.depth_limit:
            self.end()

class StraightForward(State):
    MAX_PWM = IDLE_PWM + PWM_RANGE_H
    MIN_PWM = IDLE_PWM - PWM_RANGE_H

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
        StraightForward.hfl_pwm_publisher.publish(Int32(int(pwms[0])))
        StraightForward.hfr_pwm_publisher.publish(Int32(int(pwms[1])))
        StraightForward.hbl_pwm_publisher.publish(Int32(int(pwms[2])))
        StraightForward.hbr_pwm_publisher.publish(Int32(int(pwms[3])))

    def __init__(self, Kp, desired_yaw, yaw_error_threshold=0.05):
        self.yaw_error_threshold = yaw_error_threshold
        self.desired_yaw = desired_yaw
        self.Kp = Kp

    def run(self):
        # yaw_error = wrap_to_pi(self.desired_yaw - StraightForward.yaw)
        yaw_error = 0
        if np.abs(yaw_error) < self.yaw_error_threshold:
            forward_effort = 200
        else:
            forward_effort = 0
        angular_effort = self.Kp * yaw_error
        rospy.loginfo(f"desired_yaw: {self.desired_yaw}, current_yaw: {StraightForward.yaw}, yaw_error: {yaw_error}")
        # rospy.loginfo(angular_effort)
        pwms = np.array([IDLE_PWM + forward_effort] * 4, dtype='float64') + np.array([1., -1., 1., -1.]) * angular_effort
        pwms = np.clip(pwms, StraightForward.MIN_PWM, StraightForward.MAX_PWM)
        StraightForward.publish_horizontal_pwms(pwms)

    def end(self):
        super().end()
        StraightForward.publish_horizontal_pwms([IDLE_PWM] * 4)

class RotateInPlace(State):
    MAX_PWM = IDLE_PWM + PWM_RANGE_H
    MIN_PWM = IDLE_PWM - PWM_RANGE_H
    # MAX_PWM = 2100
    # MIN_PWM = 900

    yaw = 0
    def update_yaw(msg):
        RotateInPlace.yaw = msg.yaw_z_radian
    rospy.Subscriber('/drivers/IMU/euler_orientation', EulerOrientation, update_yaw)

    hfl_pwm_publisher = rospy.Publisher('/robot/pwm/hfl', Int32, queue_size=1)
    hfr_pwm_publisher = rospy.Publisher('/robot/pwm/hfr', Int32, queue_size=1)
    hbl_pwm_publisher = rospy.Publisher('/robot/pwm/hbl', Int32, queue_size=1)
    hbr_pwm_publisher = rospy.Publisher('/robot/pwm/hbr', Int32, queue_size=1)

    def publish_horizontal_pwms(pwms):
        RotateInPlace.hfl_pwm_publisher.publish(Int32(int(pwms[0])))
        RotateInPlace.hfr_pwm_publisher.publish(Int32(int(pwms[1])))
        RotateInPlace.hbl_pwm_publisher.publish(Int32(int(pwms[2])))
        RotateInPlace.hbr_pwm_publisher.publish(Int32(int(pwms[3])))

    def __init__(self, target_delta_yaw, Kp):
        super().__init__()
        self.target_delta_yaw = target_delta_yaw
        self.start_yaw = 0
        self.previous_yaw = 0
        self.delta_yaw = 0
        self.Kp = Kp

    def start(self):
        super().start()
        start_yaw = RotateInPlace.yaw

    def run(self):
        current_yaw = RotateInPlace.yaw
        self.delta_yaw += wrap_to_pi(current_yaw - self.previous_yaw)
        error = self.target_delta_yaw - self.delta_yaw
        angular_effort = self.Kp * error
        # rospy.loginfo(angular_effort)
        pwms = np.array([IDLE_PWM ] * 4, dtype='float64') + np.array([1., -1., 1., -1.]) * angular_effort
        pwms = np.clip(pwms, RotateInPlace.MIN_PWM, RotateInPlace.MAX_PWM)
        RotateInPlace.publish_horizontal_pwms(pwms)
        self.previous_yaw = current_yaw

class WaitForReset(State):
    DEPTH_LIMIT = -0.04

    currentDepth = 0
    def update_depth(msg):
        WaitForReset.currentDepth = msg.depth
    rospy.Subscriber('/drivers/depth_sensor/depth', Depth, update_depth)

    def __init__(self):
        super().__init__()
        self.lastDepth = 100

    def run(self):
        self.lastDepth = WaitForReset.currentDepth
        if WaitForReset.currentDepth < WaitForReset.DEPTH_LIMIT and self.lastDepth < WaitForReset.DEPTH_LIMIT:
            self.end()

class DoABarrelRoll(State):
    PWM_RANGE = 200
    MAX_PWM = IDLE_PWM + PWM_RANGE
    MIN_PWM = IDLE_PWM - PWM_RANGE
    # MAX_PWM = 2100
    # MIN_PWM = 900
    USE_ALL_PWMS = False

    vfl_pwm_publisher = rospy.Publisher('/robot/pwm/vfl', Int32, queue_size=1)
    vfr_pwm_publisher = rospy.Publisher('/robot/pwm/vfr', Int32, queue_size=1)
    vbl_pwm_publisher = rospy.Publisher('/robot/pwm/vbl', Int32, queue_size=1)
    vbr_pwm_publisher = rospy.Publisher('/robot/pwm/vbr', Int32, queue_size=1)

    def publish_vertical_pwms(pwms):
        DoABarrelRoll.vfl_pwm_publisher.publish(Int32(int(pwms[0])))
        DoABarrelRoll.vfr_pwm_publisher.publish(Int32(int(pwms[1])))
        DoABarrelRoll.vbl_pwm_publisher.publish(Int32(int(pwms[2])))
        DoABarrelRoll.vbr_pwm_publisher.publish(Int32(int(pwms[3])))

    def __init__(self):
        super().__init__()

    def run(self):
        if DoABarrelRoll.USE_ALL_PWMS:
            DoABarrelRoll.publish_vertical_pwms([DoABarrelRoll.MAX_PWM, DoABarrelRoll.MIN_PWM, DoABarrelRoll.MAX_PWM, DoABarrelRoll.MIN_PWM])
        else:
            DoABarrelRoll.publish_vertical_pwms([0, DoABarrelRoll.MIN_PWM, 0, DoABarrelRoll.MIN_PWM])

    def end(self):
        super().end()
        Submerge.publish_vertical_pwms([IDLE_PWM] * 4)

class Log(State):
    def __init__(self, msg):
        super().__init__()
        self.msg = msg

    def run(self):
        rospy.loginfo(self.msg)
        self.end()

if __name__ == '__main__':
    rospy.init_node('mission', anonymous=False)
    rate = rospy.Rate(5)  # 50Hz
    startup_delay_secs = float(rospy.get_param("startup_delay_secs"))
    forward_time_secs = float(rospy.get_param("forward_time_secs"))
    drift_time_secs = float(rospy.get_param("drift_time_secs"))
    roll_time_secs = float(rospy.get_param("roll_time_secs"))
    depth_Kp = int(rospy.get_param("depth_Kp"))
    desired_depth = float(rospy.get_param("desired_depth"))
    yaw_Kp = int(rospy.get_param("yaw_Kp"))

    params = {'desired_yaw': 0}
    def record_yaw():
        params['desired_yaw'] = StraightForward.yaw
        rospy.loginfo(f'Setting Desired Yaw: {StraightForward.yaw}')

    state_machine = Sequence([
        Timer(startup_delay_secs),
        Lambda(record_yaw),
        Log("Submerging..."),
        WaitForAny([
            Submerge(depth_Kp, desired_depth),
            Sequence([
                WaitUntilSubmerged(desired_depth),
                Log("Sumberged! Driving forward..."),
                WaitForAny([
                    InitWrapper(StraightForward, [yaw_Kp], params),
                    Timer(forward_time_secs)
                ]),
                Log("Stopping..."),
                Timer(drift_time_secs),
                Log("Drift complete!")
            ])
        ]),
        Log("Rooooool <^>v<^>v"),
        WaitForAny([
            DoABarrelRoll(),
            Timer(roll_time_secs)
        ])
    ])

    # state_machine = Repeat(
    #     Sequence([
        
    #         # WaitForMissionStart(),
    #         Lambda(record_yaw),
    #         WaitForAny([
    #             # WaitForMissionEnd(),
    #             WaitForAny([
    #                 Submerge(depth_Kp, desired_depth),
    #                 Sequence([
    #                     Timer(10),  # How long we wait for the robot to submerge
    #                     WaitForAny([
    #                         Timer(7),  # How long we wait for the robot to get to the gate # TODO change to 25
    #                         InitWrapper(StraightForward, yaw_Kp, **params)
    #                     ]),
    #                     Timer(0)  # How long we wait before starting rotations
    #                     # Add 2 full rotations
    #                     # RotateInPlace(1, yaw_Kp)
    #                     # RotateInPlace(5, yaw_Kp)
    #                     # InitWrapper(StraightForward, yaw_Kp, **params)  # Move forward indefinitely
    #                 ])
    #             ])
    #         ]),
    #         Repeat(Timer(100))
    #     ])
    # )
    # state_machine.start()

    # state_machine = Repeat(
    #     Sequence([
    #         WaitForMissionStart(),
    #         WaitForAny([
    #             WaitForMissionEnd(),
    #             RotateInPlace(0, yaw_Kp)
    #         ])
    #     ])
    # )
    # state_machine.start()

    # state_machine = Repeat(
    #     Sequence([
    #         # Timer(200),
    #         WaitForAny([
    #             #Sequence([Lambda(lambda: rospy.loginfo("RESET!")), Timer(2), WaitForReset()]),
    #             state_machine
    #             # Sequence([
    #             #     Lambda(lambda: rospy.loginfo("MISSION START!!!!")),
    #             #     Repeat(Timer(100))
    #             # ])
    #         ])
    #     ])
    # )

    state_machine = Repeat(Sequence([
        Log("Waiting for mission start..."),
        WaitForMissionStart(),
        Log("Starting mission..."),
        WaitForAny([
            state_machine,
            WaitForMissionEnd()
        ]),
        Log("Mission Ended! Waiting for explicit restart..."),
        WaitForMissionEnd(),
        Log("Restarting...")
    ]))

    state_machine.start()
    # rospy.loginfo(state_machine._status)
    try:
        while not rospy.is_shutdown():
            state_machine.run()
            rate.sleep()
    finally:
        state_machine.end() # Should stop 
