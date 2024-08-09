#!/usr/bin/env python3

import rospy
import numpy as np
import select
import os

IDLE_PWM = 1500
PWM_RANGE = 600

def convert(n):
    if n <= 127:
        return n / 127
    else:
        return (n - 256) / 128

def threshold(n):
    if n < 0.1 and n > -0.1:
        return 0.0
    else:
        return n

if __name__ == "__main__":
    # Open the js0 device as if it were a file in read mode.
    pipe_path = '/home/muddsub/joystick_pipe'
    if os.path.isfile(pipe_path):
        print("pipe path already exists")
        raise ValueError(f'Please remove file {pipe_path}')
    if not os.path.exists(pipe_path):
        os.mkfifo(pipe_path)
        print("Making pipe path")
    pipe = open(pipe_path, 'r')

    # Create an empty list to store read characters.
    msg = []
    print("hello")
    rospy.init_node('joystick_teleop')

    def set_speeds(speeds):
        rospy.set_param('drivers_server/pwm/thrusters/horizontal', {
            'bl': int(speeds[0, 0]),
            'br': int(speeds[0, 1]),
            'fl': int(speeds[0, 2]),
            'fr': int(speeds[0, 3])
        })
        rospy.set_param('drivers_server/pwm/thrusters/vertical', {
            'bl': int(speeds[1, 0]),
            'br': int(speeds[1, 1]),
            'fl': int(speeds[1, 2]),
            'fr': int(speeds[1, 3])
        })

    max_speed = 1.0  # Percentage of full speed
    tank_right = np.array([
        [0, 1, 0, 1],
        [0, 0, 0, 0]
    ])

    tank_left = np.array([
        [1, 0, 1, 0],
        [0, 0, 0, 0]
    ])

    back_left = np.array([
        [1, 0, 0, 0],
        [0, 0, 0, 0]
    ])

    back_right = np.array([
        [0, 1, 0, 0],
        [0, 0, 0, 0]
    ])

    front_right = np.array([
        [0, 0, 0, 1],
        [0, 0, 0, 0]
    ])

    front_left = np.array([
        [0, 0, 1, 0],
        [0, 0, 0, 0]
    ])


    forward = np.array([
        [1, 1, 1, 1],
        [0, 0, 0, 0]
    ])
    clockwise = np.array([
        [1, -1, 1, -1],
        [0, 0, 0, 0]
    ])
    up = np.array([
        [0, 0, 0, 0],
        [0, 1, 1, 0]
    ])
    right = np.array([
        [-1, 1, 1,-1],
        [0, 0, 0, 0]
    ])
    speeds = np.zeros([2, 4])
    joy_axes = [0, 0, 0, 0, 0, 0] #[LJoyHor, LJoyVert, RJoyHor, RJoyVert, DPadHor, DPadVert]
    joy_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] #[1, 2, 3, 4, LB, RB, LT, RT, 9, 10]

    updown_axes = [1, 3]
    leftright_axes = [0, 2]

    while not rospy.is_shutdown():
        while True:
            rlist, _, _ = select.select([pipe], [], [], 0)
            if len(rlist) == 0:
                break

            for char in pipe.buffer.read(8):
                # print(type(char))

                # append the integer representation of the unicode character read to the msg list.
                msg += [char]

                # If the length of the msg list is 8...
                if len(msg) == 8:

                # Button event if 6th byte is 1
                    if msg[6] == 1:
                        if msg[4]  == 1:
                            print('button', msg[7], 'down')
                            joy_buttons[msg[7]] = 1
                        else:
                            print('button', msg[7], 'up')
                            joy_buttons[msg[7]] = 0
                            

                    # joy_axes event if 6th byte is 2
                    elif msg[6] == 2:
                        old_val = msg[5]
                        if msg[7] in updown_axes:
                            msg[5] = threshold(-convert(msg[5]))
                        else:
                            msg[5] = threshold(convert(msg[5]))
                        print('joy_axes', msg[7], msg[5], 'old value:', old_val)
                        joy_axes[msg[7]] = msg[5]

                    # Reset msg as an empty list.
                    msg = []
        
        # speeds = np.zeros([2, 4])
        # speeds += up * joy_buttons[5]
        # speeds += up * -1 * joy_buttons[4]
        # speeds += clockwise * joy_axes[2]
        # speeds += forward * joy_axes[1]
        # speeds += right * joy_axes[0]
        # speeds = np.clip(speeds, -max_speed, max_speed) * PWM_RANGE + IDLE_PWM


        #[LJoyHor, LJoyVert, RJoyHor, RJoyVert, DPadHor, DPadVert]
         #[1, 2, 3, 4, LB, RB, LT, RT, 9, 10]
         # rospy.loginfo(f"Tank_left: {joy_axes[1]}, Tank_right: {joy_axes[3]}")
        speeds = np.zeros([2, 4])
        speeds += back_left * joy_buttons[4]
        speeds += back_right * joy_buttons[5]
        speeds += front_left * joy_buttons[6]
        speeds += front_right * joy_buttons[7]
        speeds += up * joy_axes[5]
        speeds += tank_left * joy_axes[1]
        speeds += tank_right * joy_axes[3]

        speeds = speeds * max_speed * PWM_RANGE + IDLE_PWM
        set_speeds(speeds)


    set_speeds(np.zeros([2, 4]))
