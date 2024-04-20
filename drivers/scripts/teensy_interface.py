#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Int32, Float64
from drivers.utils.DepthSensorPublisher import DepthSensorPublisher
from drivers.msg import DVL
import numpy as np
import time
from std_msgs.msg import Bool

global ser
line_buffer = b''
thrusters = [1500]*8

def readline() -> str:
    '''
    Wrapper over serial.Serial.readline with some extra functionality to
    handle being called repeatedly in a while loop.
    '''
    global line_buffer

    # If no data is waiting to be read, return an empty string.
    if ser.in_waiting:
        # Read a line from the serial port. Since this is being called in
        # a loop repeatedly, the result of readline may not be an entire
        # line. Only the first part of a line might be available. The
        # result is appended to the device's read buffer for use when an
        # entire line is ready.
        partial_line = ser.readline()
        line_buffer += partial_line

    # Check if there is an entire line in the read buffer and if so,
    # remove it from the buffer and return it.
    if b'\n' in line_buffer:
        line, rest = line_buffer.split(b'\n', maxsplit=1)
        line_buffer = rest

        # Sometimes the device sends trash data that can't be converted to
        # a string?? If so, ignore it.
        try:
            return line.decode('utf-8')
        except UnicodeDecodeError as err:
            self._print('Discarding line: ', err, v=1)
            return ''
    return ''

def readlines() -> [str]:
    '''
    Read all available lines and process them
    '''
    lines = []
    line = readline()
    while len(line) != 0:
        lines.append(line)

        # Try to read another line
        line = readline()
    return lines

def parse_line(line):
    '''
    Read a line from serial and split it into the type of command and arguments
    '''
    items = line.split(',')
    if items[0] == 'depth' and len(items) == 2:
        return 'depth', float(items[1])
    elif items[0] == 'switch' and len(items) == 2:
        return 'switch', items[1] == 'on'
    elif len(items) == 8:
        return 'dvl', items
    else:
        print('Recieved:', line)
        return None, None

def pulseToSerial(msg, i: int) -> None:
    '''
    Reads the data into the global thrusters array
    '''
    pulse = msg.data
    thrusters[i] = pulse

if __name__ == '__main__':
        rospy.init_node('teensy_interface', anonymous=True)
        
        try:
            ser = serial.Serial('/dev/ttyACM0', timeout=10.0) # open serial port
        except:
            print(f"not /dev/ttyACM0")
            rospy.logerr("Couldn't open serial")
            exit()

        if not ser.is_open:
            print(f"can't open ser")
            rospy.logerr("Couldn't open serial")
            exit()
        # What code publishes to robot/pwm
        hfl_subscriber = rospy.Subscriber('/robot/pwm/hfl', Int32, pulseToSerial, (6), queue_size=1)
        hfr_subscriber = rospy.Subscriber('/robot/pwm/hfr', Int32, pulseToSerial, (2), queue_size=1)
        hbl_subscriber = rospy.Subscriber('/robot/pwm/hbl', Int32, pulseToSerial, (5), queue_size=1)
        hbr_subscriber = rospy.Subscriber('/robot/pwm/hbr', Int32, pulseToSerial, (3), queue_size=1)

        vfl_subscriber = rospy.Subscriber('/robot/pwm/vfl', Int32, pulseToSerial, (4), queue_size=1)
        vfr_subscriber = rospy.Subscriber('/robot/pwm/vfr', Int32, pulseToSerial, (0), queue_size=1)
        vbl_subscriber = rospy.Subscriber('/robot/pwm/vbl', Int32, pulseToSerial, (7), queue_size=1)
        vbr_subscriber = rospy.Subscriber('/robot/pwm/vbr', Int32, pulseToSerial, (1), queue_size=1)

        mission_started_publisher = rospy.Publisher('/robot/mission_started', Bool, queue_size=1)
        mission_started = False

        dsp = DepthSensorPublisher()
        dvlp = rospy.Publisher('/drivers/dvl', DVL, queue_size=1)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            lines = readlines()
            for line in lines:
                ty, data = parse_line(line)
                if ty == 'depth':
                    print('depth:', data)
                    dsp.publishDepth(data)
                elif ty == 'switch':
                    # print('switch', data)
                    mission_started = data
                elif ty == 'dvl':
                    dvl_msg = DVL()
                    dvl_msg.header.stamp = rospy.Time.now()
                    dvl_msg.velocity.x = float(data[1])
                    dvl_msg.velocity.y = float(data[2])
                    dvl_msg.velocity.z = float(data[3])
                    dvl_msg.fom = float(data[4])
                    dvl_msg.altitude = float(data[5])
                    dvl_msg.valid = data[6] == 'y'
                    dvl_msg.status = bool(float(data[7]))
                    dvlp.publish(dvl_msg)

            # # Code to start mission
            # submerged = np.min(depth_queue) > 0.05
            # if submerged:
            #     if not prev_submerged:
            #         print('Robot entered water')
            #         start_time = time.time()
                
            #     # Wait a minute after submerging to start
            #     mission_started = time.time() - start_time > 60
            # else:
            #     mission_started = False
            # prev_submerged = submerged
            mission_started_publisher.publish(mission_started)

            ser.write('thrust,0{},1{},2{},3{},4{},5{},6{},7{}\n'.format(*thrusters).encode('utf_8'))
            rate.sleep()