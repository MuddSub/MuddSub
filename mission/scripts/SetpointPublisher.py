#!/usr/bin/env python

import rospy
from controls.msg import State
from std_msgs.msg import Header
import dynamic_reconfigure.client

seq = 0

def getCallback(pub):
    global seq
    def callback(config):
        global seq
        #Add whatever needs to be updated when configuration parameters change here
        # Retrieve parameters from the parameter server
        setpoint = rospy.get_param('mission_server/spoof/setpoint', 
            {
                'pose': {
                    'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
                },
                'twist': {
                    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'angular': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
                }
            }
        )

        # Create the message object
        state = State()

        # Populate its header
        state.header = Header(seq, rospy.Time.now(), 'BASE')
        seq += 1

        # Populate its state
        state.state = [
            setpoint['pose']['position']['x'],
            setpoint['pose']['position']['y'],
            setpoint['pose']['position']['z'],
            setpoint['pose']['orientation']['roll'],
            setpoint['pose']['orientation']['pitch'],
            setpoint['pose']['orientation']['yaw'],
            setpoint['twist']['linear']['x'],
            setpoint['twist']['linear']['y'],
            setpoint['twist']['linear']['z'],
            setpoint['twist']['angular']['roll'],
            setpoint['twist']['angular']['pitch'],
            setpoint['twist']['angular']['yaw']
        ]
        print('New setpoint is {}'.format(state.state))

        # Publish the new setpoint
        pub.publish(state)
    return callback

def main():
    pub = rospy.Publisher('/robot_setpoint', State, queue_size=10)
    rospy.init_node('setpoint_publisher', anonymous=True)
    callback = getCallback(pub)
    print("Starting clients...")
    client = dynamic_reconfigure.client.Client("mission_server/spoof/setpoint/pose/position", timeout=30, config_callback=callback)
    client = dynamic_reconfigure.client.Client("mission_server/spoof/setpoint/pose/orientation", timeout=30, config_callback=callback)
    client = dynamic_reconfigure.client.Client("mission_server/spoof/setpoint/twist/linear", timeout=30, config_callback=callback)
    client = dynamic_reconfigure.client.Client("mission_server/spoof/setpoint/twist/angular", timeout=30, config_callback=callback)
    print("Spinning...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass