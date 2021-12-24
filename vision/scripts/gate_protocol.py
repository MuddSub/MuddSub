#!/usr/bin/env python
import numpy as np
import rospy
from vision.msg import DetectionArray
class Gate_Protocol():
    CAMERA_FOV = 2
    CAMERA_XRES = 416
    NUM_SET_POINTS = 72
    FORWARD_STEP_SIZE = 1

    def __init__(self):
        self.detectionArraySub  = rospy.Subscriber('vision/left_camera/detection_array', DetectionArray, self.callback)
        self.gateDetection = None 
        self.range_to_gate = None
        self.gate_width = None
        self.threshold = 0.05
    
    def callback(self, data):
        FOUND_GATE = False
        for detection in data.detections:
            if detection.name == "Gate":
                self.gateDetection = detection
                FOUND_GATE = True
        self.range_to_gate = self.gateDetection.range if FOUND_GATE else None
        self.gate_width = self.gateDetection.boundingBox.x * 2
        
    def main(self):
        self.spin()
        direction = self.shimmy()
        if direction != 0:  
            self.orbit()
        self.forward

    def spin(self, obstacle_name):
        # CONTROLS
        # spin around yourself until we see the gate and stop
        # CONTROLS
        # spin to set point of the gate in the center of our vision
        # OBJECT DETECTION
        # see the gate?

        # subscribe to the detected objects
        # search though the subscribed objects until we find the object with name obstacle_name
        OBS_X = self.detectionArray.boundingBox.center.x
        theta = Gate_Protocol.CAMERA_FOV * (OBS_X/Gate_Protocol.X_RES - 0.5)
        # +++ controls --> spin by theta
        # blocking
    
    def generateSetPoints(self):
        # +++ controls --> set current position as origin
        r = self.range_to_gate
        N = np.arange(0,1,Gate_Protocol.NUM_SET_POINTS)
        self.set_points = r*np.array([
            r * (1-np.cos(2*np.pi*N)), 
            r * (np.sin(2*np.pi*N)),
            0, # same z position
            0, # no rotation around x
            0, # no rotation around y
            -2*np.pi*N, # rotation around z
            0,0,0,0,0,0 # all zero velocities
            ]).reshape(-1, 12)


    def shimmy(self):
        ''' returns True if we need to orbit, returns False if we do not need to orbit
            (i.e. we're already in front of the gate)'''
        self.generateSetPoints()
        original_width = self.gate_width
        # controls --> goto self.set_points[5] # going left
        width_left = self.gate_width
        # controls --> goto self.set_points[-5] # going right
        width_right = self.gate_width
        if width_left > original_width and width_right < original_width:
            orbit_direction = +1
        elif width_left < original_width and width_right > original_width:
            orbit_direction = -1
        elif width_left < original_width and width_right < original_width:
            orbit_direction = 0
        elif width_left > original_width and width_right > original_width:
            orbit_direction = +1 # either direction
        # controls --> goto self.set_points[0] # going right
        if orbit_direction < 0:
            self.set_points = np.flip(self.set_points, axis=0)
            return True
        elif orbit_direction > 0:
            return True
        else:
            return False

        # DEPTH SENSING
        # find distance to center of gate
        # CONTROLS / NAVIGATION
        # begin orbit around center of gate
        # OBJECT DETECTION
        # if bounding box shrinks, change direction
        # if bounding box grows, don't worry
        Width = 2*self.detectionArray.boundingBox.x
        #rot


    def orbit(self):
        # DEPTH SENSING
        # find distance to center of gate
        # CONTROLS / NAVIGATION
        # orbit around center of gate while maintaining the gate in the center of our vision
        # OBJECT DETECTION
        # stop when the width of the gate reaches a maximum
        previous_width = self.gate_width
        position_around_circle = 0
        while self.gate_width > previous_width:
            previous_width = self.gate_width
            position_around_circle += 1
            # +++ controls: go to *self.set_points[position_around_circle]
        # +++ controls: go to self.set_points[position_around_circle]
        # PLEASE BE BLOCKING
        pass

    def forward(self):
        # OBJECT DETECTION
        # see the gate! remember where it starts
        # publish the set point as it is
        # continue to check where the gate is and republish the set point if neccessary
        # +++ controls: reset_position = 
        forward = np.zeros((12,))
        forward[0] = Gate_Protocol.FORWARD_STEP_SIZE
        while not self.range_to_gate is None:
            # +++ controls: go to forward
            # spin
            pass
        #  +++ controls: go to forward
        print("we reached the gate")

# def callback(detections):
#     FOUND_OBJECT = False
#     THETA = None
#     THRESH = 0.1
#     for det in detections:
#         if det.name == obstacle_name:
#             if not FOUND_OBJECT:
#                 FOUND_OBJECT = True
#                 THETA = det.boundingBox.theta
#     spin(-THETA)
#     __________________________
#     shimmy(det.range)
#     forward()
    

"""

def fixed_depth(self):
    # CONTROLS
    # move down by a fixed amount
    pass

def spin(self, obstacle_name):
    # CONTROLS
    # spin around until we see the gate and stop
    # CONTROLS
    # spin to set point of the gate in the center of our vision
    # OBJECT DETECTION
    # see the gate?

    # subscribe to the detected objects
    # search though the subscribed objects until we find the object with name obstacle_name
    FOUND_OBJECT = False
    OBS_X = None

    def callback(detections):
        # data = [
        #   [detection1, detection2]
        #   detection
        # detection1.y
        # ]
        #detection message
        # Header header
        # string name
        # float64 range
        # float64 theta
        # float64 phi
        # float64 confidence
        # vision_msgs/BoundingBox2D boundingBox
                #boundingBox.center.x
                #boundingBox.center.y
        for det in detections:
            if det.name == obstacle_name:
                if not FOUND_OBJECT:
                    FOUND_OBJECT = True
                    OBS_X = det.boundingBox.center.x
                
    rospy.Subscriber('vision/detection_array', DetectionArray, callback)

    pass"""

def main():
    rospy.init_node('gate_alignment')
    rospy.Subscriber('vision/detection_array', DetectionArray, callback)
    rospy.spin()

if __name__ == "__main__":
    main()