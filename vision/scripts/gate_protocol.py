#!/usr/bin/env python3
import numpy as np
import rospy
from vision.msg import DetectionArray
import time
class Gate_Protocol():
    CAMERA_FOV = 2 # check with our camera
    CAMERA_XRES = 416 # check with our camera
    NUM_SET_POINTS = 72
    FORWARD_STEP_SIZE = 1

    def __init__(self):
        rospy.init_node('gate_alignment')
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
        self.gate_width = self.gateDetection.boundingBox.size_x * 2 if FOUND_GATE else self.gate_width
        
    def main(self):
        # check if self.gate_width is not None

        success = self.spin()
        if success:
            direction = self.shimmy()
            if direction:  #fix returns
                self.orbit()
            self.forward()
        else:
            return False

    def spin(self):
        # CONTROLS
        # spin around yourself until we see the gate and stop
        # CONTROLS
        # spin to set point of the gate in the center of our vision
        # OBJECT DETECTION
        # see the gate?

        # subscribe to the detected objects
        # search though the subscribed objects until we find the object with name obstacle_name

        starting_time = time.time()

        while self.gateDetection == None:
            current_time = time.time()
            if current_time - starting_time > 30:
                return False
            print("Cannot find Gate")
        OBS_X = self.gateDetection.boundingBox.center.x # pixel vs normalized 0 - 1
        theta = Gate_Protocol.CAMERA_FOV * (OBS_X/Gate_Protocol.CAMERA_XRES - 0.5)
        if self.controls_spin(theta):
            return True
        # blocking
    
    def generateSetPoints(self):
        # +++ controls --> set current position as origin
        self.controls_origin()
        r = self.range_to_gate #this is in the x direction
        # check if 1 is inclusive 
        N = np.arange(0,1,Gate_Protocol.NUM_SET_POINTS)
        self.set_points = r*np.array([
            r * (1-np.cos(2*np.pi*N)), 
            r * (np.sin(2*np.pi*N)),
            np.zeros(Gate_Protocol.NUM_SET_POINTS), # same z position
            np.zeros(Gate_Protocol.NUM_SET_POINTS), # no rotation around x
            np.zeros(Gate_Protocol.NUM_SET_POINTS), # no rotation around y
            -2*np.pi*N, # rotation around z counterclockwise
            np.zeros(Gate_Protocol.NUM_SET_POINTS),np.zeros(Gate_Protocol.NUM_SET_POINTS),
            np.zeros(Gate_Protocol.NUM_SET_POINTS),np.zeros(Gate_Protocol.NUM_SET_POINTS),
            np.zeros(Gate_Protocol.NUM_SET_POINTS),np.zeros(Gate_Protocol.NUM_SET_POINTS) # all zero velocities
            ])
        self.set_points = np.column_stack(self.set_points)

    def shimmy(self):
        ''' returns True if we need to orbit, returns False if we do not need to orbit
            (i.e. we're already in front of the gate)'''
        self.generateSetPoints() #clockwise rotation
        original_width = self.gate_width
        if self.controls_goto(self.set_points[5]):
            width_left = self.gate_width
        if self.controls_goto(self.set_points[-5]):
            width_right = self.gate_width
        if width_left > original_width and width_right < original_width:
            orbit_direction = +1
        elif width_left < original_width and width_right > original_width:
            orbit_direction = -1
        elif width_left < original_width and width_right < original_width:
            orbit_direction = 0
        elif width_left > original_width and width_right > original_width:
            orbit_direction = +1 # either direction
        if self.controls_goto(self.set_points[0]):
            if orbit_direction < 0:
                self.set_points = np.flip(self.set_points, axis=0)
                return True
            elif orbit_direction > 0:
                return True
            else:
                return False


    def orbit(self):
        # DEPTH SENSING
        # find distance to center of gate
        # CONTROLS / NAVIGATION
        # orbit around center of gate while maintaining the gate in the center of our vision
        # OBJECT DETECTION
        # stop when the width of the gate reaches a maximum
        previous_width = self.gate_width        
        position_around_circle = 1
        self.controls_goto(self.set_points[position_around_circle%self.NUM_SET_POINTS])
        
        while True:
            starting_time = time.time()
            while self.gate_width == None:
                current_time = time.time()
                if current_time - starting_time > 5:
                    self.controls.goto(self.set_points[(position_around_circle+1)%self.NUM_SET_POINTS])
                    position_around_circle += 1
                    break
                #wait a bit for the gate to not be none otherwise just go to the next position blind
            if self.gate_width != None and self.gate_width > previous_width:
                previous_width = self.gate_width
                position_around_circle += 1
                self.controls_goto(self.set_points[position_around_circle%self.NUM_SET_POINTS])
            elif self.gate_width !=None and self.gate_width <= previous_width:
                if self.controls_goto(self.set_points[(position_around_circle-1)%self.NUM_SET_POINTS]):
                    break
    
    def controls_spin(self, theta):
        '''spin around ourselves'''
        print("spun by ", theta)
        return True

    def controls_origin(self):
        print("set our position as the origin")
        return True
    
    def controls_goto(self, position):
        print("went to " + str(position))
        return True
        
    # def forward(self):
    #     # OBJECT DETECTION
    #     # see the gate! remember where it starts
    #     # publish the set point as it is
    #     # continue to check where the gate is and republish the set point if neccessary
    #     # +++ controls: reset_position = 
    #     forward = np.zeros((12,))
    #     forward[0] = Gate_Protocol.FORWARD_STEP_SIZE
    #     while not self.range_to_gate is None:
    #         # +++ controls: go to forward
    #         # spin
    #         pass
    #     #  +++ controls: go to forward
    #     print("we reached the gate")



if __name__ == "__main__":
    robot = Gate_Protocol()
    robot.main()