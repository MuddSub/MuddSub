#!/usr/bin/env python
import numpy as np
import rospy
from vision.msg import DetectionArray
class Gate_Protocol():
    CAMERA_FOV = 2
    CAMERA_XRES = 416
    NUM_SET_POINTS = 72

    def __init__(self):
        self.detectionArraySub  = rospy.Subscriber('vision/left_camera/detection_array', DetectionArray, callback)
        self.detectionArray = None #
        self.range_to_gate = None
        self.threshold = 0.05
    
    def main(self):
        pass
        self.spin()
        self.orbit()
        # self....

    def detectionArrayCallback(self, data):
        self.range_to_gate = data.range
        self.detectionArray = data

    def spin(self, obstacle_name):
        # CONTROLS
        # spin around until we see the gate and stop
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
        r = self.detectionArray.boundingBox.range
        N = np.arange(0,1,Gate_Protocol.NUM_SET_POINTS)
        self.set_points = r*np.array([1-np.cos(2*np.pi*N), np.sin(2*np.pi*N)]).reshape(-1, 2)

        


    def shimmy(self):
        
        original_width = self.detectionArray.boundingBox.x * 2
        # controls --> goto self.set_points[5]
        width_left = self.detectionArray.boundingBox.x * 2
        # controls --> goto self.set_points[-5]
        width_right = self.detectionArray.boundingBox.x * 2
        if width_left > original_width and width_right < original_width:
            orbit_direction = +1
        elif width_left < original_width and width_right > original_width:
            orbit_direction = -1
        elif width_left < original_width and width_right < original_width:
            orbit_direction = 0
        elif width_left > original_width and width_right > original_width:
            orbit_direction = +1 # either direction
        return orbit_direction
        # TODO : include bearing information in set points

        # DEPTH SENSING
        # find distance to center of gate
        # CONTROLS / NAVIGATION
        # begin orbit around center of gate
        # OBJECT DETECTION
        # if bounding box shrinks, change direction
        # if bounding box grows, don't worry
        Width = 2*self.detectionArray.boundingBox.x
        #rot


    



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

    pass


def orbit(self):
    # DEPTH SENSING
    # find distance to center of gate
    # CONTROLS / NAVIGATION
    # orbit around center of gate while maintaining the gate in the center of our vision
    # OBJECT DETECTION
    # stop when the width of the gate reaches a maximum
    pass

def forward(self):
    # OBJECT DETECTION
    # see the gate! remember where it starts
    # publish the set point as it is
    # continue to check where the gate is and republish the set point if neccessary
    pass

def callback(detections):
    FOUND_OBJECT = False
    THETA = None
    THRESH = 0.1
    for det in detections:
        if det.name == obstacle_name:
            if not FOUND_OBJECT:
                FOUND_OBJECT = True
                THETA = det.boundingBox.theta
    spin(-THETA)
    __________________________
    shimmy(det.range)
    forward()
    

def main():
    rospy.init_node('gate_alignment')
    rospy.Subscriber('vision/detection_array', DetectionArray, callback)
    rospy.spin()

if __name__ == "__main__":
    main()