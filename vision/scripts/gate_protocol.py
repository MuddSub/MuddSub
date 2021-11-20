#!/usr/bin/env python
import rospy

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

def shimmy(self):
    # DEPTH SENSING
    # find distance to center of gate
    # CONTROLS / NAVIGATION
    # begin orbit around center of gate
    # OBJECT DETECTION
    # if bounding box shrinks, change direction
    # if bounding box grows, don't worry
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