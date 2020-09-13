#!/usr/bin/env python

import numpy as np
import rospy
import controls.msg

class LocalState:
    def __init__(self, state = [0, 0, 0]):
        self.xDot = state[0]
        self.zDot = state[1]
        self.thetaDot = state[2]

class WorldState:
    def __init__(self, state = [0,0,0,0,0,0,0,0]):
        self.x = state[0]
        self.y = state[1]
        self.z = state[2]
        self.theta = state[3]
        self.xDot = state[4]
        self.yDot = state[5]
        self.zDot = state[6]
        self.thetaDot = state[7]


    def getStateVector(self):
        return [self.x, self.y, self.z, self.theta, self.xDot, self.yDot, self.zDot, self.thetaDot]

    def get12StateVector(self):
        return  [self.x, self.y, self.z, 0, 0, self.theta, self.xDot, self.yDot, self.zDot, 0, 0, self.thetaDot]

    def __repr__(self):
        return str(self.getStateVector())

class ControlInput:
    def __init__(self, control=[0,0,0]):
        self.left = control[0]
        self.right = control[1]
        self.vert = control[2]

class AUV:

    def __init__(self):
        self.mass = 5
        self.radius = 1

        self.robotState = LocalState()
        self.worldState = WorldState()

        self.buoyancy = .001
        self.dragConstant = 0.5
        self.angularDragConstant = 0.5

        self.pub = rospy.Publisher("robot_state_raw", controls.msg.State, queue_size=5, latch=True)

    def propogateControls(self, control, deltaT):
        robotState = self.robotState
        xDotRobot = robotState.xDot
        zDotRobot = robotState.zDot
        thetaDotRobot = robotState.thetaDot


        # Update robot's state
        aX = (control.left + control.right) / self.mass
        if aX > 1:
            aX = 1
        aZ = (control.vert)/self.mass
        if aZ > 1:
            aZ = 1
        alpha = (control.left - control.right) / (self.mass*self.radius)

        robotState.xDot += aX * deltaT
        robotState.zDot += aZ * deltaT
        robotState.thetaDot = robotState.thetaDot + alpha * deltaT


        # Update state in world frame
        worldState = self.worldState

        theta = worldState.theta
        worldState.xDot = robotState.xDot * np.cos(theta)
        worldState.yDot = robotState.xDot * np.sin(theta)
        worldState.zDot = robotState.zDot
        worldState.thetaDot = robotState.thetaDot

        worldState.x += worldState.xDot * deltaT
        worldState.y += worldState.yDot * deltaT
        worldState.z += worldState.zDot * deltaT
        rospy.loginfo(worldState.z)
        worldState.theta += worldState.thetaDot * deltaT

        stateMsg = controls.msg.State()
        stateMsg.state = self.worldState.get12StateVector()
        self.pub.publish(stateMsg)

    def getState(self):
        return self.worldState.get12StateVector()

    def getZPosition(self):
        return self.worldState.z


if __name__ == '__main__':
    rospy.init_node("VehicleDynamics")

    auv = AUV()
    c = ControlInput([0,0,0])
    rate = rospy.Rate(10)
    for _ in range(4):
        auv.propogateControls(c, 1)
        rate.sleep()
