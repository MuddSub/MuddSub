import numpy as np

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

    def __repr__(self):
        vector = [self.x, self.y, self.z, self.theta, self.xDot, self.yDot, self.zDot, self.thetaDot]
        return str(vector)

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

        self.buoyancy = .25
        self.dragConstant = 0.5
        self.angularDragConstant = 0.5

    def propogateControls(self, control: ControlInput, deltaT: float):
        robotState = self.robotState
        xDotRobot = robotState.xDot
        zDotRobot = robotState.zDot
        thetaDotRobot = robotState.thetaDot


        # Update robot's state
        aX = (control.left + control.right) / self.mass - self.dragConstant * xDotRobot
        aZ = (control.vert)/self.mass - self.buoyancy
        alpha = (control.left - control.right) / (self.mass*self.radius) - \
                self.angularDragConstant * thetaDotRobot

        robotState.xDot += aX * deltaT
        robotState.zDot += aZ * deltaT
        robotState.thetaDot = robotState.thetaDot + alpha * deltaT


        # Update state in world frame
        worldState = self.worldState

        theta = worldState.theta
        worldState.xDot = xDotRobot * np.cos(theta)
        worldState.yDot = xDotRobot * np.sin(theta)
        worldState.zDot = zDotRobot
        worldState.thetaDot = thetaDotRobot

        worldState.x += worldState.xDot * deltaT
        worldState.y += worldState.yDot * deltaT
        worldState.z += worldState.zDot * deltaT
        worldState.theta += worldState.thetaDot * deltaT

        print(worldState)

if __name__ == '__main__':
    auv = AUV()
    c = ControlInput([1,0.8,1])
    for _ in range(4):
        auv.propogateControls(c, 1)
