#!/usr/bin/env python
from VehicleModel import *
import rospy
from std_msgs.msg import Float64

class Controller:
    def __init__(self):
        self.model = AUV()

        self.kP = .00001
        self.kI = 0
        self.kD = .005


        self.error = 0
        self.derivativeError = 0
        self.integralError = 0

        self.previousDerivativeError = 0

        self.setpoint = 0

    def updateErrors(self, deltaT):

        # Store for the sake of derivative
        prevError = self.error

        # Calculate the new error
        self.error = self.setpoint - self.model.getZPosition()
        self.derivativeError = 0.7 * self.previousDerivativeError + 0.3 * (self.error - prevError)/deltaT
        self.previousDerivativeError = self.derivativeError
        self.integralError += self.error * deltaT


    def computeControl(self):
        P = self.kP * self.error
        I = self.kI * self.integralError
        D = self.kD * self.derivativeError
        rospy.loginfo("Error: {}, Deriv: {}, kD: {}, D: {}".format(self.error, self.derivativeError, self.kD, D))
        return (P + I + D)




if __name__ == '__main__':
    rospy.init_node("Controller", anonymous=True)

    controller = Controller()

    def kpCB(msg):
        controller.kP = msg.data
    def kiCB(msg):
        controller.kI = msg.data
    def kdCB(msg):
        controller.kD = msg.data
    def setpointCB(msg):
        controller.setpoint = msg.data

    kPSub = rospy.Subscriber("kP", Float64, kpCB)
    kISub = rospy.Subscriber("kI", Float64, kiCB)
    kDSub = rospy.Subscriber("kD", Float64, kdCB)

    setpointSub = rospy.Subscriber("setpoint", Float64, setpointCB)

    loopRate = rospy.Rate(10)

    t = rospy.get_time()

    while not rospy.is_shutdown():
        deltaT = rospy.get_time() - t
        controller.updateErrors(deltaT)
        controlEffort = controller.computeControl()

        control = ControlInput([0,0,controlEffort])
        controller.model.propogateControls(control, deltaT)

        loopRate.sleep()
