#!/usr/bin/env python
from VehicleModel import *
import rospy
from std_msgs.msg import Float64

class Controller:
    """
    This implements a PID controller for the vertical axis of a robot, in a simple
    mode.
    """
    def __init__(self):
        self.model = AUV()


        # Proportional Gain
        self.kP = .00001

        # Integral Gain
        self.kI = 0

        # Derivative Gain
        self.kD = .005

        # setpoint - plantState
        self.error = 0

        # dError/dt
        self.derivativeError = 0

        # integral (error) dt
        self.integralError = 0

        # This is used to filter the derivative error, as sudden impulses
        # make the system unreliable
        self.previousDerivativeError = 0

        # The desired vertical location of the robot
        self.setpoint = 0

    def updateErrors(self, deltaT):
        # Note: the setpoint is self.setpoint, and the current state is self.model.getZPosition()
        pass

    def computeControl(self):
        # Compute the PID controller, and return the sum of P+I+D
        # I recommend setting the I term to always be 0 by never changing kI
        return 0



if __name__ == '__main__':
    rospy.init_node("Controller", anonymous=True)

    controller = Controller()

    # This is the callback which will be called when a Float64 message is recieved on the /kP topic
    def kPCallback(msg):
        controller.kP = msg.data
    """
    Fill in the callbacks for the other subscribers. These are the topics:
    - /kI: Store the result in controller.kI
    - /kD: Store the result in controller.kD
    - /setpoint: Store the result in controller.setpoint
    """

    # This declares the subscriber, and specifies to call the kP callback
    kPSub = rospy.Subscriber("kP", Float64, kPCallback)
    """
    Declare the other three subscribers
    """

    loopRate = rospy.Rate(10)

    # Get the system time in seconds
    t = rospy.get_time()

    # This loops until ROS sends a shutdown
    while not rospy.is_shutdown():

        # Calculate the time step
        deltaT = rospy.get_time() - t

        # Update the error, derivative, and integral
        controller.updateErrors(deltaT)

        # Calculate the control effort
        controlEffort = controller.computeControl()

        # Construct a ControlInput object with the only effort on the vertical thruster
        control = ControlInput([0,0,controlEffort])

        # Apply the computed control to the robot
        controller.model.propogateControls(control, deltaT)

        loopRate.sleep()
