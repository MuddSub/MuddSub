#!/usr/bin/env python

import rospy
from bayes_opt import BayesianOptimization
from bayes_opt.logger import JSONLogger
from bayes_opt.event import Events
import rospkg
from std_msgs.msg import Empty
from controls.msg import State
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time

class Tuner:

    def __init__(self):
        self.axes = ["surge", "sway", "heave", "roll", "pitch", "yaw"]
        self.axis = "heave"

        self.setpoints = [3,1,5]
        self.plantState = None

        self.secondsPerStep = 5

        pBounds = {"kP" : (0, 100), "kI": (0, 10), "kD": (0, 100)}
        self.optimizer = BayesianOptimization(f = self.iterate,
                                              pbounds = pBounds,
                                              random_state = 1,
                                              verbose=0)

        rospack = rospkg.RosPack()
        path = rospack.get_path("controls") + "/tune_logs"
        logger = JSONLogger(path=path + "/logs.json")
        self.optimizer.subscribe(Events.OPTIMIZATION_STEP, logger)

        self.resetPub = rospy.Publisher("/reset_simulation", Empty, queue_size=1)
        self.setpointPub = rospy.Publisher("/robot_setpoint", State, queue_size=1)

        self.odomSub = rospy.Subscriber("/sim/ground_truth_pose", Odometry, self.storeState)

    def optimize(self, initialPoints = 10, numIts = 100):
        self.optimizer.maximize(init_points = initialPoints, n_iter = numIts)
        print(self.optimizer.max)

    def storeState(self, msg):
        p = msg.pose.pose.position
        x,y,z = p.x, p.y, p.z

        q = msg.pose.pose.orientation
        qList = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(qList)

        self.plantState = [x,y,z,roll,pitch,yaw]


    def iterate(self, kP, kI, kD):
        rate = rospy.Rate(20)
        rospy.loginfo("Trying: kP: {}, kI: {}, kD: {}".format(kP, kI, kD))

        rospy.set_param(self.axis+"/kP", float(kP))
        rospy.set_param(self.axis+"/kI", float(kI))
        rospy.set_param(self.axis+"/kD", float(kD))

        integralError = 0

        # Index of current axis in state vector
        axisIndex = self.axes.index(self.axis)

        # Reset the simulation
        emptyMsg = Empty()
        self.resetPub.publish(emptyMsg)

        while self.plantState is None:
            # Use python sleep since rospy sleep will hang if using sim time
            # and simulation starts paused
            time.sleep(0.1)

        startTime = rospy.get_time()
        totalDuration = self.secondsPerStep * len(self.setpoints)

        setpointIndex = -1

        prevTime = startTime
        newTime = startTime

        while not rospy.is_shutdown():
            newTime = rospy.get_time()

            # Which setpoint are we currently targeting
            newIndex = int((newTime-startTime)  // self.secondsPerStep)

            if newIndex == len(self.setpoints):
                break

            # If the setpoint has changed, re-publish it
            if newIndex != setpointIndex:
                setpointIndex = newIndex
                rospy.loginfo("New Setpoint: {}".format(self.setpoints[setpointIndex]))

                stateMsg = State()
                stateMsg.state = [0 for _ in range(12)]
                stateMsg.state[axisIndex] = self.setpoints[setpointIndex]

                self.setpointPub.publish(stateMsg)

            plantState = self.plantState[axisIndex]

            error = abs(self.setpoints[setpointIndex] - plantState)
            integralError += (newTime - prevTime) * error
            prevTime = newTime
            rate.sleep()


        rospy.loginfo("Integral Error: {}".format(integralError))
        return -integralError

if __name__ == '__main__':
    rospy.init_node("autotune")

    tuner = Tuner()
    tuner.optimize()
