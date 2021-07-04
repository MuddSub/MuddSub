#!/usr/bin/env python

import rospy
import numpy as np
from SimulatedAnnealing import SimulatedAnnealing
from slam.srv import *
from SLAMEvalClient import evaluateFastSLAM
import pickle

if __name__=="__main__":
    print("start parameter tuning client")
    # number of times we run fastSlam
    experimentRepetitions = 5

    # Setup hyperparameters for simulated annealing algorithm
    default = np.array([5, 0.04, 0.0125, 0.07, 0.075, 0.025, 0.03, 0.015])
    t0 = 100
    tf = 0.1
    tempFx = "slow-decrease"
    iterations = 1
    alpha = 10
    beta = 0.1
    goodVal = 0
    steps = [5] + [0.0005] * 7

    def randomIndividual():
        '''Creates a random set of parameters for FastSLAM'''
        bounds = [(10, 50)] + [(0.0001, 0.2)] * 7
        individual = []
        for bound in bounds:
            lower, upper = bound
            individual += [np.random.uniform(lower, upper)]
        individual[0] = int(individual[0])
        return np.array(individual)

    s0 = randomIndividual()

    def evaluation(s):
        '''Define evaluation function for the simulated annealing algorithm'''
        print("evaluating parameters",s)
        pathRMS = []
        for i in range(experimentRepetitions):
          pathRMS.append(evaluateFastSLAM(s))
        return sum(pathRMS)/len(pathRMS)


    def findNeighbors(s):
        '''Returns a set of parameters neighoring the given one'''

        neighborIndex = np.random.choice(range(len(s)))
        diff = steps[neighborIndex] * np.random.choice([1, -1]) * (np.random.random()+.00001)
        if neighborIndex == 0:
            diff = int(diff)
        newS = s.copy()
        newS[neighborIndex] += diff
        newS[0] = max(1, newS[0])
        return newS

    sa = SimulatedAnnealing(s0, t0, tf, evaluation, \
            findNeighbors, tempFx, \
            iterations, goodVal, \
            alpha, beta)
    sa.run()
    fileObj = open("FastSlamHistory","wb")
    pickle.dump(sa.history,fileObj)


    print("default: cost", evaluation(default))
