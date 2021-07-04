#!/usr/bin/env python

import rospy
import numpy as np
from SimulatedAnnealing import SimulatedAnnealing
from slam.srv import *

import pickle 
def evaluateFastSLAM(params):
    '''Send request for the FastSLAM evaluation with the given parameters and wait for the response'''
    #print("evaluating parameters",params)
    rospy.wait_for_service("evalFastSLAM")    
    rms = None
    try:
        evalFastSLAM = rospy.ServiceProxy("evalFastSLAM", EvalFastSLAM)
        res = evalFastSLAM(int(params[0]), *params[1:])
        rms = res.pathRMS
    except rospy.ServiceException as e:
        print("Service call failed for slam params tuning:", str(e))
    return rms
      


if __name__=="__main__":
    print("start parameter evaluation client")
    # number of times we run fastSlam 
    data = {}
    for particle in [50,5]:
        parameters = np.array([particle, 0.04, 0.0125, 0.07, 0.075, 0.025, 0.03, 0.015])
        pathRMS = []
        iteration = 0
        #[5,10,25,50,75,100]
        for iterationInc in [5,5,15,25,25,25]:
        # Setup hyperparameters for simulated annealing algorithm
            

            print("progress: total iteration",iteration,"to",iteration+iterationInc,"for ",particle,"particles")
            

            for i in range(iterationInc):
                pathRMS.append(evaluateFastSLAM(parameters))
                print("particle",particle,"iteration",i+iteration,"rms",pathRMS[-1])
            
            npPathRMS = np.array(pathRMS)
            data[(tuple(parameters),iteration)] = pathRMS
            print("number of iterations",iteration,"number of particles",parameters[0])
            print("mean",np.mean(npPathRMS),"std",np.std(npPathRMS),"max",np.max(npPathRMS),"min",np.min(npPathRMS))
            iteration+= iterationInc
    print("\nexperiment summary\n")
    for para in data:
        parameters, iteration = para
        pathRMS = data[para]
        print("number of iterations",iteration,"number of particles",parameters[0])
        print("mean",np.mean(pathRMS),"std",np.std(pathRMS),"max",np.max(pathRMS),"min",np.min(pathRMS))

    fileObj = open("FastSlamExperiment","wb")
    pickle.dump(data,fileObj)


