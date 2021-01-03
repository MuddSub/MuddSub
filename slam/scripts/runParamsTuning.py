#!/usr/bin/env python
import rospy 
import numpy as np
import SimulatedAnnealing
from slam.srv import * 

def evaluateFastSLAM(params):
  rospy.wait_for_service("slam_params_tuning")
  try:
	  score = rospy.ServiceProxy("slam_params_tuning", EvalFastSLAM)
	  return score
  except rospy.ServiceException as e:
	  print("Service call failed for slam params tuning: %s"%e)
particleChanges = 0
if __name__=="__main__":
  print("start parameter tuning clinet")
  default = np.array([5, 0.04, 0.0125, 0.07, 0.075, 0.025, 0.03, 0.015])
  t0 = 100
  tf = 0.1
  tempFx = "slow-decrease"
  alpha = 10
  beta = 0.1
  goodVal = 0
  steps = [1] + [0.0005] * 7
  
  def randomIndividual():
    bounds = [(3, 15)] + [(0.0001, 0.2)] * 7
    individual = []
    for bound in bounds:
      lower, upper = bound
      individual += [np.random.uniform(lower, upper)]
    individual[0] = int(individual[0])
    return np.array(individual)

  s0 = randomIndividual()

  def evaluation(s):
    pathRMS = evaluateFastSLAM(s)
    return pathRMS
  
  print("default: cost", evaluation(default))

  def findNeighbors(s):
    global particleChanges
    neighborIndex = np.random.choice(range(len(s)))
    diff = steps[neighborIndex] * np.random.choice([1, -1])
    newS = s.copy()
    newS[neighborIndex] += diff
    newS[0] = max(1, newS[0])
    if neighborIndex == 0:
      particleChanges += 1
    return newS

  iterations = 1

  print("Num particle changes", particleChanges)

  sa = SimulatedAnnealing.simulatedAnnealing(s0, t0, tf, evaluation, \
            findNeighbors, tempFx, \
            iterations, goodVal, \
            alpha, beta)
  sa.run()

  print("Num particle changes", particleChanges)
