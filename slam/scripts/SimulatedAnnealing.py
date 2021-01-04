import random
import numpy as np
import math
import pickle
#from FastSLAMEvolution import evaluateFastSLAM
#from DataLoaderEvolution import *

class SimulatedAnnealing:
    def __init__(self, s0, t0, tf, evaluation, findNeighbors, tempFx, iterations, goodVal=0, alpha= None, beta=None ):
        '''
        s: current solution, starting with initial solution
        t: current temperature, starting with initial temperature
        tf: final temperature (termination condition)
        evaluation: evaluation function
        tempFx: temperature update function. linear, geometric, slow-decrease, or custom
        iteration: number of iterations
        goodVal: acceptable value
        findNeighbors

        alpha and beta: temperature change parameters
        '''
        self.s = s0
        self.t = t0
        self.tf = tf
        self.evaluation = evaluation
        self.cost = self.evaluation(self.s)
        self.tempFx, self.alpha, self.beta = tempFx, alpha, beta
        self.iterations = iterations
        self.goodVal = goodVal
        self.findNeighbors = findNeighbors
        self.bestSolution, self.minimalCost, self.firstIter = self.s, float('inf'),0
        self.history = [(self.cost, s0, 0)]

    def tempUpdate(self):
        tempFx, alpha, beta = self.tempFx, self.alpha, self.beta
        if tempFx == "linear":
            self.t -= alpha
        elif tempFx == "geometric":
            self.t /= alpha
        elif tempFx == "slow-decrease":
            self.t = self.t / (1 + beta * self.t)
        else:
            self.t = tempFx(self.t)

    def run(self):
        n = 0
        while self.t >= self.tf or (self.goodVal and cost <= self.goodVal) :
            i = 0
            for i in range(self.iterations):
                n+=1
                s1 = self.findNeighbors(self.s)
                #neighborIndex = np.random.choice(range(len(neighbors)))
                #s1 = neighbors[neighborIndex]
                newCost = self.evaluation(s1)
                prob = 1

                if newCost < self.cost:
                    print("Accepted change, difference in cost was", newCost - self.cost)
                    self.s = s1
                    self.cost = newCost
                else:
                    diffCost = newCost - self.cost
                    prob = np.exp(-1*diffCost/self.t)
                    if prob >= np.random.random():
                        print("Accepted change, difference in cost was", diffCost)
                        self.s = s1
                        self.cost = newCost
                    else:
                        print("Rejected change, difference in cost was", diffCost)

                if self.cost < self.minimalCost:
                    self.bestSolution = self.s
                    self.minimalCost = self.cost
                    self.firstIter = n
                print("num iteration", n, "cost", self.cost, "solution",self.s, "temperature", self.t, "probability", prob)

                self.history += [(self.cost, self.s, self.firstIter)]

                # if self.goodVal!=None and self.cost<=self.goodVal:
                #     print("optimal: cost",self.minimalCost,"solution",self.bestSolution,"@ iter",self.firstIter)
                #     return

            self.tempUpdate()

        print("initial: cost", self.history[0][0], "solution", self.history[0][1], "@ iter", self.history[0][2])
        print("final: cost", self.history[-1][0], "solution", self.history[-1][1], "@ iter", self.history[-1][2])
        print("optimal: cost", self.minimalCost, "solution", self.bestSolution, "@ iter", self.firstIter)
        self.history.sort(key=lambda s:s[0])
        print("worst: cost", self.history[-1][0], "solution", self.history[-1][1], "@ iter", self.history[-1][2])
'''
particleChanges = 0

if __name__ == "__main__":
    data = pickle.load(open("Jar/dataset1.pkl", "rb"))
    default = np.array([5, 0.04, 0.0125, 0.07, 0.075, 0.025, 0.03, 0.015])
    # s0 = np.array([5, 0.04, 0.0125, 0.07, 0.075, 0.025, 0.03, 0.015])
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
        pathRMS = evaluateFastSLAM(data, s)
        return pathRMS

    print("default: cost", evaluation(default))

    # def findNeighbors(s):
    #     ls = []
    #     for i in [-1,0,1]:
    #         for j in [-1,0,1]:
    #             for k in [-1,0,1]:
    #                 ls.append(s+np.array([i,j,k]))
    #     return ls

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

    sa = simulatedAnnealing(s0, t0, tf, evaluation, \
                        findNeighbors, tempFx, \
                        iterations, goodVal, \
                        alpha, beta)
    sa.run()

    print("Num particle changes", particleChanges)
'''
