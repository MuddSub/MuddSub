import matplotlib.pyplot as plt
import numpy as np
import math
import pickle

if __name__ == "__main__":
    # Array of (cost, parameters, ?, temperature, iteration)
    history = pickle.load(open("../datasets/Jar/FastSLAMHistory.pkl", "rb"))
    iterations = range(len(history))
    costs = [slice[0] for slice in history]
    print("First parameters:", history[0][1])

    avgRMS = sum(costs) / len(history)
    varRMS = sum([(cost - avgRMS) ** 2 for cost in costs]) / len(history)
    stdRMS = varRMS ** 0.5
    print("avgRMS:", avgRMS)
    print("varRMS:", varRMS)
    print("stdRMS:", stdRMS)

    plt.plot(iterations, costs, "bo-", label="RMS")
    # plt.legend(loc="best")
    plt.xlabel("Iteration #")
    plt.ylabel("RMS")

    particles = [slice[1][0] for slice in history]

    plt.figure()
    plt.plot(iterations, particles, "ro-", label="particles")
    plt.xlabel("Iteration #")
    plt.ylabel("Particles")

    plt.show()
