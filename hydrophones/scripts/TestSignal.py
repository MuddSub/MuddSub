import Signal

import matplotlib.pyplot as plt
import math
import random
import numpy

def graph_signals(time, array):
    plt.plot(time, array[0], 'r-', time, array[1], 'g-', time, array[2], 'b-', time, array[3], 'k-')

    plt.xlabel('Time (ms)')
    plt.ylabel('Charge (arbitrary)')
    plt.show()

def main():
    d = 20 #m
    angle = 1.8 #rad
    coords = [[0.005, 0.005], [0.005, -0.005], [-0.005, -0.005], [-0.005, 0.005]]
    freq = 45 #kHz
    sos = 1.5 #m/ms
    sr = 1000 #kHz
    read = 1500 #ms
    noise = 1 #standard deviation
    time_on = 100 #ms --ping duration
    ping_period = 1000 #ms
    ping_start = random.random() * ping_period

    #create an array containing the simulated readings arrays of each sensor
    signals = []
    for i in range(len(coords)):
        signals.append(Signal.Signal(d, angle, coords[i], freq, sos, sr, read, noise, time_on, ping_period, ping_start).signals)
    times = numpy.arange(0, read, 1/sr)
    print(times,signals)
    graph_signals(times, signals)

if __name__ == "__main__":
    main()
