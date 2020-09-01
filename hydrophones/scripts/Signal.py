import math
import numpy
import random

###calculate the distance to sensor based on ref point
#coords: [x, y] distances in m from ref point
#distance: distance to ref point
#angle: between x-axis and ping to ref point
def calc_distance(coords, distance, angle):
    x_source = distance * math.cos(angle)
    y_source = distance * math.sin(angle)
    return math.sqrt((coords[0] - x_source)**2 + (coords[1] - y_source)**2)

###calculate a single reading of the sensor (pure sine wave)
#distance: distance from source to sensor (m)
#time: current reading time (ms)
#frequency: ping frequency (kHz)
#sound_speed: current speed of sound (m/ms)
#amplitude: amplitude of sine wave
def calc_sine(distance, time, frequency, sound_speed, amplitude):
    tot_time = distance / sound_speed + time
    return amplitude * math.sin(2 * math.pi * frequency * tot_time)

class Signal:
    ###initializer
    def __init__(self, distance, angle, coords, frequency, sound_speed, sample_rate, read_time, noise, time_on, ping_period, ping_start):
        self.distance = calc_distance(coords, distance, angle) #m
        self.angle = angle #rad between 0 and pi
        self.frequency = frequency #kHz
        self.sound_speed = sound_speed #m/ms
        self.sample_rate = sample_rate #kHz
        self.read_time = read_time #ms
        self.noise = noise #standard deviation
        self.time_on = time_on #ms
        self.ping_period = ping_period #ms
        self.ping_start = ping_start #ms

        self.signals = self.generate_signals()

    ###generate complete simulated signal reading array for the sensor
    def generate_signals(self):
        signal = []
        #determine the start time of the first ping
        start = self.ping_start + self.distance / self.sound_speed
        if(start > self.ping_period - self.time_on):
            start -= self.ping_period
        print(start)
        #create echo parameters for random echo
        echo_length = random.random() * 100
        echo_amp = random.uniform(0.3, 0.5)
        echo_freq = random.uniform(0.9, 1.1) * self.frequency
        #determine the start time of the echo's first ping
        echo_start = self.ping_start + self.distance + echo_length / self.sound_speed
        if(echo_start > self.ping_period - self.time_on):
            echo_start -= self.ping_period

        #calculate the total number of samples for the sensor
        samples = math.floor(self.sample_rate * self.read_time)
        #create a noise array to add to the pings
        noise_array = numpy.random.normal(0, self.noise, samples)
        #calculate each ping reading with echoes
        for i in range(samples):
            sig = 0
            #add ping sine wave if sensor is recieving a ping
            #print((i / self.sample_rate - start), self.time_on);
            if(((i / self.sample_rate - start) % self.ping_period) < self.time_on):
                sig += calc_sine(self.distance, i / self.sample_rate, self.frequency, self.sound_speed, 1)
            #add echo sine wave if sensor is recieving an echo
            if(((i / self.sample_rate - echo_start) % self.ping_period) < self.time_on):
                sig += calc_sine(self.distance + echo_length, i / self.sample_rate, echo_freq, self.sound_speed, echo_amp)
            #add noise
            #print(i, sig)
            signal.append(sig + noise_array[i])

        return signal