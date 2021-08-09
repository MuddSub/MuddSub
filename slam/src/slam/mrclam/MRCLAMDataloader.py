import pandas as pd
import argparse
import os
import copy
import numpy as np
import pickle
import scipy.interpolate as interp
import matplotlib.pyplot as plt

class Map:
    def __init__(self, data):
        self.data = data
        self.landmark_dict = self.data.set_index('Subject').to_dict('index')

    def __getitem__(self, subject_ID):
        return self.landmark_dict[subject_ID]

    def get_landmark_location(self, subject_ID):
        return self.landmark_dict[subject_ID]

class Robot:
    # All arguments are corresponding pandas dataframes
    def __init__(self, groundtruth, measurements, odometry, barcodes):
        self.ground_truth_DF = groundtruth
        self.measurements_DF = measurements
        self.odometry_DF = odometry
        self.barcodes_DF = barcodes

        # List of data in order. Serves as a backup to dataQueue
        self.data_list = []

        # List of data we will pop from.
        self.data_queue = []

        self.odometry = []
        self.measurements = []
        self.ground_truth_position = []

        self.robot_data = []

        self.build_dict()

    # Get next data. May include one or more of ground truth, measurement, and barcode
    def get_next(self):
        return self.robot_data.pop(0)

    def empty(self):
        return len(self.robot_data) == 0

    def size(self):
        return len(self.robot_data)

    def reset(self):
        self.data_queue = copy.deepcopy(self.robot_data)

    def get_compass(self, t):
        if t < self.ground_truth_times[0]:
            return self.ground_truth_compass[0]
        if t > self.ground_truth_times[-1]:
            return self.ground_truth_compass[-1]
        return self.compass_interp(t)

    def get_x_truth(self, t):
        if t < self.ground_truth_times[0]:
            return self.ground_truth_x[0]
        if t > self.ground_truth_times[-1]:
            return self.ground_truth_x[-1]
        return self.x_interp(t)

    def get_y_truth(self, t):
        if t < self.ground_truth_times[0]:
            return self.ground_truth_y[0]
        if t > self.ground_truth_times[-1]:
            return self.ground_truth_y[-1]
        return self.y_interp(t)

    def build_dict(self):
        self.data_queue = []
        self.data_dict = {}
        self.ground_truth_compass = []
        self.ground_truth_x = []
        self.ground_truth_y = []
        self.ground_truth_times = []
        barcode_dict = {}

        for row in self.barcodes_DF.itertuples():
            barcode_dict[row.Barcode] = row.Subject

        for row in self.ground_truth_DF.itertuples():
            time = row.Time
            x = row.X
            y = row.Y
            self.ground_truth_x.append(x)
            self.ground_truth_y.append(y)
            heading = row.Heading
            self.ground_truth_compass.append(heading)
            self.ground_truth_times.append(time)

            self.ground_truth_position.append((time,x,y,heading))

        i = 0
        self.compass_interp = interp.interp1d(self.ground_truth_times, self.ground_truth_compass, assume_sorted = True)
        self.x_interp = interp.interp1d(self.ground_truth_times, self.ground_truth_x, assume_sorted=True)
        self.y_interp = interp.interp1d(self.ground_truth_times, self.ground_truth_y, assume_sorted=True)

        for row in self.odometry_DF.itertuples():
            time = row.Time


            if time > self.ground_truth_times[-1]:
                compass = self.ground_truth_compass[-1]
            else:
                compass = self.compass_interp(time)
            compass += np.random.normal(0, 0.005)

            self.odometry.append((time, row.Velocity, row.AngularVelocity))


        for row in self.measurements_DF.itertuples():
            subject = None
            barcode = row.Barcode
            if barcode not in barcode_dict:
                print("Unrecogonized Barcode: {}. Skipping".format(barcode))
                continue
            else:
                subject = barcode_dict[barcode]

            time = row.Time
            self.measurements.append((time, subject, row.Range, row.Bearing))

        # Merge measurements and odometry
        odom_ptr = 0
        meas_ptr = 0
        self.robot_data = []
        while odom_ptr < len(self.odometry) and meas_ptr < len(self.measurements):

            # If odometry is earlier (or same)
            if self.odometry[odom_ptr][0] <= self.measurements[meas_ptr][0]:
                self.robot_data.append(('odometry', self.odometry[odom_ptr]))
                odom_ptr += 1
            else:
                self.robot_data.append(('measurement', self.measurements[meas_ptr]))
                meas_ptr += 1

        if odom_ptr < len(self.odometry):
            for i in range(odom_ptr, len(self.odometry)):
                self.robot_data.append(('odometry', self.odometry[i]))
                odom_ptr += 1
        elif meas_ptr < len(self.measurements):
            for i in range(meas_ptr, len(self.measurements)):
                self.robot_data.append(('measurement', self.measurements[i]))
                meas_ptr += 1

        for t in sorted(self.data_dict.keys()):
            dict = self.data_dict[t]
            self.data_list.append(dict)

        self.reset()

class Data:
    def __init__(self, directory):
        self.directory = directory
        self.load_all_data()
        print("=== Data Loaded ===")


    def create_df_from_file(self, fname, headers):
        return pd.read_table(fname, names=headers, skiprows=4)

    def load_all_data(self):
        files = os.scandir(self.directory)
        self.num_robots = int((len(list(files))-2)/3)

        self.robot_ground_truth = [None for _ in range(self.num_robots)]
        self.robot_measurements = [None for _ in range(self.num_robots)]
        self.robot_odometry = [None for _ in range(self.num_robots)]

        i = 0

        if self.directory[-1] != '/':
            self.directory += "/"

        for file in os.scandir(self.directory):
            headers = None
            if(file.path.startswith(self.directory + "Barcodes")):
                headers = ["Subject", "Barcode"]
                self.barcodes = self.create_df_from_file(file.path, headers)
            elif(file.path.startswith(self.directory + "Landmark")):
                headers = ["Subject", "X", "Y", "XStd", "YStd"]
                self.landmarks = self.create_df_from_file(file.path, headers)
            elif(file.path.endswith("Groundtruth.dat")):
                headers = ["Time", "X", "Y", "Heading"]
                robot_index = int(file.path[len(self.directory)+5])-1
                self.robot_ground_truth[robot_index] = self.create_df_from_file(file.path, headers)
            elif(file.path.endswith("Odometry.dat")):
                headers = ["Time", "Velocity", "AngularVelocity"]
                robot_index = int(file.path[len(self.directory)+5])-1
                self.robot_odometry[robot_index] = self.create_df_from_file(file.path, headers)
            elif(file.path.endswith("Measurement.dat")):
                headers = ["Time", "Barcode", "Range", "Bearing"]
                robot_index = int(file.path[len(self.directory)+5])-1
                self.robot_measurements[robot_index] = self.create_df_from_file(file.path, headers)
            else:
                raise Exception("File not recognized {}".format(file))
            i += 1

        self.map = Map(self.landmarks)
        self.robots = []
        for i in range(self.num_robots):
            ground_truth = self.robot_ground_truth[i]
            measurements = self.robot_measurements[i]
            odometry = self.robot_odometry[i]
            self.robots.append(copy.deepcopy(Robot(ground_truth, measurements, odometry, self.barcodes)))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Mavenlink Optimization Problem')
    parser.add_argument('source', type=str, help='Directory with input files')
    parser.add_argument('output', type=str, help='Output pickle file path')
    args = parser.parse_args()

    data = Data(args.source)
    t = np.linspace(start=data.robots[0].ground_truth_times[0], stop=data.robots[0].ground_truth_times[-1], num=1000)
    c = [data.robots[0].compass_interp(p) for p in t]

    pickle.dump(data, open(args.output, "wb"))