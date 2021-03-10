import pandas as pd
import argparse
import os
import copy
import numpy as np
import pickle
import matplotlib.pyplot as plt

class Map:
    def __init__(self, data):
        self.data = data
        self.landmarkDict = self.data.set_index('Subject').to_dict('index')

    def __getitem__(self, subjectID):
        return self.landmarkDict[subjectID]

    def getLandmarkLocation(self, subjectID):
        return self.landmarkDict[subjectID]

class Robot:
    # All arguments are corresponding pandas dataframes
    def __init__(self, groundtruth, measurements, odometry, barcodes):
        self.groundtruthDF = groundtruth
        self.measurementsDF = measurements
        self.odometryDF = odometry
        self.barcodesDF = barcodes

        self.groundtruth = []
        self.measurements = []
        self.odometry = []
        self.barcodes = {}

        self.buildDict()

    # Initialize the frame duration and reset indices
    def initFrames(self, frameDuration):
        self.frameDuration = frameDuration
        self.frameTime = self.odometry[0][0]
        self.groundtruthIndex = 0
        self.measurementsIndex = 0
        self.odometryIndex = 0

    # Get next data frame. Includes one or more odometry commands, measurements, and/or groundtruths
    def getNext(self):
        groundtruth = []
        measurements = []
        odometry = []
        nextFrameTime = self.frameTime + self.frameDuration

        if self.groundtruthIndex < len(self.groundtruth):
            while self.frameTime <= self.groundtruth[self.groundtruthIndex][0] < nextFrameTime:
                groundtruth.append(self.groundtruth[self.groundtruthIndex])
                self.groundtruthIndex += 1

        if self.measurementsIndex < len(self.measurements):
            while self.frameTime <= self.measurements[self.measurementsIndex][0] < nextFrameTime:
                measurements.append(self.measurements[self.measurementsIndex])
                self.measurementsIndex += 1

        if self.odometryIndex < len(self.odometry):
            while self.frameTime <= self.odometry[self.odometryIndex][0] < nextFrameTime:
                odometry.append(self.odometry[self.odometryIndex])
                self.odometryIndex += 1

        self.frameTime = nextFrameTime
        return (self.frameTime, groundtruth, measurements, odometry)

    def hasNext(self):
        return self.odometryIndex < len(self.odometry)

    def buildDict(self):
        for row in self.barcodesDF.itertuples():
            self.barcodes[row.Barcode] = row.Subject

        for row in self.groundtruthDF.itertuples():
            self.groundtruth.append((row.Time, row.X, row.Y, row.Heading))

        for row in self.odometryDF.itertuples():
            self.odometry.append((row.Time, row.Velocity, row.AngularVelocity))

        for row in self.measurementsDF.itertuples():
            subject = None
            barcode = row.Barcode
            if barcode not in self.barcodes:
                print("Unrecogonized Barcode: {}. Skipping".format(barcode))
                continue
            else:
                subject = barcodeDict[barcode]
            self.measurements.append((row.Time, subject, row.Range, row.Bearing))

class Data:
    def __init__(self, directory):
        self.directory = directory
        self.loadAllData()
        print("=== Data Loaded ===")


    def createDfFromFile(self, fname, headers):
        return pd.read_table(fname, names=headers, skiprows=4)

    def loadAllData(self):
        files = os.scandir(self.directory)
        self.numRobots = int((len(list(files)) - 2) / 3)

        self.robotGroundtruth = [None for _ in range(self.numRobots)]
        self.robotMeasurements = [None for _ in range(self.numRobots)]
        self.robotOdometry = [None for _ in range(self.numRobots)]

        i = 0

        if self.directory[-1] != '/':
            self.directory += "/"

        for file in os.scandir(self.directory):
            headers = None
            if(file.path.startswith(self.directory + "Barcodes")):
                headers = ["Subject", "Barcode"]
                self.barcodes = self.createDfFromFile(file.path, headers)
            elif(file.path.startswith(self.directory + "Landmark")):
                headers = ["Subject", "X", "Y", "XStd", "YStd"]
                self.landmarks = self.createDfFromFile(file.path, headers)
            elif(file.path.endswith("Groundtruth.dat")):
                headers = ["Time", "X", "Y", "Heading"]
                robotIndex = int(file.path[len(self.directory)+5])-1
                self.robotGroundtruth[robotIndex] = self.createDfFromFile(file.path, headers)
            elif(file.path.endswith("Odometry.dat")):
                headers = ["Time", "Velocity", "AngularVelocity"]
                robotIndex = int(file.path[len(self.directory)+5])-1
                self.robotOdometry[robotIndex] = self.createDfFromFile(file.path, headers)
            elif(file.path.endswith("Measurement.dat")):
                headers = ["Time", "Barcode", "Range", "Bearing"]
                robotIndex = int(file.path[len(self.directory)+5])-1
                self.robotMeasurements[robotIndex] = self.createDfFromFile(file.path, headers)
            else:
                raise Exception("File not recognized {}".format(file))
            i += 1

        self.map = Map(self.landmarks)
        self.robots = []
        for i in range(self.numRobots):
            groundtruth = self.robotGroundtruth[i]
            measurements = self.robotMeasurements[i]
            odometry = self.robotOdometry[i]
            self.robots.append(copy.deepcopy(Robot(groundtruth, measurements, odometry, self.barcodes)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='MRCLAM Dataset Loader')
    parser.add_argument('directory', type=str, help='Directory with input files')
    args = parser.parse_args()
    data = Data(args.directory)
    pickle.dump(data, open("./datasets/Jar", "wb"))
