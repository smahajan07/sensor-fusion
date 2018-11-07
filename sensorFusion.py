#!/usr/bin/env python
import json
import numpy as np
from kalmanFilter import kalmanFilter
from helperMethods import helperMethods

# setting some constants
ACTUAL_GRAVITY = 9.80665

# open json file with collected data
file_name = 'test.json'
with open(file_name) as data_file:
  data = json.load(data_file)
# read initial data
initialData = data[0]

# set standard deviations
latLonStdDev = 2.0
altStdDev = 3.518522417151836
accEastStdDev = ACTUAL_GRAVITY * 0.033436506994600976
accNorthStdDev = ACTUAL_GRAVITY * 0.05355371135598354
accUpStdDev = ACTUAL_GRAVITY * 0.2088683796078286

# create object of helper class
helperObj = helperMethods()

# create objects of kalman filter
objEast = kalmanFilter(helperObj.lonToMtrs(initialData["gps_lon"]), \
                    initialData["vel_east"], latLonStdDev, \
                    accEastStdDev, initialData["timestamp"])

# for debugging
# print(objEast.X)
# print(objEast.Q)
# print(objEast.R)
# print(objEast.currStateTime)

objNorth = kalmanFilter(helperObj.latToMtrs(initialData["gps_lat"]), \
                    initialData["vel_north"], latLonStdDev, \
                    accNorthStdDev, initialData["timestamp"])

objUp = kalmanFilter(initialData["gps_alt"], \
                    initialData["vel_down"] * -1.0, latLonStdDev, \
                    accUpStdDev, initialData["timestamp"])

# # run loop over new readings
for i in range(1,len(data)):
  pass