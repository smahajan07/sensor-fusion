#!/usr/bin/env python
import json
import numpy as np
import matplotlib.pyplot as plt
from kalmanFilter import kalmanFilter
from helperMethods import helperMethods

# setting some constants
ACTUAL_GRAVITY = 9.80665

# open json file with collected data
file_name = 'pos_final.json'
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

  pointsToPlotLat = []
  pointsToPlotLon = []

  orgLat = []
  orgLon = []

  # run loop over new readings
  for i in range(1,len(data)): #len(data)
    currData = data[i]

    objEast.predict(currData["abs_east_acc"] * ACTUAL_GRAVITY, currData["timestamp"])
    objNorth.predict(currData["abs_north_acc"] * ACTUAL_GRAVITY, currData["timestamp"])
    objUp.predict(currData["abs_up_acc"] * ACTUAL_GRAVITY, currData["timestamp"])

    if(currData["gps_lat"] != 0.0):

      defPosErr = 0.0

      vEast = currData["vel_east"]
      longitude = objEast.lonToMtrs(currData["gps_lon"])
      objEast.update(longitude, vEast, defPosErr, currData["vel_error"])

      vNorth = currData["vel_north"]
      latitude = objNorth.latToMtrs(currData["gps_lat"])
      objNorth.update(latitude, vNorth, defPosErr, currData["vel_error"])

      vUp = currData["vel_down"] * -1.0
      objUp.update(currData["gps_alt"], vUp, currData["altitude_error"], currData["vel_error"])

      orgLat.append(currData["gps_lat"])
      orgLon.append(currData["gps_lon"])

    predictedLonMtrs = objEast.getPredictedPos()
    predictedLatMtrs = objNorth.getPredictedPos()
    predictedAlt = objUp.getPredictedPos()

    predictedLat, predictedLon = helperObj.mtrsToGeopoint(predictedLatMtrs, predictedLonMtrs)

    predictedVE = objEast.getPredictedVel()
    predictedVN = objNorth.getPredictedVel()

    resultantV = np.sqrt(np.power(predictedVE, 2) + np.power(predictedVN, 2))
    deltaT = currData["timestamp"] - initialData["timestamp"]

    pointsToPlotLat.append(predictedLat)
    pointsToPlotLon.append(predictedLon)

    # print("{} seconds in, Lat: {}, Lon: {}, Alt: {}, Vel(mph): {}".format(
    #         deltaT, predictedLat, predictedLon, predictedAlt, resultantV))

# print(pointsToPlotLat[1])
# print(pointsToPlotLon[1])
plt.subplot(2,1,1)
plt.title('Original')
plt.plot(orgLat, orgLon)

plt.subplot(2,1,2)
plt.title('Fused')
plt.plot(pointsToPlotLat, pointsToPlotLon)

plt.show()
