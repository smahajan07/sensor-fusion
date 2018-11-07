#!/usr/bin/env python
import numpy as np

EARTH_RADIUS = 6371 * 1000.0 # meters

class helperMethods(object):
  def __init__(self):
    pass

  def latToMtrs(self, latitude):
    distance = self.getDistMtrs(latitude, 0.0, 0.0, 0.0)
    if(distance < 0):
      distance *= -1

    return distance

  def lonToMtrs(self, longitude):
    distance = self.getDistMtrs(0.0, longitude, 0.0, 0.0)
    if(longitude < 0):
      distance *= -1

    return distance

  def degToRad(self, latOrLon):
    
    return (latOrLon * 180.0) / np.pi

  def getDistMtrs(self, lat_from, lon_from, lat_to, lon_to):
    deltaLon = self.degToRad(lon_to - lon_from)
    deltaLat = self.degToRad(lat_to - lat_from)

    a = np.power(np.sin(deltaLat/2.0), 2) + \
          np.cos(self.degToRad(lat_from)) * np.cos(self.degToRad(lat_to)) * \
              np.power(np.sin(deltaLon/2.0), 2)
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1.0 - a))

    return EARTH_RADIUS * c



