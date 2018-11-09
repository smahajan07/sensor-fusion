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

  def getPointAhead(self, lat_from, lon_from, distMtrs, azimuth):
    radiusFraction = distMtrs / EARTH_RADIUS
    bearing = self.degToRad(azimuth)
    lat1 = self.degToRad(lat_from)
    lon1 = self.degToRad(lon_from)

    lat2_part1 = np.sin(lat1) * np.cos(radiusFraction)
    lat2_part2 = np.cos(lat1) * np.sin(radiusFraction) * np.cos(bearing)

    lat2 = np.arcsin(lat2_part1 + lat2_part2)

    lon2_part1 = np.sin(bearing) * np.sin(radiusFraction) * np.cos(lat1)
    lon2_part2 = np.cos(radiusFraction) - (np.sin(lat1) * np.sin(lat2))

    lon2 = lon1 + np.arctan2(lon2_part1, lon2_part2)
    lon2 = np.mod((lon2 + 3 * np.pi), (2 * np.pi)) - np.pi

    return self.degToRad(lat2), self.degToRad(lon2)


  def mtrsToGeopoint(self, latAsMtrs, lonAsMtrs):
    lat_tmp, lon_tmp = self.getPointAhead(0.0, 0.0, lonAsMtrs, 90.0)
    lat_ret, lon_ret = self.getPointAhead(lat_tmp, lon_tmp, latAsMtrs, 0.0)

    return lat_ret, lon_ret

