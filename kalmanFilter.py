#!/usr/bin/env python
import numpy as np
from helperMethods import helperMethods

class kalmanFilter(helperMethods):
  def __init__(self, initPos, initVel, posStdDev, accStdDev, currTime):
    helperMethods.__init__(self)
    # set these values from the arguments received
    # current state
    self.X = np.array([[np.float64(initPos)], [np.float64(initVel)]])
    self.I = np.identity(2);
    self.P = np.identity(2); 
    self.H = np.identity(2);
    self.Q = np.array([[accStdDev * accStdDev, 0], [0, accStdDev * accStdDev]])
    self.R = np.array([[posStdDev * posStdDev, 0], [0, posStdDev * posStdDev]])
    self.currStateTime = currTime
    # self.A = 
    # self.B = defined in predict
    # self.u = 
    # self.z = 

  # main functions
  def predict(self, accThisAxis, timeNow):
    deltaT = timeNow - self.currStateTime
    self.B = np.array([[0.5 * deltaT * deltaT], [deltaT]])
    self.A = np.array([[1.0, deltaT], [0.0, 1.0]])
    self.u = np.array([[accThisAxis]])

    self.X = np.add(np.matmul(self.A, self.X), np.matmul(self.B, self.u))
    self.P = np.add(np.matmul(np.matmul(self.A, self.P), np.transpose(self.A)), self.Q)
    self.currStateTime = timeNow

  def updates(self):
    pass

  def fusion(self):
    pass

  def getPredictedValues(self):
    pass


