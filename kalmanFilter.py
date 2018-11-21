#!/usr/bin/env python
import numpy as np
from helperMethods import helperMethods

class kalmanFilter(helperMethods):
  '''
  Kalman Filter class fuses the data from GPS and IMU.
  The predict and update functions play the most vital role.
  '''
  def __init__(self, initPos, initVel, posStdDev, accStdDev, currTime):
    helperMethods.__init__(self)
    # set these values from the arguments received
    # current state
    self.X = np.array([[np.float64(initPos)], [np.float64(initVel)]])
    # Identity matrix
    self.I = np.identity(2)
    # Initial guess for covariance
    self.P = np.identity(2)
    # transformation matrix for input data
    self.H = np.identity(2)
    # process (accelerometer) error variance
    self.Q = np.array([[accStdDev * accStdDev, 0], [0, accStdDev * accStdDev]])
    # measurement (GPS) error variance
    self.R = np.array([[posStdDev * posStdDev, 0], [0, posStdDev * posStdDev]])
    # current time
    self.currStateTime = currTime
    # self.A = defined in predict
    # self.B = defined in predict
    # self.u = defined in predict
    # self.z = defined in update

  # main functions
  def predict(self, accThisAxis, timeNow):
    '''
    Predict function perform the initial matrix multiplications.
    Objective is to predict current state and compute P matrix.
    '''
    deltaT = timeNow - self.currStateTime
    self.B = np.array([[0.5 * deltaT * deltaT], [deltaT]])
    self.A = np.array([[1.0, deltaT], [0.0, 1.0]])
    self.u = np.array([[accThisAxis]])

    self.X = np.add(np.matmul(self.A, self.X), np.matmul(self.B, self.u))
    self.P = np.add(np.matmul(np.matmul(self.A, self.P), np.transpose(self.A)), self.Q)
    self.currStateTime = timeNow

  def update(self, pos, velThisAxis, posError, velError):
    '''
    Update function performs the update when the GPS data has been
    received. 
    '''
    self.z = np.array([[pos], [velThisAxis]])
    if(not posError):
      self.R[0, 0] = posError * posError
    else:
      self.R[1, 1] = velError * velError
    y = np.subtract(self.z, self.X)
    s = np.add(self.P, self.R)
    try:
      sInverse = np.linalg.inv(s)
    except np.linalg.LinAlgError:
      print("Matrix is not invertible")
      pass
    else:
      K = np.matmul(self.P, sInverse)
      self.X = np.add(self.X, np.matmul(K, y))
      self.P = np.matmul(np.subtract(self.I, K), self.P)

  def getPredictedPos(self):
    '''
    Returns predicted position in that axis.
    '''

    return self.X[0, 0]

  def getPredictedVel(self):
    '''
    Returns predicted velocity in that axis.
    '''

    return self.X[1, 0]


