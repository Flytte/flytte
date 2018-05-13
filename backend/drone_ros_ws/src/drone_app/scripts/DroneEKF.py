#!/usr/bin/env python
from filterpy.kalman import ExtendedKalmanFilter as EKF

import numpy as np
import time, math

class DroneEKF(EKF):
    """
    This class implemets predict method (f(x,u) function) of an EKF
    """
    def __init__(self):
        # Call the EDK with dimention of the state set to 8,
        # dimention of the observation = 9 and action dimention = 4
        EKF.__init__(self, dim_x=8, dim_z=9, dim_u=4)
        self.x = np.array([0.0, 0.015, 3.5, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.startingTime = 0

    def predict(self, u = np.zeros(4)):
        # If it is the first call of the function, set the starting time
        if self.startingTime == 0:
            self.startingTime = time.time()
        # Calculate the elapsed time from the last call of the function
        dt = time.time() - self.startingTime
        self.startingTime = time.time()
        # Estimate the next state
        self.x = np.array([ self.x[0] + self.x[4]*dt*math.cos(self.x[7]*dt) + self.x[6]*dt*math.sin(self.x[7]*dt),
                            self.x[1] + self.x[5]*dt,
                            self.x[2] - self.x[4]*dt*math.sin(self.x[7]*dt)+ self.x[6]*dt*math.cos(self.x[7]*dt),
                            self.x[3] + self.x[7]*dt,
                            -1*u[0]*math.sin(self.x[3]) - u[1]*math.cos(self.x[3]),
                            -1*u[2],
                            u[0]*math.cos(self.x[3]) - u[1]*math.sin(self.x[3]),
                            u[3]]).astype(np.float64)
        # Jacobian of the next state
        F_t = np.array([[1.0, 0.0, 0.0, 0.0, dt*math.cos(self.x[7]*dt), 0.0, dt*math.sin(self.x[7]*dt), -1.0 * math.pow(dt,2)*self.x[4]*math.sin(dt*self.x[7]) + math.pow(dt,2)*self.x[6]*math.cos(dt*self.x[7])],
                        [0.0, 1.0, 0.0, 0.0, 0.0, dt, 0.0, 0.0],
                        [0.0, 0.0, 1.0, 0.0, -1.0 * dt*math.sin(self.x[7]*dt), 0.0, dt*math.cos(self.x[7]*dt), -1.0 * math.pow(dt,2)*self.x[4]*math.cos(dt*self.x[7]) - math.pow(dt,2)*self.x[6]*math.sin(dt*self.x[7])],
                        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, dt],
                        [0.0, 0.0, 0.0, -1.0*u[0]*math.cos(self.x[3])+u[1]*math.sin(self.x[3]), 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, -1.0*u[0]*math.sin(self.x[3])-u[1]*math.cos(self.x[3]), 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).astype(np.float64)
        # TODO
        std_vel = 0.01
        # Jacobian of the state noise
        V_t = np.array([[0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0],
                        [-1.0 * math.sin(self.x[3]), -1.0* math.cos(self.x[3]), 0.0, 0.0],
                        [0.0, 0.0, -1.0, 0.0],
                        [math.cos(self.x[3]), -1.0*math.sin(self.x[3]), 0.0, 0.0],
                        [0.0, 0.0, 0.0, -1.0]])
        # Calculate the state noise
        M = np.diag([std_vel**2, std_vel**2, std_vel**2,std_vel**2])
        # Compute probability
        self.P = np.dot(F_t, self.P).dot(F_t.T) + np.dot(V_t, M).dot(V_t.T)
