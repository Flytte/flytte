#!/usr/bin/env python
from filterpy.kalman import ExtendedKalmanFilter as EKF

import numpy as np
import time, math

class DroneObserver():
    """
    This class implemets observation predict method (h(x)) of an EKF
    """
    def __init__(self, camera_matrix, drone_width, drone_height):

        self.camera_matrix = camera_matrix
        self.drone_width = drone_width
        self.drone_height = drone_height

        self.start_state = np.array([0.0, 0.015, 3.5, 0.0])

    def predict(self, state):
        # Predict the observation from the state
        return np.array([(self.camera_matrix[0][0] * ((2 * state[0] - self.drone_width) / (2 * state[2])) + self.camera_matrix[0][1]) * 100 / 640,
                        (self.camera_matrix[1][0] * ((2 * state[1] - self.drone_height) / (2 * state[2])) + self.camera_matrix[1][1]) * 100 / 480,
                        (self.camera_matrix[0][0] * self.drone_width / state[2]) * 100 / 640,
                        (self.camera_matrix[1][0] * self.drone_height / state[2]) * 100 / 480,
                        state[3],
                        -1.0 * math.cos(self.start_state[3]) * (state[0] - self.start_state[0]) + math.cos(self.start_state[3]) * (state[2] - self.start_state[2]),
                        -1.0 * math.cos(self.start_state[3]) * (state[0] - self.start_state[0]) - math.sin(self.start_state[3]) * (state[2] - self.start_state[2]),
                        -1.0 * (state[1] - self.start_state[1]),
                        state[3] - self.start_state[3]])

    def jacobi_at(self, state):
        # Calculate jacobian at the position of a state
        return np.array([[self.camera_matrix[0][0]*100/(640 * state[2]), 0.0, -50.0 * self.camera_matrix[0][0]*(2*state[0] - self.drone_width)/(640*math.pow(state[2],2)), 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, self.camera_matrix[1][0]*100/(480 * state[2]), -50.0 * self.camera_matrix[1][0]*(2*state[0] - self.drone_height)/(480*math.pow(state[2],2)), 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, -100.0 * self.camera_matrix[0][0] * self.drone_width/(math.pow(state[2],2) * 640), 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, -100.0 * self.camera_matrix[1][0] * self.drone_height/(math.pow(state[2],2) * 480), 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                        [-1.0*math.sin(self.start_state[3]), 0.0, math.cos(self.start_state[3]), 0.0, 0.0, 0.0, 0.0, 0.0],
                        [-1.0*math.cos(self.start_state[3]), 0.0, -1.0*math.sin(self.start_state[3]), 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]])
