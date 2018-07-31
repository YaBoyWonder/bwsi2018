#!/urs/bin/env python

"""
YaBoyWonder LICENSE: Apache-2.0
"""

import time
import math
import numpy as np


class PIDController:

    def __init__(self, kP=0.0, kI=0.0, kD=0.0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.integral = 0.0
        self.past_error = 0.0
        self.prev_time = 0.0

    def output(self, current, setpoint):
        pres_time = time.time()
        dt = pres_time - self.prev_time

        error = setpoint - current

        proportional = self.kP * error
        self.integral += self.kI * (error + self.past_error) * dt
        derivative = self.kD * (error - self.past_error) / dt
        output = proportional + self.integral + derivative

        self.past_error = error
        self.prev_time = pres_time
        return output


class PotentialFieldController:

    def __init__(self, gain=1.0):
        self.gain = gain

        # Linspace creates an array of evenly spaced values
        self.angles = np.linspace(math.radians(-135), math.radians(135), num=1081)
        self.sines = np.sin(self.angles)
        self.cosines = np.cos(self.angles)

    def output(self, scan_data):
        scd = np.asarray(scan_data)

        # Get inverse squares of distances, and combine them to form a new vector
        x_sum = np.sum(np.multiply(self.cosines, np.power(scd, -2)))
        y_sum = np.sum(np.multiply(self.sines, np.power(scd, -2)))

        # Calculate the angle using atan
        angle = math.atan(y_sum / x_sum) * self.gain
        return angle
