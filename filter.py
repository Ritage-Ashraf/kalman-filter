#!/usr/bin/env python3
from filterpy.kalman import KalmanFilter
from pykalman import KalmanFilter
import numpy as np
import rospy
class KalmanFilter1D:
    def __init__(self, process_variance, measurement_variance, initial_estimate, initial_error_estimate):
        self.process_variance = process_variance          # Process variance (Q)
        self.measurement_variance = measurement_variance  # Measurement variance (R)
        self.estimate = initial_estimate                  # Initial state estimate
        self.error_estimate = initial_error_estimate      # Initial estimate of error

    def predict(self):
        # In a 1D Kalman filter, the state prediction remains the same (no control input)
        self.error_estimate += self.process_variance  # Update the error estimate with process variance

    def update(self, measurement):
        # Compute Kalman Gain
        kalman_gain = self.error_estimate / (self.error_estimate + self.measurement_variance)

        # Update the estimate with the latest measurement
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)

        # Update the error estimate
        self.error_estimate = (1 - kalman_gain) * self.error_estimate

    def get_estimate(self):
        return self.estimate


# Example usage:
# Set variances for process and measurement (you can tune these)
process_variance = 0.01  # Q
measurement_variance = 0.1  # R

# Initial estimates
initial_yaw_angle = 0.0
initial_error_estimate = 1.0

# Create Kalman filter object
kf = KalmanFilter1D(process_variance, measurement_variance, initial_yaw_angle, initial_error_estimate)

# Simulating a loop where you receive new yaw angles (e.g., from IMU)
yaw_measurements = [0.05, 0.1, 0.12, 0.15, 0.18]  # Example yaw angle measurements

for measurement in yaw_measurements:
    kf.predict()           # Prediction step
    kf.update(measurement) # Update step with the new measurement
    print(f"Filtered YAW Angle: {kf.get_estimate()}")