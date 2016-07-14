import numpy as np
import csv
from pykalman.sqrt import CholeskyKalmanFilter as KalmanFilter

trainingdata = np.empty((0, 12))
with open('stationary.csv') as csvfile:
    csv_reader = csv.reader(csvfile)
    for row in csv_reader:
        trainingdata = np.concatenate((trainingdata, np.atleast_2d(np.asarray([float(x) for x in row]))), axis=0)

testdata = np.empty((0, 12))
with open('movement.csv') as csvfile:
    csv_reader = csv.reader(csvfile)
    for row in csv_reader:
        testdata = np.concatenate((testdata, np.atleast_2d(np.asarray([float(x) for x in row]))), axis=0)


# Prepare Kalman filter
GRAVITY = np.asarray([0, 0, -9.80665])
LOCAL_MAG_FIELD = None

def transition_function(state, noise):
    position = state['position'] + dt*state['position'] + dt*dt*state['acceleration']/2
    velocity = state['velocity'] + dt*state['acceleration']
    acceleration = state['acceleration']
    orientation = state['orientation'] + dt*state['angular_velocity'] + dt*dt*state['angular_acceleration']/2
    angular_velocity = dt*state['angular_acceleration']
    angular_acceleration = state['angular_acceleration']

    return {'position': position, 'velocity': velocity, 'acceleration': acceleration,
            'orientation': orientation, 'angular_velocity': angular_velocity, 'angular_acceleration': angular_acceleration}

def observation_function(state, noise):
    position = state['position']
    magnetometer = state['orientation'] * LOCAL_MAG_FIELD
    accelerometer = state['acceleration'] + state['orientation'] * GRAVITY
    gyroscope = state['angular_velocity'] # + gyro_bias
    return {'position': position, 'magnetometer': magnetometer, 'accelerometer': accelerometer, 'gyroscope': gyroscope}

transition_covariance = np.eye(19)
observation_covariance = np.eye(12)
initial_state_mean = np.zeros((1, 19))
initial_state_covariance = np.eye(19)