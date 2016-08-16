import numpy as np
from transformations import quaternion_matrix, quaternion_conjugate, quaternion_multiply
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman.UKF import UnscentedKalmanFilter as UKF

# Load the test data
trainingdata = np.genfromtxt('stationary.csv', delimiter=',')  # Recorded while controller was stationary upright
testdata = np.genfromtxt('movement.csv', delimiter=',')  # Recorded while controller was moving around.

# These were the calibration values taken from PSMoveService when stationary.csv and movement.csv were collected.
calibration = {
    'Accel': {
        'X': {'k': 0.000228754434, 'b': -0.0156696439},
        'Y': {'k': 0.000230096644, 'b': 0.0416475534},
        'Z': {'k': 0.00023118715, 'b': -0.0112125874},
        'NoiseRadius': 0.00866362546
    },
    'Gyro': {
        'X': {'k': 0.00152541522, 'b': 0.0},
        'Y': {'k': 0.00177830202, 'b': 0.0},
        'Z': {'k': 0.00164234079, 'b': 0.0},
        'Variance': 0.000382817292,
        'Drift': 0.0281366557
    },
    'Magnetometer': {
        'Center': {'X': -56, 'Y': 49, 'Z': -153},
        'BasisX': {'X': 1, 'Y': 0, 'Z': 0},
        'BasisY': {'X': 0, 'Y': 1, 'Z': 0},
        'BasisZ': {'X': 0, 'Y': 0, 'Z': 1},
        'Extents': {'X': 138, 'Y': 143, 'Z': 132},
        'Identity': {'X': 0.737549126, 'Y': 0.675293505, 'Z': 1},
        'Error': 54.9325409
    }
}

# DESIGN STATE VARIABLE
#Define column indices of each state variable in state matrix.
st_pos_x, st_pos_y, st_pos_z = 0, 1, 2  # Position in 3D space in cm
st_lvel_x, st_lvel_y, st_lvel_z = 3, 4, 5  # Linear velocity in cm/s
st_lacc_x, st_lacc_y, st_lacc_z = 6, 7, 8  # Linear acceleration in cm/s/s
st_e_w, st_e_x, st_e_y, st_e_z = 9, 10, 11, 12  #Unit quaternion
st_avel_x, st_avel_y, st_avel_z = 13, 14, 15  # Angular velocity in rad/s
NSTATEDIM = 16

def quat_from_ang_vel(ang_vel, dt):
    rot_angle = np.linalg.norm(ang_vel) * dt
    if rot_angle > 0:
        rot_axis = ang_vel / np.linalg.norm(ang_vel)
        result = np.hstack((np.cos(rot_angle/2), rot_axis*np.sin(rot_angle/2)))
    else:
        result = np.asarray([1.0, 0., 0., 0.])
    return result / np.linalg.norm(result)

def process_fun(state, dt):
    """
    Describes how state at time k-1 evolves to a state at time k
    """
    output = np.nan * np.ones((NSTATEDIM,))

    # Position and linear updates
    output[(st_pos_x, st_pos_y, st_pos_z),] = state[(st_pos_x, st_pos_y, st_pos_z),]\
                                           + dt * state[(st_lvel_x, st_lvel_y, st_lvel_z),]\
                                           + dt * dt * state[(st_lacc_x, st_lacc_y, st_lacc_z),] / 2
    output[(st_lvel_x, st_lvel_y, st_lvel_z),] = state[(st_lvel_x, st_lvel_y, st_lvel_z),]\
                                              + dt*state[(st_lacc_x, st_lacc_y, st_lacc_z),]
    output[(st_lacc_x, st_lacc_y, st_lacc_z),] = state[(st_lacc_x, st_lacc_y, st_lacc_z),]

    # Orientation update
    delta_q = quat_from_ang_vel(state[(st_avel_x, st_avel_y, st_avel_z),], dt)
    q_out = quaternion_multiply(state[(st_e_w, st_e_x, st_e_y, st_e_z),], delta_q)
    output[(st_e_w, st_e_x, st_e_y, st_e_z),] = q_out / np.linalg.norm(q_out)
    output[(st_avel_x, st_avel_y, st_avel_z),] = state[(st_avel_x, st_avel_y, st_avel_z),]

    return output

#Define the column indices of each observation variable in the observation matrix
obs_accel_x, obs_accel_y, obs_accel_z = 0, 1, 2
obs_gyro_x, obs_gyro_y, obs_gyro_z = 3, 4, 5
obs_mag_x, obs_mag_y, obs_mag_z = 6, 7, 8
obs_track_x, obs_track_y, obs_track_z = 9, 10, 11
NOBSDIM = 12

# To transform vectors from controller frame to navigation frame...
#DCM = quaternion_matrix(q)
# To transform from navigation frame to controller frame...
#DCM.T = quaternion_matrix(q).T = quaternion_matrix(quaternion_conjugate(q))

GRAVITY = np.asarray([0, 0, -1])  # -1 because accelerometer observations are returned normalized to gravity (I think)
LOCAL_MAG_FIELD = calibration['Magnetometer']['Identity']
mag_world = np.asarray([0, LOCAL_MAG_FIELD['X'], LOCAL_MAG_FIELD['Y'], LOCAL_MAG_FIELD['Z']])

def observation_fun(state):
    output = np.nan * np.ones((NOBSDIM,))

    q = state[(st_e_w, st_e_x, st_e_y, st_e_z),]
    # world_to_cntrl = quaternion_matrix(quaternion_conjugate(q))
    accel_world = np.hstack((0, state[(st_lacc_x, st_lacc_y, st_lacc_z),] + GRAVITY))
    output[(obs_accel_x, obs_accel_y, obs_accel_z),] = quaternion_multiply(q, quaternion_multiply(accel_world, quaternion_conjugate(q)))[1:]
                                                        # + accel_bias + accel_noise
    output[(obs_gyro_x, obs_gyro_y, obs_gyro_z),] = state[(st_avel_x, st_avel_y, st_avel_z),]  # + gyro_bias + gyro_noise
    output[(obs_mag_x, obs_mag_y, obs_mag_z),] = quaternion_multiply(q, quaternion_multiply(mag_world,
                                                                                                  quaternion_conjugate(
                                                                                                      q)))[1:]
                                                # + mag_bias + mag_noise
    output[(obs_track_x, obs_track_y, obs_track_z),] = state[(st_pos_x, st_pos_y, st_pos_z),]
    return output

mean_dt = np.nanmean(np.diff(trainingdata[:, -1]))

# Initial state
x_init = np.zeros((NSTATEDIM,))
x_init[(st_pos_x, st_pos_y, st_pos_z),] = testdata[0, (obs_track_x, obs_track_y, obs_track_z)]
x_init[(st_e_w, st_e_x, st_e_y, st_e_z),] = testdata[0, 12:16]  # Cheating here. I could calculate orientation from accel/gyro but it was already done by PSMoveService.

# Initial state covariance
P_init = 10.0 * np.eye(NSTATEDIM)  #TODO: Something more realistic.

# Process noise covariance
Q_init = 0.1*np.eye(NSTATEDIM)
wn = Q_discrete_white_noise(3, mean_dt, var=0.1)
Q_init[(st_pos_x, st_lvel_x, st_lacc_x),][:, (st_pos_x, st_lvel_x, st_lacc_x)] = wn
Q_init[(st_pos_y, st_lvel_y, st_lacc_y),][:,(st_pos_y, st_lvel_y, st_lacc_y)] = wn
Q_init[(st_pos_z, st_lvel_z, st_lacc_z),][:,(st_pos_z, st_lvel_z, st_lacc_z)] = wn
#TODO: Q_init for orientation and angular velocity

# Measurement noise covariance
R_init = 5.0 * np.diag(np.cov(trainingdata[:, :12], rowvar=False)) * np.eye(NOBSDIM)

points = MerweScaledSigmaPoints(n=NSTATEDIM, alpha=.1, beta=2., kappa=-1)

def x_mean_fn(sigmas, Wm):
    # Calculate average state from sigma points.
    # For position, velocity, acceleration, and angular velocity this is simple
    x = np.dot(Wm, sigmas)
    # For orientation, we need the weighted average quaternion.
    #http://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
    Q = sigmas[:, (st_e_w, st_e_x, st_e_y, st_e_z)].T * Wm
    eig_val, eig_vec = np.linalg.eig(np.dot(Q, Q.T))
    q_out = eig_vec[:, np.argmax(eig_val)]
    q_out /= np.linalg.norm(q_out)
    # x[(st_e_w, st_e_x, st_e_y, st_e_z),] = q_out  #TODO: Uncomment, but this causes crash
    # In fact, simply normalizing the quaternion in the state vector causes a crash
    return x

def residual_x(sigma_point, state_x):
    #Calculate difference between predicted sigma point and the mean state
    # For position, velocity, acceleration, and angular velocity this should be simple
    result = np.subtract(sigma_point, state_x)
    # For orientation this is more complicated (what is a quaternion difference)?
    qbar_conj = quaternion_conjugate(state_x[(st_e_w, st_e_x, st_e_y, st_e_z),])
    delta_q = quaternion_multiply(sigma_point[(st_e_w, st_e_x, st_e_y, st_e_z),], qbar_conj)
    # result[(st_e_w, st_e_x, st_e_y, st_e_z),] = delta_q / np.linalg.norm(delta_q)  #TODO: Uncomment, but this causes crash
    return result

myUKF = UKF(dim_x=NSTATEDIM, dim_z=NOBSDIM, dt=mean_dt,
            hx=observation_fun, fx=process_fun, points=points,
            x_mean_fn=x_mean_fn, residual_x=residual_x)
myUKF.x = x_init
myUKF.P = P_init
myUKF.Q = Q_init
myUKF.R = R_init

observations = testdata[np.logical_not(np.any(np.isnan(testdata), axis=1))]
last_time = observations[0, -1] - mean_dt
uxs = []
for z in observations:
    dt = z[-1] - last_time
    myUKF.predict(dt=dt)
    myUKF.update(z[:12])
    uxs.append(myUKF.x.copy())
    last_time = z[-1]
uxs = np.array(uxs)

track_pos = observations[:, (obs_track_x, obs_track_y, obs_track_z)]
filt_pos = uxs[:, (st_pos_x, st_pos_y, st_pos_z)]
diffs = np.linalg.norm(filt_pos-track_pos, axis=1)
print("Difference mean: {}, std: {}".format(np.mean(diffs), np.std(diffs)))

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax1 = fig.add_subplot(211, projection='3d')
pl_tracker, = ax1.plot(track_pos[:, 0], track_pos[:, 1], track_pos[:, 2], 'b', label="Original")
pl_filter, = ax1.plot(filt_pos[:, 0], filt_pos[:, 1], filt_pos[:, 2], 'r', label="UKF")
ax1.legend(handles=[pl_tracker, pl_filter])

ax2 = fig.add_subplot(212)
psm_q, = ax2.plot(observations[:, -1], observations[:, 12], 'b', label="PSMoveService q0")
ukf_q, = ax2.plot(observations[:, -1], uxs[:, st_e_w], 'r', label="Predicted q0")
ax2.legend(handles=[psm_q, ukf_q])
plt.show()

#Additional TODOs:
#-handle partial observations (only IMU, only optical tracker)
#-compensate for sensor offset, lever arm
#-account for delays in optical measurements