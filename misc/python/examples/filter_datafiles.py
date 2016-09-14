import sys, os
sys.path.insert(0, os.path.abspath('..'))

import numpy as np
import math
import scipy.linalg
from pypsmove.transformations import quaternion_matrix, quaternion_conjugate, quaternion_multiply
from filterpy.common import Q_discrete_white_noise, dot3
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman.unscented_transform import unscented_transform
from filterpy.kalman.UKF import UnscentedKalmanFilter as UKF

# Load the test data
trainingdata = np.genfromtxt(os.path.join('..', '..', 'test_data', 'stationary.csv'), delimiter=',')  # Recorded while controller was stationary upright
testdata = np.genfromtxt(os.path.join('..', '..', 'test_data', 'movement.csv'), delimiter=',')  # Recorded while controller was moving around.
mean_dt = np.nanmean(np.diff(trainingdata[:, -1]))

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

# DESIGN STATE VECTOR
#Define column indices of each state variable in state matrix.
st_pos_x, st_pos_y, st_pos_z = 0, 1, 2  # Position in 3D space in cm
st_lvel_x, st_lvel_y, st_lvel_z = 3, 4, 5  # Linear velocity in cm/s
st_lacc_x, st_lacc_y, st_lacc_z = 6, 7, 8  # Linear acceleration in cm/s^2
st_e_w, st_e_x, st_e_y, st_e_z = 9, 10, 11, 12  #Unit quaternion
st_avel_x, st_avel_y, st_avel_z = 13, 14, 15  # Angular velocity in rad/s
NSTATEDIM = 16
st_lin_list = (st_pos_x, st_pos_y, st_pos_z, st_lvel_x, st_lvel_y, st_lvel_z, st_lacc_x, st_lacc_y, st_lacc_z, st_avel_x, st_avel_y, st_avel_z)

# DESIGN STATE COVARIANCE VECTOR
p_pos_x, p_pos_y, p_pos_z = 0, 1, 2  # Position in 3D space in cm
p_lvel_x, p_lvel_y, p_lvel_z = 3, 4, 5  # Linear velocity in cm/s
p_lacc_x, p_lacc_y, p_lacc_z = 6, 7, 8  # Linear acceleration in cm/s^2
p_e_x, p_e_y, p_e_z = 9, 10, 11  #Angle-Axis representation
p_avel_x, p_avel_y, p_avel_z = 12, 13, 14  # Angular velocity in rad/s
NPDIM = 15
p_lin_list = (p_pos_x, p_pos_y, p_pos_z, p_lvel_x, p_lvel_y, p_lvel_z, p_lacc_x, p_lacc_y, p_lacc_z, p_avel_x, p_avel_y, p_avel_z)

# DESIGN OBSERVATION VECTOR
#Define the column indices of each observation variable in the observation matrix
obs_accel_x, obs_accel_y, obs_accel_z = 0, 1, 2  # In g-units
obs_gyro_x, obs_gyro_y, obs_gyro_z = 3, 4, 5  # In rad/s
obs_mag_x, obs_mag_y, obs_mag_z = 6, 7, 8  # In units of the strength of the local magnetic field.
obs_track_x, obs_track_y, obs_track_z = 9, 10, 11  # in cm
NOBSDIM = 12

# Some constants
GRAVITY = 9.806  # Depends on latitude.
grav_world = np.asarray([0, 0, -GRAVITY])  # -1 because accelerometer observations are returned normalized to gravity (I think)
LOCAL_MAG_FIELD = calibration['Magnetometer']['Identity']
mag_world = np.asarray([0, LOCAL_MAG_FIELD['X'], LOCAL_MAG_FIELD['Y'], LOCAL_MAG_FIELD['Z']])
QSigmaSq = 0.1
RScale = 5.0  # Only variable that appears to make any difference.
alpha= 0.1
beta= 2.0  # 2 is optimal for gaussian variables
kappa= 3 - NPDIM


# To transform vectors from controller frame to navigation frame...
#DCM = quaternion_matrix(q)
# To transform from navigation frame to controller frame...
#DCM.T = quaternion_matrix(q).T = quaternion_matrix(quaternion_conjugate(q))

def quat_from_axang(axang, dt=1):
    """
    Convert an axis-angle (3D) representation of orientation into a quaternion (4D)
    :param axang:
    :param dt:
    :return:
    """
    s = np.linalg.norm(axang)
    rot_angle = s * dt
    if rot_angle > 0:
        rot_axis = axang / s
        result = np.hstack((np.cos(rot_angle/2), rot_axis*np.sin(rot_angle/2)))
    else:
        result = np.asarray([1., 0., 0., 0.])
    return result / np.linalg.norm(result)

def angax_from_quat(quat):
    if quat[0] > 1:
        #if w > 1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
        quat /= np.linalg.norm(quat)
    angle = 2 * math.acos(quat[0])
    s = math.sqrt(1 - quat[0]*quat[0])
    if (s < 0.001):
        axis = [1., 0., 0.]
    else:
        axis = quat[1:] / s
    return angle * (axis / np.linalg.norm(axis))

def shrink_state(state):
    # Handle the linear parts
    out = np.zeros(NPDIM)
    out[(p_lin_list),] = state[(st_lin_list),]
    # Handle the quaternion part
    quat = state[(st_e_w, st_e_x, st_e_y, st_e_z),]
    angax = angax_from_quat(quat)
    out[(p_e_x, p_e_y, p_e_z),] = angax
    return out

def pcol_to_state(pcol):
    out = np.zeros(NSTATEDIM)
    out[(st_lin_list),] = pcol[(p_lin_list),]
    out[(st_e_w, st_e_x, st_e_y, st_e_z),] = quat_from_axang(pcol[(p_e_x, p_e_y, p_e_z),])
    return out

def state_diff(x1, x2):
    """
    Calculate the difference between two state vectors (e.g., state and sigmapoint)
    Used in update calculation of state covariance (P): state_sp - state_mean
    Used in calculation of state-observation cross-covariance: [state_sp - state_mean][obs_sp - obs_mean]
    (Only used for the first part. Obs difference may use its own function)
    :param x1:
    :param x2:
    :return:
    """
    if len(x1) != len(x2):
        raise ValueError("Vectors must be the same length.")
    result = np.nan * np.ones(len(x1))

    # For linear measurements, this is pretty easy, just subtract
    result[st_lin_list,] = np.subtract(x1[st_lin_list,], x2[st_lin_list,])

    # For orientation it is more complicated.
    x2q_conj = quaternion_conjugate(x2[(st_e_w, st_e_x, st_e_y, st_e_z),])
    delta_q = quaternion_multiply(x1[(st_e_w, st_e_x, st_e_y, st_e_z),], x2q_conj)
    result[(st_e_w, st_e_x, st_e_y, st_e_z),] = delta_q / np.linalg.norm(delta_q)
    return result

def state_diff_p(x1, x2):
    return shrink_state(state_diff(x1, x2))

def state_covcol_diff(state, pcol):
    return state_diff(state, pcol_to_state(pcol))

def x_mean_fn(sigmas, Wm):
    # Calculate average state x from sigma points.
    x = np.nan*np.ones(sigmas.shape[1])
    # For position, velocity, acceleration, and angular velocity this is simple
    x[st_lin_list,] = np.dot(Wm, sigmas[:, st_lin_list])
    # For orientation, we need the weighted average quaternion.
    #http://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
    Q = sigmas[:, (st_e_w, st_e_x, st_e_y, st_e_z)].T * Wm
    eig_val, eig_vec = np.linalg.eig(np.dot(Q, Q.T))
    q_out = eig_vec[:, np.argmax(eig_val)]
    q_out /= np.linalg.norm(q_out)
    x[(st_e_w, st_e_x, st_e_y, st_e_z),] = q_out
    return x

def process_fun(state, dt):
    """
    Describes how state at time k evolves to a state at time k+dt
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
    delta_q = quat_from_axang(state[(st_avel_x, st_avel_y, st_avel_z),], dt)
    q_out = quaternion_multiply(state[(st_e_w, st_e_x, st_e_y, st_e_z),], delta_q)  #Maybe to do: *qnoise in the middle
    output[(st_e_w, st_e_x, st_e_y, st_e_z),] = q_out / np.linalg.norm(q_out)
    output[(st_avel_x, st_avel_y, st_avel_z),] = state[(st_avel_x, st_avel_y, st_avel_z),]

    return output

def observation_fun(state):
    output = np.nan * np.ones((NOBSDIM,))

    q = state[(st_e_w, st_e_x, st_e_y, st_e_z),]
    # world_to_cntrl = quaternion_matrix(quaternion_conjugate(q))
    accel_world_g = np.hstack((0, state[(st_lacc_x, st_lacc_y, st_lacc_z),] + grav_world)) / GRAVITY
    output[(obs_accel_x, obs_accel_y, obs_accel_z),] = quaternion_multiply(q, quaternion_multiply(accel_world_g, quaternion_conjugate(q)))[1:]
                                                        # + accel_bias + accel_noise
    output[(obs_gyro_x, obs_gyro_y, obs_gyro_z),] = state[(st_avel_x, st_avel_y, st_avel_z),]  # + gyro_bias + gyro_noise
    output[(obs_mag_x, obs_mag_y, obs_mag_z),] = quaternion_multiply(q, quaternion_multiply(mag_world,
                                                                                                  quaternion_conjugate(
                                                                                                      q)))[1:]
                                                # + mag_bias + mag_noise
    output[(obs_track_x, obs_track_y, obs_track_z),] = state[(st_pos_x, st_pos_y, st_pos_z),]
    return output

# Initial state
x_init = np.zeros((NSTATEDIM,))  # Assume all velocities and accelerations are initially 0
x_init[(st_pos_x, st_pos_y, st_pos_z),] = testdata[0, (obs_track_x, obs_track_y, obs_track_z)]  # First optical pos
x_init[(st_e_w, st_e_x, st_e_y, st_e_z),] = testdata[0, 12:16]  # First orientation. I could calc from accel/gyro, but it's already done for me...

# Initial state covariance
P_init = 0.1 * np.eye(NPDIM)  #TODO: calculate from trainingdata.

# Process noise covariance
Q_init = QSigmaSq*np.eye(NPDIM)
wn = Q_discrete_white_noise(3, mean_dt, var=QSigmaSq)  # 3rd order for position
Q_init[(p_pos_x, p_lvel_x, p_lacc_x),][:, (p_pos_x, p_lvel_x, p_lacc_x)] = wn
Q_init[(p_pos_y, p_lvel_y, p_lacc_y),][:,(p_pos_y, p_lvel_y, p_lacc_y)] = wn
Q_init[(p_pos_z, p_lvel_z, p_lacc_z),][:,(p_pos_z, p_lvel_z, p_lacc_z)] = wn
wn = Q_discrete_white_noise(2, mean_dt, var=QSigmaSq)  # Only 2nd order for orientation
Q_init[(p_e_x, p_avel_x),][:, (p_e_x, p_avel_x)] = wn
Q_init[(p_e_y, p_avel_y),][:, (p_e_y, p_avel_y)] = wn
Q_init[(p_e_z, p_avel_z),][:, (p_e_z, p_avel_z)] = wn

# Measurement noise covariance - calculate from trainingdata
# Every one of our measurements should be independent, so no off-diagonals.
R_init = RScale * np.diag(np.cov(trainingdata[:, :12], rowvar=False)) * np.eye(NOBSDIM)

#Subclass MerweScaledSigmaPoints and override sigma_points to get rid of size assertion
class MySigmaPoints(MerweScaledSigmaPoints):
    def sigma_points(self, x, P):
        n = self.n
        lambda_ = self.alpha**2 * (n + self.kappa) - n
        U = self.sqrt((lambda_ + n)*P)
        sigmas = np.zeros((2*n+1, x.size))
        sigmas[0] = x
        for k in range(n):
            sigmas[k+1]   = self.subtract(x, -U[k])
            sigmas[n+k+1] = self.subtract(x, U[k])
        return sigmas

class MyUKF(UKF):
    @property
    def P(self):
        return self._P
    @P.setter
    def P(self, val):
        self._P = val
        self._num_sigmas = 2 * self._P.shape[0] + 1
        self.sigmas_f = np.zeros((self._num_sigmas, self._dim_x))
        self.sigmas_h = np.zeros((self._num_sigmas, self._dim_z))

    def update(self, z, R=None, UT=None, hx_args=()):
        if z is None:
            return
        if not isinstance(hx_args, tuple):
            hx_args = (hx_args,)
        if UT is None:
            UT = unscented_transform
        if R is None:
            R = self.R
        elif np.isscalar(R):
            R = np.eye(self._dim_z) * R

        for i in range(self._num_sigmas):
            self.sigmas_h[i] = self.hx(self.sigmas_f[i], *hx_args)

        # mean and covariance of prediction passed through unscented transform
        zp, Pz = UT(self.sigmas_h, self.Wm, self.Wc, R, self.z_mean, self.residual_z)

        # compute cross variance of the state and the measurements
        Pxz = np.zeros((self.P.shape[0], self._dim_z))  # Changed
        for i in range(self._num_sigmas):
            dx = self.residual_x(self.sigmas_f[i], self.x)
            dz =  self.residual_z(self.sigmas_h[i], zp)
            Pxz += self.Wc[i] * np.outer(dx, dz)

        K = np.dot(Pxz, scipy.linalg.inv(Pz))   # Kalman gain
        y = self.residual_z(z, zp)   #residual
        self.x = state_covcol_diff(self.x, -np.dot(K, y))  # Changed
        self._P = self.P - dot3(K, Pz, K.T)

def myUT(sigmas, Wm, Wc, noise_cov=None,
                        mean_fn=None, residual_fn=None):
    kmax, n = sigmas.shape
    if mean_fn is None:
        # new mean is just the sum of the sigmas * weight
        x = np.dot(Wm, sigmas)    # dot = \Sigma^n_1 (W[k]*Xi[k])
    else:
        x = mean_fn(sigmas, Wm)
    if residual_fn is None:
        y = sigmas - x[np.newaxis,:]
        P = y.T.dot(np.diag(Wc)).dot(y)
    else:
        if noise_cov is not None:
            p_shape = noise_cov.shape
        else:
            y0 = residual_fn(sigmas[0], x)
            p_shape = (len(y0), len(y0))
        P = np.zeros(p_shape)
        for k in range(kmax):
            y = residual_fn(sigmas[k], x)
            P += Wc[k] * np.outer(y, y)
    if noise_cov is not None:
        P += noise_cov
    return (x, P)

# subtract wants non-sharnk version
sigpoints = MySigmaPoints(n=NPDIM, alpha=alpha, beta=beta, kappa=kappa, subtract=state_covcol_diff)

# residual_x wants shrank version
myukf = MyUKF(dim_x=NSTATEDIM, dim_z=NOBSDIM, dt=mean_dt,
            hx=observation_fun, fx=process_fun, points=sigpoints,
            x_mean_fn=x_mean_fn, residual_x=state_diff_p)
myukf.x = x_init
myukf.P = P_init
myukf.Q = Q_init
myukf.R = R_init

# Run the filter
observations = testdata[np.logical_not(np.any(np.isnan(testdata), axis=1))]
last_time = observations[0, -1] - mean_dt
uxs = []
for z in observations:
    dt = z[-1] - last_time
    myukf.predict(dt=dt, UT=myUT)
    myukf.update(z[:12], UT=None)
    uxs.append(myukf.x.copy())
    last_time = z[-1]
uxs = np.array(uxs)

# Plot the result
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

#TODO: Plot 3D end-point of quaternion on unit sphere. Show roll with different colour.
ax2 = fig.add_subplot(212)
psm_q, = ax2.plot(observations[:, -1], observations[:, 12], 'b', label="PSMoveService q0")
ukf_q, = ax2.plot(observations[:, -1], uxs[:, st_e_w], 'r', label="Predicted q0")
ax2.legend(handles=[psm_q, ukf_q])
plt.show()

#Additional TODOs:
#-handle partial observations (only IMU, only optical tracker)
#-compensate for sensor offset, lever arm
#-account for delays in optical measurements