import numpy as np
from scipy.linalg import cholesky, qr, lstsq

"""
The Q_discrete_white_noise function is taken from filterpy by Roger R Labbe Jr. (MIT Licensed)
"""
def Q_discrete_white_noise(dim, dt=1., var=1.):
    """ Returns the Q matrix for the Discrete Constant White Noise
    Model. dim may be either 2 or 3, dt is the time step, and sigma is the
    variance in the noise.

    Q is computed as the G * G^T * variance, where G is the process noise per
    time step. In other words, G = [[.5dt^2][dt]]^T for the constant velocity
    model.

    Parameters
    -----------

    dim : int (2 or 3)
        dimension for Q, where the final dimension is (dim x dim)

    dt : float, default=1.0
        time step in whatever units your filter is using for time. i.e. the
        amount of time between innovations

    var : float, default=1.0
        variance in the noise
    """

    assert dim == 2 or dim == 3
    if dim == 2:
        Q = np.array([[.25*dt**4, .5*dt**3],
                   [ .5*dt**3,    dt**2]], dtype=float)
    else:
        Q = np.array([[.25*dt**4, .5*dt**3, .5*dt**2],
                   [ .5*dt**3,    dt**2,       dt],
                   [ .5*dt**2,       dt,        1]], dtype=float)

    return Q * var

def cholupdate(R,x,sign):
    #http://stackoverflow.com/questions/8636518/dense-cholesky-update-in-python
    p = np.size(x)
    x = x.T
    for k in range(p):
        if sign == '+':
            r = np.sqrt(R[k,k]**2 + x[k]**2)
        elif sign == '-':
            r = np.sqrt(R[k,k]**2 - x[k]**2)
        c = r/R[k,k]
        s = x[k]/R[k,k]
        R[k,k] = r
        if sign == '+':
            R[k,k+1:p] = (R[k,k+1:p] + s*x[k+1:p])/c
        elif sign == '-':
            R[k,k+1:p] = (R[k,k+1:p] - s*x[k+1:p])/c
        x[k+1:p]= c*x[k+1:p] - s*R[k, k+1:p]
    return R

def sigmapoint_function(state, S):
    # TODO: Fill this in
    pass

def process_function(state, Q, dt):
    # TODO: Add first column of Q to output
    output = np.nan * np.ones((state.size,))

    # Position and linear updates
    output[(st_pos_x, st_pos_y, st_pos_z),] = state[(st_pos_x, st_pos_y, st_pos_z),] \
                                              + dt * state[(st_lvel_x, st_lvel_y, st_lvel_z),] \
                                              + dt * dt * state[(st_lacc_x, st_lacc_y, st_lacc_z),] / 2
    output[(st_lvel_x, st_lvel_y, st_lvel_z),] = state[(st_lvel_x, st_lvel_y, st_lvel_z),] \
                                                 + dt * state[(st_lacc_x, st_lacc_y, st_lacc_z),]
    output[(st_lacc_x, st_lacc_y, st_lacc_z),] = state[(st_lacc_x, st_lacc_y, st_lacc_z),]

    # Orientation update
    delta_q = quat_from_axang(state[(st_avel_x, st_avel_y, st_avel_z),], dt)
    q_out = quaternion_multiply(state[(st_e_w, st_e_x, st_e_y, st_e_z),], delta_q)  # Maybe to do: *qnoise in the middle
    output[(st_e_w, st_e_x, st_e_y, st_e_z),] = q_out / np.linalg.norm(q_out)
    output[(st_avel_x, st_avel_y, st_avel_z),] = state[(st_avel_x, st_avel_y, st_avel_z),]

    return output

def mean_function(sigmas, weights):
    # TODO: Only for ax-angle
    # Calculate average state x from sigma points.
    x = np.nan * np.ones(sigmas.shape[1])
    # For position, velocity, acceleration, and angular velocity this is simple
    x[st_lin_list,] = np.dot(weights, sigmas[:, st_lin_list])
    # For orientation, we need the weighted average quaternion.
    # http://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
    Q = sigmas[:, (st_e_w, st_e_x, st_e_y, st_e_z)].T * weights
    eig_val, eig_vec = np.linalg.eig(np.dot(Q, Q.T))
    q_out = eig_vec[:, np.argmax(eig_val)]
    q_out /= np.linalg.norm(q_out)
    x[(st_e_w, st_e_x, st_e_y, st_e_z),] = q_out
    return x
    pass

def residualx_function(sigma, state):
    # TODO: Fill this in
    pass

def observation_function(state, R):
    # TODO: add first row of R to output
    output = np.nan * np.ones((R.shape[0],))

    q = state[(st_e_w, st_e_x, st_e_y, st_e_z),]
    # world_to_cntrl = quaternion_matrix(quaternion_conjugate(q))
    accel_world_g = np.hstack((0, state[(st_lacc_x, st_lacc_y, st_lacc_z),] + grav_world)) / GRAVITY
    output[(obs_accel_x, obs_accel_y, obs_accel_z),] = quaternion_multiply(q, quaternion_multiply(accel_world_g,
                                                                                                  quaternion_conjugate(
                                                                                                      q)))[1:]
    # + accel_bias + accel_noise
    output[(obs_gyro_x, obs_gyro_y, obs_gyro_z),] = state[
        (st_avel_x, st_avel_y, st_avel_z),]  # + gyro_bias + gyro_noise
    output[(obs_mag_x, obs_mag_y, obs_mag_z),] = quaternion_multiply(q, quaternion_multiply(mag_world,
                                                                                            quaternion_conjugate(
                                                                                                q)))[1:]
    # + mag_bias + mag_noise
    output[(obs_track_x, obs_track_y, obs_track_z),] = state[(st_pos_x, st_pos_y, st_pos_z),]
    return output

def innovation_function(real_obs, pred_obs):
    #Not used by me
    return np.subtract(pred_obs, real_obs)

def addx_fn(state1, state2):
    #TODO: Handle angle-axis
    return state1 + state2

class Noise(object):
    def __init__(self, type='gaussian', cov_type='sqrt', dim=0, mu=0, cov=None, adapt_method=None, adapt_params=None):
        self.cov_type = cov_type
        self.dim = dim
        self.mu = mu

        cov = cov if cov else np.eye(self.dim)
        if self.cov_type == 'sqrt':
            self.cov = cholesky(cov)
        else:
            self.cov = cov

        self.adapt_method = adapt_method
        self.adapt_params = adapt_params

class SRUKF(object):

    def __init__(self, init_x=None, init_S=None, init_Q=None, init_R=Noise(dim=1),
                 alpha=0.1, beta=2.0, kappa=None,
                 special_state_inds=None, sigmapoint_fn=None, process_fn=None, mean_fn=None, residualx_fn=None):
        self.x = init_x if init_x else 0.0
        self.S = init_S if init_S else Noise(dim=self._xdim)
        self.Q = init_Q if init_Q else Noise(dim=self._xdim)
        self.R = init_R
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa if kappa else (3 - self._xdim)
        self.reset_weights()

        self.sigmapoint_fn = sigmapoint_fn
        self.special_xinds = np.arange(self._xdim) if (special_state_inds is None and self.sp_fn is not None) else special_state_inds

        self.process_fn = process_fn
        self.mean_fn = mean_fn
        self.residualx_fn = residualx_fn

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, val):
        self._xdim = val.size if val is not None else 0
        self._x = val

    def reset_weights(self):
        L = self._xdim + self.Q.dim + self.R.dim
        nsp = 2 * L + 1  # Number of sigma points
        kappa = self.alpha**2 * (L + self.kappa) - L
        W = np.asarray([kappa, 0.5, 0]) / (L + kappa)
        W[2] = W[0] + (1 - self.alpha**2) + self.beta

        self.W2isPos = W[2] >= 0
        self.W = W
        sqrtW = self.W
        sqrtW[2] = np.abs(sqrtW[2])
        self.sqrtW = np.sqrt(sqrtW)

        self.sqrt_L_plus_kappa = np.sqrt(L + kappa)

    def predict_step(self, dt, save=False):
        # Create augmented state variable
        x_a = np.concatenate((self.x, self.Q.mu, self.R.mu))

        # Create augmented covariance
        L = self.S.dim + self.Q.dim + self.R.dim
        S_a = np.zeros((L, L))
        S_a[0:self.S.dim, 0:self.S.dim] = self.S.cov
        S_a[self.S.dim:self.S.dim+self.Q.dim, self.S.dim:self.S.dim+self.Q.dim] = self.Q.cov
        S_a[L-self.R.dim:, L-self.R.dim:] = self.R.cov
        S_a *= self.sqrt_L_plus_kappa

        # Calculate sigma points
        nsp = 2 * L + 1
        X_a = np.tile(x_a, (1, nsp))
        # Default sigma point calculation
        X_a[:, 1:self._xdim + 1] += S_a
        X_a[:, self._xdim + 1:] -= S_a
        # Special sigma point calculation
        if self.sigmapoint_fn is not None:
            X_a[self.special_xinds, :] = self.sigmapoint_fn(x_a[self.special_xinds], S_a[self.special_xinds, :])

        # Propagate sigmapoints through process function. It should know how to handle special_inds
        X_k = np.nan * np.ones((self._xdim, X_a.shape[1]))
        for sig_ix in range(X_a.shape[1]):
            X_k[:, sig_ix] = self.process_fn(X_a[:self._xdim, sig_ix], X_a[self._xdim+1:self._xdim+self.Q.dim, sig_ix], dt)

        # Predicted state = weighted sum of propagated sigma points
        # Default mean
        x_k = self.W[0] * X_k[:, 0] + self.W[1] * np.sum(X_k[:, 1:], axis=1)
        if self.mean_fn is not None:
            # Special mean
            x_k[self.special_xinds] = self.mean_fn(X_k[self.special_xinds, :], self.W)

        # Sigma residuals, used in process noise
        # Default residuals
        X_k_residuals = np.subtract(X_k, x_k)
        if self.residualx_fn is not None:
            # Special residuals
            for sig_ix in range(X_k.shape[1]):
                X_k_residuals[self.special_xinds, sig_ix] = self.residualx_fn(X_k[self.special_xinds, sig_ix], x_k[self.special_xinds])

        # process noise = qr update of weighted X_k_residuals
        [_, S_k] = qr((self.sqrtW[1] * X_k_residuals[:, 1:]).T, mode='economic')  # Upper
        S_k = cholupdate(S_k, self.sqrtW[2] * X_k_residuals[:, 0], '+' if self.W2isPos else '-')  # Upper

        if save:
            self.X_a = X_a  # Augmented sigma points
            self.x_k = x_k  # Predicted state
            self.X_k = X_k  # Predicted sigma points
            self.X_k_residuals = X_k_residuals
            self.S_k = S_k  # Process noise

        return x_k, S_k.T

    def update(self, observation):
        L = self.S.dim + self.Q.dim + self.R.dim
        sigma_R = self.X_a[L-self.R.dim:, :]

        # Predict observation sigma points
        # TODO: Handle incomplete observation; only predict necessary variables
        Y_k = np.nan*np.ones((self.R.dim, self.X_k.shape[1]))
        for sig_ix in range(self.X_k.shape[1]):
            Y_k[:, sig_ix] = self.observation_fn(self.X_k[:, sig_ix], sigma_R[:, sig_ix])

        # Predicted observation = weighted sum of observation sigma points
        y_k = self.W[0]*Y_k[:, 0] + self.W[1]*np.sum(Y_k[:, 1:], axis=1)
        # TODO: custom observation mean if any variables are not addable

        # Observation residuals
        Y_k_residuals = np.subtract(Y_k, y_k)

        # Observation covariance
        [_, S_y] = qr((self.sqrtW[1] * Y_k_residuals[:, 1:]).T, mode='economic')  # Upper
        S_y = cholupdate(S_y, self.sqrtW[2] * Y_k_residuals[:, 0], '+' if self.W2isPos else '-')  # Upper
        S_y = S_y.T  # Need lower from upper

        # State-Observation covariance
        P_xy = self.W[2] * self.X_k_residuals[:, 0]*Y_k_residuals[:, 0].T + self.W[1]*self.X_k_residuals[:, 1:]*Y_k_residuals[:, 1:].T

        # Kalman gain
        K = lstsq(S_y, P_xy.T)[0]
        K = lstsq(S_y.T, K)[0]
        K = K.T

        # Calculate innovation
        innovation = np.subtract(observation, y_k)
        # TODO: custom innovation if any observation variables are not addable

        # Update state estimate
        upd = K*innovation
        self.x = self.x_k + upd
        if self.addx_fn is not None:
            self.x[self.special_xinds] = self.addx_fn(self.x_k, upd[self.special_xinds])

        # Update state covariance
        cov_update_vectors = K * S_y # Correct covariance.This is equivalent to:  Px = Px_ - KG * Py * KG';
        for j in range(self.R.dim):
            self.S_k = cholupdate(self.S_k, cov_update_vectors[:, j], '-')
        self.S = self.S_k.T

        if self.S.adapMethod:
            #TODO: self.S.cov = adapted_S
            pass