# alpha is a constant that determines spread of sigma points around states x. E-4 < alpha < 1
# kappa is secondary scaling parameter
# beta is used to incorporate prior knowledge of the distribution of states x, beta = 2
# lambda = alpha^2 * (L + kappa) - L
# gamma = sqrt(L + lambda)
# W_0(m) = lambda / (L + lambda)
# W_0(c) = W_0(m) + 1 - alpha^2 + beta
# W_i(m) = W_i(c) = 1 / 2(L + lambda) for i = 1:2L

class Parameters(object):
    Covariance_size = 100 #???
    AlphaSquared = 1.0
    Beta = 0.0
    Kappa = 3.0
    Lambda = AlphaSquared * (Covariance_size + Kappa) - Covariance_size
    MRP_A = 0.0
    MRP_F = 2.0 * (MRP_A + 1.0)
    Sigma_WM0 = Lambda / (Covariance_size + Lambda)
    Sigma_WC0 = Sigma_WM0 + (1.0 - AlphaSquared + Beta)
    Sigma_WMI = 1.0 / (2.0 * Covariance_size + Lambda)
    Sigma_WCI = Sigma_WMI

# State vector: x
# ---------------
# Position x, y, z (m)
# Velocity x, y, z (m/s)
# Acceleration x, y, z (m/s^2)
# Orientation w, x, y, z (Quaternion)
# AngularVelocity x, y, z (rad/s)
# AngularAcceleration x, y, z (rad/s)
# GyroBias?

# Q = process noise covariance
# ----------------------------
# a.k.a. R^v

# P = dynamic state covariance
# ----------------------------
#

# Measurement vector: z
# ---------------------
# TrackerPosition x, y, z
# Magnetometer x, y, z
# Accelerometer x, y, z
# Gyroscope x, y, z

# R = static measurement noise covariance
# ---------------------------------------
# a.k.a. R^n
# Only use diagonals
# Can be calculated emperically

# State update model: F
# ---------------------
# (May have to mess around with these if not in same coordinate frame)
# Position is deltaT*last_veloc
# Velocity is deltaT*last_accel
# Aceleration = last_accel
# Orientation = deltaT * last_ang_vel
# AngularVelocity = dletaT * last_ang_accel
# AngularAcceleration = last_ang_accel

# Process model (state->sensor): H
# --------------------------------
# Magnetometer = Orientation * local_magnetic_field
# Accelerometer = Acceleration  + Orientation * gravity; gravity = (0, 0, -9.80665)
# Gyroscope = AngularVelocity + GyroscopeBias
# TrackerPosition = Position

# Initialize
# ----------
# x0 = expected initial state (mean of some default position and orientation with 0 vel and accel)
# S0 = chol(expected initial state covariance)

# On every iteration:

# Time Update
# -----------
# Predict state:
#   X_(k-1) = [x_(k-1)  x_(k-1) + gamma*S_k  x_(k-1) - gamma*S_k]  i.e., draw sigma points
#   X_k = F(X_(k-1))      i.e., propagate sigma points through state transition matrix (update model)
#   x_k = sum([W_i(m) * X_ik for i in 0:2L])  i.e., predicted state is weighted sum of propagated sigma points
# Update process noise covariance:
#   S_k = qr([sqrt(W_1(c)) * (X_(1:2L,k) - x_k)   sqrt(Q)])  i.e., updated cholesky factor from qr decomp of weighted propagated sigma points and sqrt process noise covariance
#   S_k = cholupdate(S_k, X_(0,k) - x_k, W_0(c))
#   X_k = [x_k    x_k + gamma*S_k     x_k - gamma*S_k]  i.e., redraw sigma points to incorporate effect of process noise
# Predict measurements:
# Y_k = H(X_k)  i.e., propagate sigma points through measurement process model to get measurement sigma points
# y_k = sum([W_i(m)Y_(i,k) for i in 0:2L])  i.e., predicted mreasurements are weighted sum of propagated measurement sigma points

# Measurement Update
# ------------------
# Update measurement noise covariance:
#   S_yk = qr([sqrt(W_1(c)) * [Y_(1:2L,k) - y_khat]  sqrt(R_k)])
#   S_yk = cholupdate(S_yk, Y_(0,k) - y_khat, W_0(c))
# Calculate measurement/process noise covariance:
#   P_(xk,yk) = sum(W_i(c) * [X_(i,k) - x_k]*[Y_(i,k) - y_khat]^T for i in 0:2L)
# Calculate kalman gain:
#   K_k = (P_(xk,yk) / S_(yk)^T) / S_yk
# Update state with novel measurements
#   x_k = x_k + K_k(y_k - y_khat)  # state += kalman gain * unexpected_measurements
# Posterior update of state covariance factor
#   U + K_k * S_yk
#   S_k = cholupdate(S_k, U, -1)

class MyUKF(object):
    sigma_points = None
    w_prime = None
    z_prime = None
    state = None
    state_covariance_root = None
    measurement = None
    measurement_covariance_root = None

    def step(self, delta_t, measurement=None):
        self.a_priori_step(delta_t)
        self.measurement_step(measurement)
        self.a_posteriori_step()

    def perturb_state(self, state, cov):
        temp = [state, cov+state, -(cov-state)]

    def calculate_field_sigmas(self, S):
        return self.perturb_state(get_state(), S)

    def a_priori_step(self, delta_t):
        self.sigma_points = self.calculate_field_sigmas(self.state_covariance_root * sqrt(cov_size + Parameters.Lambda))