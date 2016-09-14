function new_states = process_function(old_states, Q, dt)
% new_states = process_function(old_states, Q, dt)
% Process function for square-root sigma point Kalman filter
% old_states -  A matrix where each column is a state vector (like a set of
%               sigma points)
%           posx, velx, accx,
%           posy, vely, accy,
%           posz, velz, accz,
%           qw, qx, qy, qz,
%           avelx, avely, avelz
%               In SRUKF with augmented state vector, old_states is
%               [x_t    x_t + zS    x_t - zS]
% Q - A structure with fields .mu, and .cov. .mu fields:
%           posx, velx, accx,
%           posy, vely, accy,
%           posz, velz, accz,
%           angx, avelx, angy, avely, angz, avelz
% dt -  A scalar indicating time delta (in seconds) to predict state
%       evolution.
% new_states -	Return value. A matrix where each column is a state vector,
%               like the input. In SRUKF with augmented state vector,
%               new_states is....
%               [p(x_t) + Q.mu,  p(x_t + zS) + Q.mu, p(x_t - zS) + Q.mu,
%               p(x_t) + Q.mu + z*Q.cov, p(x_t) + Q.mu - z*Q.cov]
[Xdim, L] = size(old_states);
Sdim = (L - 1) / 2;  % Number of state covariance variables.
new_states = nan(Xdim, L + 2 * Q.dim);

% Simple Cartesian updates
pos = old_states([1, 4, 7], :);  % In m
lvel = old_states([2, 5, 8], :);  % In m/s
lacc = old_states([3, 6, 9], :);  % In m/s^2

new_states([1, 4, 7], 1:L) = pos + dt*lvel + 0.5 * dt * dt * lacc;  % Last term sometimes unused in literature.
new_states([2, 5, 8], 1:L) = lvel + dt*lacc;
new_states([3, 6, 9], 1:L) = lacc;

% Assume angular velocity does not change.
avel = old_states(14:16, :);  % In rad/s
new_states(14:16, 1:L) = avel;

% Linear-Add process noise to propagated sigma points
x_lin = [1:9, 14 15 16];  % Indices into x of linear components.
Q_lin = [1:9, 11 13 15];  % Indices into Q of linear components
% TODO: What's the difference between adding process noise to the
% propagated sigma points vs adding process noise to the input sigma points
% then propagating those?
% i.e. x_base = propagate(old_states(x_lin, 1) + Q.mu(Q_lin))
% First add bias (Q.mu) to ALL sigma points.
new_states(x_lin, 1:L) = bsxfun(@plus, new_states(x_lin, 1:L), Q.mu(Q_lin));
% Second add/subtract noise in extension sigma points.
x_base = new_states(x_lin, 1);  % Extension SPs based on 1st SP
new_states(x_lin, L + 1 : end) = repmat(x_base, 1, 2 * Q.dim);  % Initialize with base values.
new_states(x_lin, L + Q_lin) = new_states(x_lin, L + Q_lin) + Q.cov(Q_lin, Q_lin);  % +zQ
new_states(x_lin, L + Q.dim + Q_lin) = new_states(x_lin, L + Q.dim + Q_lin) - Q.cov(Q_lin, Q_lin);  % -zQ

% Orientation updates
% In a couple papers I've read, q_noise is 'added' (quat_mult) first before
% the process function:
% q_new = q_old * (q_bias * q_noise) * q_delta
% To keep consistent with above, where process noise is added last, use:
% q_new = q_old * q_delta * (q_bias * q_noise)
x_quat = 10:13;
q_old = old_states(x_quat, :);

% From Kraft or Enayati:
q_delta = axisAngle2Quat(avel*dt);
new_states(x_quat, 1:L) = quaternion_multiply(q_old, q_delta);

% % Alternative ways with
% % Omega = skew-symmetric form of body rotations about reference frame
% 
% % q_delta from Schleppe '96
% tmp = reshape(avel, [3 1 size(avel, 2)]);
% Omega = [...
%     zeros(1, 1, L)	tmp(3, 1, :)	-tmp(2, 1, :)	tmp(1, 1, :);...
%     -tmp(3, 1, :)	zeros(1, 1, L)  tmp(1, 1, :)    tmp(2, 1, :);...
%     tmp(2, 1, :)    -tmp(1, 1, :)   zeros(1, 1, L)  tmp(3, 1, :);...
%     -tmp(1, 1, :)   -tmp(2, 1, :)   -tmp(3, 1, :)   zeros(1, 1, L)];
% q_delta = 0.5 * Omega * q_old * dt;
% new_states(x_quat, 1:L) = quaternion_multiply(q_old, q_delta);
% 
% % from van der Merwe, Wan, and Julier
% tmp = reshape(avel, [3 1 size(avel, 2)]);
% Omega = [...
%     zeros(1, 1, L)	-tmp(1, 1, :)	-tmp(2, 1, :)	-tmp(3, 1, :);...
%     tmp(1, 1, :)	zeros(1, 1, L)  tmp(3, 1, :)	-tmp(2, 1, :);...
%     tmp(2, 1, :)	-tmp(3, 1, :)   zeros(1, 1, L)  tmp(1, :, :);...
%     tmp(3, 1, :)	tmp(2, 1, :)    -tmp(1, 1, :)   zeros(1, 1, L)];
% PHI_delta = 0.5 * Omega * dt;
% s = 0.5 * norm(avel*dt);
% alpha = 1 - sum(q_old.^2);
% zeta = 1;
% ls = bsxfun(@times, repmat(eye(4), 1, 1, L), reshape(cos(s) + zeta * dt * alpha, 1, 1, L));
% q_delta = ls + PHI_delta * (sin(s)/s);  % 4x4 matrices
% for sp_ix = 1:L
%     new_states(x_quat, sp_ix) = q_delta(:, :, sp_ix) * q_old(:, sp_ix);
% end

% Add orientation bias to all propagated sigma points.
Q_axang = [10 12 14];
q_bias = axisAngle2Quat(Q.mu(Q_axang));
new_states(x_quat, 1:L) = quaternion_multiply(new_states(x_quat, 1:L), q_bias);

% 'Add' orientation noise to propagated extension sigma points.
q_base = new_states(x_quat, 1);
new_states(x_quat, L+1:end) = repmat(q_base, 1, 2*Q.dim);
q_noise = axisAngle2Quat(Q.cov(Q_axang, Q_axang));
new_states(x_quat, L + Q_axang) = quaternion_multiply(q_base, q_noise);  % +zQ
new_states(x_quat, L + Q.dim + Q_axang) = quaternion_multiply(q_base, quaternion_conjugate(q_noise));  % -zQ