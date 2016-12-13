function new_states = process_function(filt_struct, old_states, dt, varargin)
% new_states = process_function(old_states, Q, dt)
% Process function for square-root sigma point Kalman filter
% old_states -  A matrix where each column is a state vector (like a set of
%               sigma points)
%           posx, velx, accx,
%           posy, vely, accy,
%           posz, velz, accz,
%           avelx, avely, avelz,
%           qw, qx, qy, qz
%               In SRUKF with augmented state vector, old_states is
%               [x_t    x_t + zS    x_t - zS]
% Q - A structure with fields .mu, and .cov. .mu fields:
%           posx, velx, accx,
%           posy, vely, accy,
%           posz, velz, accz,
%           avelx, avely, avelz,
%           angx, angy, angz
% dt -  A scalar indicating time delta (in seconds) to predict state
%       evolution.
% new_states -	Return value. A matrix where each column is a state vector,
%               like the input. In SRUKF with augmented state vector,
%               new_states is....
%               [p(x_t) + Q.mu,  p(x_t + zS) + Q.mu, p(x_t - zS) + Q.mu,
%               p(x_t) + Q.mu + z*Q.cov, p(x_t) + Q.mu - z*Q.cov]
if ~isempty(varargin)
    Q = varargin{1};
else
    Q = filt_struct.Q;
end
[Xdim, L] = size(old_states);
% Sdim = (L - 1) / 2;  % Number of state covariance variables.
new_states = nan(Xdim, L + 2 * Q.dim);

% Extract useful variables from sigma points
pos = old_states([1, 4, 7], :);  % In m
lvel = old_states([2, 5, 8], :);  % In m/s
lacc = old_states([3, 6, 9], :);  % In m/s^2
avel = old_states(10:12, :);
q_old = old_states(filt_struct.x_quat_ix, :);

% Simple Cartesian updates
new_states([1, 4, 7], 1:L) = pos + dt*lvel;% + 0.5 * dt * dt * lacc;  %m; integrating accel may not be necessary.
new_states([2, 5, 8], 1:L) = lvel + dt*lacc;  %m/s
new_states([3, 6, 9], 1:L) = lacc;  % m/s^2; Assume no change in accel
new_states(10:12, 1:L) = avel; % In rad/s; Assume angular velocity does not change.

% Orientation updates

% From Kraft or Enayati:
q_delta = axisAngle2Quat(avel*dt);
new_states(filt_struct.x_quat_ix, 1:L) = quaternion_multiply(q_old, q_delta);

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

% Add process noise to propagated sigma points

% TODO: What's the difference between adding process noise to the
% propagated sigma points vs adding process noise to the input sigma points
% then propagating those?
% i.e. x_base = propagate(old_states(x_lin, 1) + Q.mu(Q_lin_ix))
% In a couple papers I've read, q_noise is 'added' (quat_mult) first before
% the process function:
% q_new = q_old * (q_bias * q_noise) * q_delta
% To keep consistent with above, where process noise is added last, use:
% q_new = q_old * q_delta * (q_bias * q_noise)

% Linear-Add process noise to propagated sigma points
% First add bias (Q.mu) to ALL sigma points.
new_states(filt_struct.x_lin_ix, :) = bsxfun(@plus, new_states(filt_struct.x_lin_ix, :), Q.mu(Q.lin_ix));
% Second add/subtract zQ noise in extension sigma points...
x_base = new_states(filt_struct.x_lin_ix, 1);  % Extension SPs based on 1st SP
zQ = Q.zcov(Q.lin_ix, :);
new_states(filt_struct.x_lin_ix, L+1:end) = [bsxfun(@plus, x_base, zQ) bsxfun(@minus, x_base, zQ)];

% Add orientation process noise.
% First, add orientation bias to all propagated sigma points.
q_bias = axisAngle2Quat(Q.mu(Q.axang_ix));
new_states(filt_struct.x_quat_ix, 1:L) = quaternion_multiply(new_states(filt_struct.x_quat_ix, 1:L), q_bias);
% Second, 'add' orientation noise to propagated extension sigma points.
q_base = new_states(filt_struct.x_quat_ix, 1);
q_base = repmat(q_base, 1, Q.dim);
zQ = axisAngle2Quat(Q.zcov(Q.axang_ix, :));
new_states(filt_struct.x_quat_ix, L+1:end) = [quaternion_multiply(q_base, zQ) quaternion_multiply(q_base, quaternion_conjugate(zQ))];