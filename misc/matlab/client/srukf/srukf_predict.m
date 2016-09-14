function filt_struct = srukf_predict(filt_struct, dt)
%

%In the below variables, the subscripts are as follows
%k is the next/predicted time point
%t = k-1 (time point of previous estimate)

% Reminder:
% state vector: posx, velx, accx, posy, vely, accy, posz, velz, accz, qw, qx, qy, qz, avelx, avely, avelz
% Units: pos: m, vel: m/s, acc: m/s^2, orient. in quat, avel: rad/s
%
% state_cov vector: posx, velx, accx, posy, vely, accy, posz, velz, accz, angx, avelx, angy, avely, angz, avelz
% Units: Same as state vector.
%
% process noise vector: posx, velx, accx, posy, vely, accy, posz, velz, accz, angx, avelx, angy, avely, angz, avelz
% Units: Same as state vector.
%
% observation vector: a.x, a.y, a.z, g.x, g.y, g.z, m.x, m.y, m.z, pos.x, pos.y, pos.z
% Units: acc: g-units, gyro: rad/s, mag: B-units, pos: cm

%% 1. If weights are incorrect shape then reset them
if ~isfield(filt_struct, 'W')
    %TODO: Add more checks.
    filt_struct = reset_weights(filt_struct);
end

%% 2. Calculate sigma points
nsp = 1 + 2 * filt_struct.Sdim;

% The 1st sigma-point is just the state vector.
% Each remaining sigma-point is the state +/- the scaled sqrt covariance.
X_t = repmat(filt_struct.x, 1, nsp);
% zS = filt_struct.weights.zeta * filt_struct.S;  % zS is scaled sqrt cov.
% X_t(:, 2:(1+filt_struct.Xdim)) = X_t(:, 2:(1+filt_struct.Xdim)) +/- zS;
% Can't do this because of orientations.

% Some of the state variables (and covariance variables) are simple...
x_lin = [1:9 14 15 16];  % pos, vel, accel, avel
simple_x = filt_struct.x(x_lin);
s_lin = [1:9 11 13 15];  % pos, vel, accel, avel
simple_zS = filt_struct.weights.zeta * filt_struct.S(s_lin, s_lin);
X_t(x_lin, 1 + s_lin) = bsxfun(@plus, simple_x, simple_zS);
X_t(x_lin, 1 + filt_struct.Sdim + s_lin) = bsxfun(@minus, simple_x, simple_zS);

% But some of the state variables represent orientations, and cannot be
% simply added or subtracted.
x_qinds = 10:13;
q = filt_struct.x(x_qinds);  % State orientation as quaternion
s_ang = [10 12 14];
S_a = filt_struct.weights.zeta * filt_struct.S(s_ang, s_ang);  % sqrt orientation covariance
S_q = axisAngle2Quat(S_a);
X_t(x_qinds, 1 + s_ang) = quaternion_multiply(q, S_q);  % TODO: Check order.
X_t(x_qinds, 1 + filt_struct.Sdim + s_ang) = quaternion_multiply(q, quaternion_conjugate(S_q));

% Note: We could add Q (not sqrt) to P (=SS^T) before calculating S, and
% before calculating the sigma points. This would eliminate the need to add
% Q in the process function.

%% 3. Propagate sigma points through process function
zQ = filt_struct.Q;
zQ.cov = filt_struct.weights.zeta * zQ.cov;
X_k = process_function(X_t, zQ, dt);
% Note that X_k is larger than X_t because the process noise added more
% state vectors.

%% 4. Estimate mean state from weighted sum of propagated sigma points
x_k = nan(filt_struct.Xdim, 1);
wm = filt_struct.weights.wm;
% Extend X_k with 2*filt_struct.R.dim repeats of the first column
% This emulates the rest of the augmented matrix. It's necessary to extend
% it here because the weights only work with the correct number of columns.
X_k = [X_k repmat(X_k(:,1), 1, 2*filt_struct.R.dim)];
x_k(x_lin) = sum([wm(1) * X_k(x_lin, 1), wm(2) * X_k(x_lin, 2:end)], 2);

%Average of quaternions in X_k(x_qinds, :).
%For linear variables, weights (wm) scale each column (by ~ 1/L) so their
%addition results in an average.
%But we can't do the same thing with orientation.
%We have a function to do weighted average of quaternions in
%mean_quat, but this actually averages, instead of sums, so the weight
%scaling by ~1/L is undesirable. TODO: Can we 'unscale' the weights first?
%Alternatively, we can calculate the delta orientations in each column,
%convert to axang, scale and add those, convert to quat, update 0th quat.
delta_quats = quaternion_multiply(X_k(x_qinds, 2:end), quaternion_conjugate(X_k(x_qinds, 1)));
delta_axang = sum(wm(2) * quat2AxisAngle(delta_quats), 2);
delta_quat = axisAngle2Quat(delta_axang);
x_k(x_qinds) = quaternion_multiply(X_k(x_qinds, 1), delta_quat);
%TODO: But what about wm(1)?
%TODO: Test average quaternions function on X_t(x_qinds, :) to see if it
%returns filt_struct.x(x_qinds)

%% 5. Get residuals in S-format
X_k_r = nan(filt_struct.Sdim, size(X_k, 2));
% Subtract linear parts
X_k_r(s_lin, :) = bsxfun(@minus, X_k(x_lin, :), x_k(x_lin));
%Subtract quaternions
X_k_r(s_ang, :) = quat2AxisAngle(quaternion_multiply(X_k(x_qinds, :), quaternion_conjugate(x_k(x_qinds))));


%% 6. Estimate state covariance (sqrt)
%filt_struct.weights.w_qr is scalar
% QR update of state Cholesky factor.

% What does it mean to get quaternion (co)variance?
% Does the qr update capture this? Probably not.
% We may need to get Sx_k the slow way.

%filt_struct.weights.w_qr and .w_cholup cannot be negative.
Sx_k = triu(qr(filt_struct.weights.w_qr*X_k_r(:,2:end)', 0));
Sx_k = Sx_k(1:filt_struct.Sdim, 1:filt_struct.Sdim);
%NOTE: here Sx_k is the UPPER Cholesky factor (Matlab excentricity)
if filt_struct.weights.wc(1) > 0
    Sx_k = cholupdate(Sx_k, filt_struct.weights.w_cholup*X_k_r(:,1), '+');
else
    % deal with possible negative zero'th covariance weight
    Sx_k = cholupdate(Sx_k, filt_struct.weights.w_cholup*X_k_r(:,1), '-');
end

% Save results to filt_struct.intermediates
filt_struct.intermediates.X_t = X_t;
filt_struct.intermediates.X_k = X_k;
filt_struct.intermediates.X_k_r = X_k_r;
filt_struct.intermediates.x_k = x_k;
filt_struct.intermediates.Sx_k = Sx_k;  %Still UPPER Cholesky factor

end

%% Helper functions
function filt_struct = reset_weights(filt_struct)
% state dimension
L = filt_struct.Sdim + filt_struct.Q.dim + filt_struct.R.dim;

% For standard UKF, here are the weights...

% compound scaling parameter
lambda = filt_struct.alpha^2 * (L + filt_struct.kappa) - L;

% Scaling factor for sigma points.
zeta = sqrt(L + lambda);

%wm = weights for calculating mean (both process and observation)
wm_0 = lambda / (L + lambda);
wm_rest = 0.5 / (L + lambda);

%wc = weights for calculating covariance (proc., obs., proc-obs)
wc_0 = wm_0 + (1 - filt_struct.alpha^2 + filt_struct.beta);
%wc_rest = wm_rest

% For SRUKF, we also need sqrt of wc_rest for chol update.
w_qr = sqrt(wm_rest);
w_cholup = sqrt(abs(wc_0));

filt_struct.weights = struct(...
    'zeta', zeta,...
    'wm', [wm_0 wm_rest],...
    'wc', [wc_0 wm_rest],...
    'w_qr', w_qr,...
    'w_cholup', w_cholup);

% % sigma-point weights
% W = [lambda 0.5 0]/(L+lambda);                                    
% W(3) = W(1) + (1-filt_struct.alpha^2) + filt_struct.beta;
% sqrtW = W;
% positive_W3 = (W(3) > 0);  % is zero'th covariance weight positive?
% sqrtW(1:2) = sqrt(W(1:2));  % square root weights
% sqrtW(3) = sqrt(abs(W(3)));
% 
% % Matlab W : Python Wc, Wm
% % W(1) = Wm[0]
% % W(2) = Wm[1:] or Wc[1:] (same)
% % W(3) = Wc[0]

end