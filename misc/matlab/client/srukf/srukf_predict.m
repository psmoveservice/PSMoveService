function filt_struct = srukf_predict(filt_struct, dt)
%filt_struct = srukf_predict(filt_struct, dt)

%In the below variables, the subscripts are as follows
%k is the next/predicted time point
%t = k-1 (time point of previous estimate)

% Reminder:
% State vector: posx, velx, accx, posy, vely, accy, posz, velz, accz, avelx, avely, avelz, qw, qx, qy, qz
% Units: pos: m, vel: m/s, acc: m/s^2, avel: rad/s, orientation as quaternion
%
% state_cov vector: posx, velx, accx, posy, vely, accy, posz, velz, accz, avelx, avely, avelz, angx, angy, angz
% Units: Same as state vector.
%
% process noise vector: posx, velx, accx, posy, vely, accy, posz, velz, accz, avelx, avely, avelz, angx, angy, angz
% Units: Same as state vector.
%
% observation vector: a.x, a.y, a.z, g.x, g.y, g.z, m.x, m.y, m.z, pos.x, pos.y, pos.z
% Units: acc: g-units, gyro: rad/s, mag: normalized B-units, pos: m

%% 1. If weights are incorrect shape then reset them
if ~isfield(filt_struct, 'W')
    %TODO: Add more checks for individual fields.
    filt_struct = reset_weights(filt_struct);
end

%% 2. Calculate sigma points
% In the paper, the authors create an augmented state variable:
% [x; Q.mu; R.mu], with length L = Xdim + Q.dim + R.dim
% Then, when constructing the sigma points, this state vector is repeated
% 1 + 2*L times (spmat size is L x 1+2*L).
% Added to this sigmapoint matrix 2:end columns is the zeta-scaled
% augmented covariance matrix:
%  [S       zeros   zeros;
%   zeros   Q.cov   zeros;
%   zeros   zeros   R.cov]
% First negative, then positive. So, the resulting sigma point matrix looks
% like...
% [ x	x+zS	x       x       x-zS	x       x;
%   q   q       q+zQ	q       q       q-zQ	q;
%   r   r       r       r+zR	r       r       r-zR]
%
% But only the upper Xdim rows are sent to the process function, and the
% next Q.dim rows are sent separately as noise to be added to the sigma
% points. Using () to indicate propagation, our propagated state vectors +
% Q noise looks like:
% [(x)+q    (x+zS)+q    (x)+q+zQ	(x)+q   (x-zS)+q    (x)+q-zQ    (x)+q]
% Notice that we are only propagating x, x+zS, and x-zS. Therefore, those
% are really the only sigma points we need to create right now.
nsp = 1 + 2 * filt_struct.S.dim;

% The 1st sigma-point is just the state vector.
% Each remaining sigma-point is the state +/- the scaled sqrt covariance.
X_t = repmat(filt_struct.x, 1, nsp);
% zS = filt_struct.weights.zeta * filt_struct.S;  % zS is scaled sqrt cov.
% X_t(:, 2:(1+filt_struct.Xdim)) = X_t(:, 2:(1+filt_struct.Xdim)) +/- zS;
% Can't do this because of orientations.

% Some of the state variables (and covariance variables) are simple...
x_lin = filt_struct.x(filt_struct.x_lin_ix);
zS_lin = filt_struct.weights.zeta * filt_struct.S.cov(filt_struct.S.lin_ix, :);
X_t(filt_struct.x_lin_ix, :) = [x_lin bsxfun(@plus, x_lin, zS_lin) bsxfun(@minus, x_lin, zS_lin)];

% But some of the state variables represent orientations, and cannot be
% simply added or subtracted.
x_q = filt_struct.x(filt_struct.x_quat_ix);  % State orientation as quaternion
S_ang = filt_struct.weights.zeta * filt_struct.S.cov(filt_struct.S.axang_ix, :);  % sqrt orientation covariance
S_q = axisAngle2Quat(S_ang);
X_t(filt_struct.x_quat_ix, :) = [x_q quaternion_multiply(x_q, S_q) quaternion_multiply(x_q, quaternion_conjugate(S_q))];

% We now have our minimal sigma points: [x x+zS x-zS]

% Note: We could add Q (not sqrt) to P (=SS^T) before calculating S, and
% before calculating the sigma points. This would eliminate the need to add
% Q in the process function.

% % Test mean_quat because we know what the average should be...
% test_quats = X_t(filt_struct.x_quat_ix, :);
% test_quats = [test_quats repmat(test_quats(:, 1), 1, 2 * (filt_struct.Q.dim + filt_struct.R.dim))];
% [test_quats(:, 1) mean_quat(test_quats, filt_struct.weights.wm, 'pivot')]

%% 3. Propagate sigma points through process function
% Scale noise covariance before passing to process_function.
Q = filt_struct.Q;
Q.zcov = filt_struct.weights.zeta * Q.cov;
X_k = process_function(filt_struct, X_t, dt, Q);
% Note that X_k is wider than X_t because the addition of the process
% covariance required more columns.
% X_k: [(x)+q (x+zS)+q (x-zS)+q (x)+q+zQ (x)+q-zQ]

%% 4. Estimate mean state from weighted sum of propagated sigma points
x_k = nan(filt_struct.Xdim, 1);
wm = filt_struct.weights.wm;

% We will eventually need to extend X_k with 2*Rdim (for +/- zR in the
% observation function). The weights assume that X_k is already this wide,
% so let's extend it now using repeats of its first column.
X_k = [X_k repmat(X_k(:,1), 1, 2*filt_struct.R.dim)];

% We can now calculate the average state for the linear part.
x_k(filt_struct.x_lin_ix) = sum([wm(1) * X_k(filt_struct.x_lin_ix, 1), wm(2) * X_k(filt_struct.x_lin_ix, 2:end)], 2);
%mean([size(X_k, 2)*wm(1) * X_k(x_lin_ix, 1) size(X_k, 2)*wm(2)*X_k(x_lin_ix, 2:end)], 2);

%Average of quaternions in X_k(x_q_ix, :).
x_k(filt_struct.x_quat_ix) = mean_quat(X_k(filt_struct.x_quat_ix, :), wm, 'pivot');

%% 5. Get residuals in S-format
X_k_r = nan(filt_struct.S.dim, size(X_k, 2));
% Subtract linear parts
X_k_r(filt_struct.S.lin_ix, :) = bsxfun(@minus, X_k(filt_struct.x_lin_ix, :), x_k(filt_struct.x_lin_ix));
%Subtract quaternions, store result as AxisAngle.
X_k_r(filt_struct.S.axang_ix, :) = quat2AxisAngle(quaternion_multiply(X_k(filt_struct.x_quat_ix, :), quaternion_conjugate(x_k(filt_struct.x_quat_ix))));


%% 6. Estimate state covariance (sqrt)
%filt_struct.weights.w_qr is scalar
% QR update of state Cholesky factor.

% What does it mean to get orientation AxisAngle (co)variance?
% Does the qr update capture this? Probably not.

%filt_struct.weights.w_qr and .w_cholup cannot be negative.
Sx_k = triu(qr(filt_struct.weights.w_qr*X_k_r(:,2:end)', 0));
Sx_k = Sx_k(1:filt_struct.S.dim, 1:filt_struct.S.dim);
%NOTE: here Sx_k is the UPPER Cholesky factor (Matlab excentricity)
if filt_struct.weights.wc(1) > 0
    Sx_k = cholupdate(Sx_k, filt_struct.weights.w_cholup*X_k_r(:,1), '+');
else
    % deal with possible negative zero'th covariance weight
    Sx_k = cholupdate(Sx_k, filt_struct.weights.w_cholup*X_k_r(:,1), '-');
end

% Save results to filt_struct.intermediates
filt_struct.intermediates.X_k = X_k;
filt_struct.intermediates.X_k_r = X_k_r;
filt_struct.intermediates.x_k = x_k;
filt_struct.intermediates.Sx_k = Sx_k;  %Still UPPER Cholesky factor

end

%% Helper functions
function filt_struct = reset_weights(filt_struct)
% state dimension
L = filt_struct.S.dim + filt_struct.Q.dim + filt_struct.R.dim;

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