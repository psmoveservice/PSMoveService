function filt_struct = srukf_predict(filt_struct, dt)
%

%In the below variables, the subscripts are as follows
%k is the next/predicted time point
%t = k-1 (time point of previous estimate)

%% 1. If weights are incorrect shape then reset them
if ~isfield(filt_struct, 'W')
    %TODO: Add more checks.
    filt_struct = reset_weights(filt_struct);
end

%% 2. Build augmented state vector
L = filt_struct.Xdim + filt_struct.Q.dim + filt_struct.R.dim;  % augmented state dimension
nsp = 2*L+1;
x_t = [filt_struct.x; filt_struct.Q.mu; filt_struct.R.mu];

%% 3. Build augmented sqrt state covariance matrix
S_a = zeros(L);  % = sqrt(state_covariance)
S_a(1:filt_struct.Xdim, 1:filt_struct.Xdim) = filt_struct.S;  % LOWER triangular
Q_inds = filt_struct.Xdim + 1:filt_struct.Xdim + filt_struct.Q.dim;
S_a(Q_inds, Q_inds) = filt_struct.Q.cov;
R_inds = filt_struct.Xdim + filt_struct.Q.dim + 1:L;
S_a(R_inds, R_inds) = filt_struct.R.cov;

%% 4. Calculate augmented sigma points
zetaSa = filt_struct.weights.zeta * S_a;
X_t = [x_t, x_t(:, ones(L,1))+zetaSa, x_t(:, ones(L,1))-zetaSa];
if isfield(filt_struct, 'special_xinds') && ~isempty(filt_struct.special_xinds)
    X_t(filt_struct.special_xinds, 2:L+1) = filt_struct.special_state_add(x_t(filt_struct.special_xinds), zetaSa(filt_struct.special_xinds, :));
	X_t(filt_struct.special_xinds, L+2:end) = filt_struct.special_state_sub(x_t(filt_struct.special_xinds), zetaSa(filt_struct.special_xinds, :));
end
% Note that
% X_t(1:Xdim, :) will be the state sigma points.
% X_t(Q_inds, :) will be used for process noise.
% X_t(R_inds, :) will be used for observation noise.

%% 5. Propagate sigma points through process function
X_k = process_function(X_t(1:filt_struct.Xdim, :), X_t(Q_inds, :), dt);

%% 6. Estimate mean state from weighted sum of propagated sigma points
x_k = sum(bsxfun(@times, filt_struct.weights.wm, X_k), 2);
if isfield(filt_struct, 'special_xinds') && ~isempty(filt_struct.special_xinds)
    x_k(filt_struct.special_xinds) = filt_struct.special_state_mean(X_k(filt_struct.special_xinds, :), filt_struct.weights.wm);
end

%% 7. Get residuals
X_k_r = bsxfun(@minus, X_k, x_k);
if isfield(filt_struct, 'special_xinds') && ~isempty(filt_struct.special_xinds)
    X_k_r(filt_struct.special_xinds, :) = filt_struct.special_state_sub(X_k(filt_struct.special_xinds, :), x_k(filt_struct.special_xinds));
end
% Only residuals from sigma points 2:nsp will be used.

%% 8. Estimate state covariance (sqrt)
%filt_struct.weights.w_qr is scalar
% QR update of state Cholesky factor.

%filt_struct.weights.w_qr and .w_cholup cannot be negative.x
[~, Sx_k] = qr((filt_struct.weights.w_qr*X_k_r(:,2:nsp))', 0);
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
% augmented state dimension
L = filt_struct.Xdim + filt_struct.Q.dim + filt_struct.R.dim;

% For standard UKF, here are the weights...

% compound scaling parameter
lambda = filt_struct.alpha^2 * (L + filt_struct.kappa) - L;

% Scaling factor for sigma points.
zeta = sqrt(L + lambda);

%wm = weights for calculating mean (both process and observation)
wm_0 = lambda / (L + lambda);
wm_rest = 0.5 / (L + lambda);
wm = [wm_0 wm_rest*ones(1, 2*L)];

%wc = weights for calculating covariance (proc., obs., proc-obs)
wc_0 = wm_0 + (1 - filt_struct.alpha^2 + filt_struct.beta);
wc_rest = wm_rest;
wc = [wc_0 wc_rest*ones(1,2*L)];

% For SRUKF, we also need sqrt of wc_rest for chol update.
w_qr = sqrt(wc(2));
w_cholup = sqrt(abs(wc(1)));

filt_struct.weights = struct(...
    'zeta', zeta,...
    'wm', wm,...
    'wc', wc,...
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