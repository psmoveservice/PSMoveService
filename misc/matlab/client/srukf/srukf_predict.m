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

%% 3. Build augmented covariance matrix
S_a = zeros(L);
S_a(1:filt_struct.Xdim, 1:filt_struct.Xdim) = filt_struct.S;
Q_inds = filt_struct.Xdim + 1:filt_struct.Xdim + filt_struct.Q.dim;
S_a(Q_inds, Q_inds) = filt_struct.Q.cov;
R_inds = filt_struct.Xdim + filt_struct.Q.dim + 1:L;
S_a(R_inds, R_inds) = filt_struct.R.cov;
S_t = filt_struct.Sqrt_L_plus_kappa * S_a;

%% 4. Calculate sigma points
X_t = x_t(:, ones(nsp,1));
sSzM = [S_t -S_t];
X_t(:, 2:nsp) = X_t(:, 2:nsp) + sSzM;
if isfield(filt_struct, 'special_xinds') && ~isempty(filt_struct.special_xinds)
    X_t(filt_struct.special_xinds, 2:size(S_t, 2) + 1) = filt_struct.special_state_add(x_t(filt_struct.special_xinds), S_t(filt_struct.special_xinds, :));
	X_t(filt_struct.special_xinds, size(S_t, 2) + 2:end) = filt_struct.special_state_sub(x_t(filt_struct.special_xinds), S_t(filt_struct.special_xinds, :));
end

%% 5. Propagate sigma points through process function
X_k = nan(filt_struct.Xdim, nsp);
for sp_ix = 1:nsp
    X_k(1:filt_struct.Xdim, sp_ix) = process_function(X_t(1:filt_struct.Xdim, sp_ix), filt_struct.Q, dt);
end

%% 6. Estimate mean state from weighted sum of propagated sigma points
x_k = filt_struct.W(1)*X_k(:,1) + filt_struct.W(2)*sum(X_k(:,2:nsp),2);
if isfield(filt_struct, 'special_xinds') && ~isempty(filt_struct.special_xinds)
    x_k(filt_struct.special_xinds) = filt_struct.special_state_mean(X_k(filt_struct.special_xinds, :), filt_struct.W);
end

%% 7. Get residuals
X_k_r = bsxfun(@minus, X_k, x_k);
if isfield(filt_struct, 'special_xinds') && ~isempty(filt_struct.special_xinds)
    X_k_r(filt_struct.special_xinds, :) = filt_struct.special_state_sub(X_k(filt_struct.special_xinds, :), x_k(filt_struct.special_xinds));
end

%% 8. Estimate state covariance (sqrt)
% QR update of state Cholesky factor.
[~, Sx_k] = qr((filt_struct.sqrtW(2)*X_k_r(:,2:nsp))', 0);
%NOTE: here Sx_k is the UPPER Cholesky factor (Matlab excentricity)

% deal with possible negative zero'th covariance weight
if filt_struct.possitive_W3
    Sx_k = cholupdate(Sx_k, filt_struct.sqrtW(3)*X_k_r(:,1), '+');
else
    Sx_k = cholupdate(Sx_k, filt_struct.sqrtW(3)*X_k_r(:,1), '-');
end

% TODO: Save results to filt_struct.intermediates
filt_struct.intermediates.X_k = X_k;
filt_struct.intermediates.X_k_r = X_k_r;
filt_struct.intermediates.x_k = x_k;
filt_struct.intermediates.Sx_k = Sx_k;  %Still UPPER Cholesky factor

end

%% Helper functions
function filt_struct = reset_weights(filt_struct)
% Matlab W : Python Wc, Wm
% W(1) = Wm[0]
% W(2) = Wm[1:] or Wc[1:] (same)
% W(3) = Wc[0]

% augmented state dimension
L = filt_struct.Xdim + filt_struct.Q.dim + filt_struct.R.dim;

% compound scaling parameter
lambda = filt_struct.alpha^2 * (L + filt_struct.kappa) - L;

% sigma-point weights
W = [lambda 0.5 0]/(L+lambda);                                    
W(3) = W(1) + (1-filt_struct.alpha^2) + filt_struct.beta;

sqrtW = W;
possitive_W3 = (W(3) > 0);  % is zero'th covariance weight possitive?
sqrtW(1:2) = sqrt(W(1:2));  % square root weights
sqrtW(3) = sqrt(abs(W(3)));

Sqrt_L_plus_kappa = sqrt(L + lambda);

filt_struct.W = W;
filt_struct.sqrtW = sqrtW;
filt_struct.possitive_W3 = possitive_W3;
filt_struct.Sqrt_L_plus_kappa = Sqrt_L_plus_kappa;
end