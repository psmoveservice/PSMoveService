function filt_struct = srukf_update(filt_struct, observation)
X_k = filt_struct.intermediates.X_k;
X_k_r = filt_struct.intermediates.X_k_r;
x_k = filt_struct.intermediates.x_k;
Sx_k = filt_struct.intermediates.Sx_k;
%% 1. Propagate sigma points through observation function.
nsp = size(X_k, 2);
Y_k = nan(filt_struct.Odim, nsp);
for sp_ix = 1:nsp
    Y_k(:, nsp) = observation_function(filt_struct, X_k(:, nsp), filt_struct.R);
end

%% 2. Calculate observation mean.
y_k = filt_struct.W(1) * Y_k(:, 1) + filt_struct.W(2) * sum(Y_k(:, 2:end), 2);
if isfield(filt_struct, 'special_yinds') && ~isempty(filt_struct.special_yinds)
    y_k(filt_struct.special_yinds) = observation_mean(filt_struct, Y_k(filt_struct.special_yinds, :));
end

%% 3. Calculate y-residuals.
%Used in observation covariance and state-observation cross-covariance for
%Kalman gain.
Y_k_r = bsxfun(@minus, Y_k, y_k);
if isfield(filt_struct, 'special_yinds') && ~isempty(filt_struct.special_yinds)
    for sp_ix = 1:size(Y_k_r, 2)
        Y_k_r(filt_struct.special_yinds, sp_ix) = obsdiff(Y_k(filt_struct.special_yinds, sp_ix), y_k(filt_struct.special_yinds));
    end
end

%% 4. Calculate observation sqrt covariance
[~, Sy_k] = qr((filt_struct.sqrtW(2)*Y_k_r(:,2:nsp))',0);       % QR update of observation error Cholesky factor.
%NOTE: here Sy is the UPPER Cholesky factor (Matlab excentricity)

% deal with possible negative zero'th covariance weight
if filt_struct.possitive_W3                                    
    Sy_k = cholupdate(Sy_k, filt_struct.sqrtW(3) * Y_k_r(:,1), '+');
else
    Sy_k = cholupdate(Sy_k, filt_struct.sqrtW(3) * Y_k_r(:,1), '-');
end
Sy_k = Sy_k';  % We need the lower triangular Cholesky factor
    
%% 5. Calculate Kalman Gain        
Pxy = filt_struct.W(3) * X_k_r(:,1) * Y_k_r(:,1)' + filt_struct.W(2) * X_k_r(:,2:nsp) * Y_k_r(:,2:nsp)';
KG = (Pxy/Sy_k')/Sy_k;

%% 6. Calculate innovation
innov = observation - y_k;
if isfield(filt_struct, 'special_yinds') && ~isempty(filt_struct.special_yinds)
    innov(filt_struct.special_yinds) = obsdiff(observation(filt_struct.special_yinds), y_k(filt_struct.special_yinds));
end

%% 7. State update
upd = KG * innov;
filt_struct.x = x_k + upd;
if isfield(filt_struct, 'special_xinds') && ~isempty(filt_struct.special_xinds)
    filt_struct.x(filt_struct.special_xinds) = filt_struct.special_state_add(x_k(filt_struct.special_xinds), upd(filt_struct.special_xinds));
end

%% 8. Update covariance
cov_update_vectors = KG * Sy_k;      % Correct covariance. This is equivalent to :  Px = Px_ - KG*Py*KG';
for j=1:filt_struct.Odim
    Sx_k = cholupdate(Sx_k, cov_update_vectors(:,j), '-');
end
filt_struct.S = Sx_k';
end

%% Helper functions - Not used because our observations have linear updates.
function obsmean = observation_mean(filt_struct, obs_sigmapoints)
end

function obd = obsdiff(obs1, obs2)
    obd = bsxfun(@minus, obs1, obs2);
end
