function filt_struct = srukf_update(filt_struct, observation)
X_t = filt_struct.intermediates.X_t;
X_k = filt_struct.intermediates.X_k;
X_k_r = filt_struct.intermediates.X_k_r;
x_k = filt_struct.intermediates.x_k;
Sx_k = filt_struct.intermediates.Sx_k;  % Still UPPER

%% 1. Propagate sigma points through observation function.
zR = filt_struct.R;
zR.cov = filt_struct.weights.zeta * zR.cov;
zR.observation = observation; % Just for debugging in observation function.
Y_k = observation_function(filt_struct, X_k, zR);

%% 2. Calculate observation mean.
wm = filt_struct.weights.wm;
y_k = sum([wm(1) * Y_k(:, 1), wm(2) * Y_k(:, 2:end)], 2);

%% 3. Calculate y-residuals.
%Used in observation covariance and state-observation cross-covariance for
%Kalman gain.
Y_k_r = bsxfun(@minus, Y_k, y_k);

%% 4. Calculate observation sqrt covariance
Sy_k = triu(qr(filt_struct.weights.w_qr*Y_k_r(:,2:end)', 0));
Sy_k = Sy_k(1:filt_struct.Odim, 1:filt_struct.Odim);
%NOTE: here Sy is the UPPER Cholesky factor (Matlab excentricity)
% deal with possible negative zero'th covariance weight
if filt_struct.weights.wc(1) > 0
    Sy_k = cholupdate(Sy_k, filt_struct.weights.w_cholup*Y_k_r(:,1), '+');
else
    % deal with possible negative zero'th covariance weight
    Sy_k = cholupdate(Sy_k, filt_struct.weights.w_cholup*Y_k_r(:,1), '-');
end
Sy_k = Sy_k';  % We need the LOWER triangular Cholesky factor
    
%% 5. Calculate Kalman Gain
wc = filt_struct.weights.wc;
%First calculate state-observation cross (sqrt) covariance
Pxy = wc(1) * X_k_r(:,1) * Y_k_r(:,1)' + wc(2) * X_k_r(:,2:end) * Y_k_r(:,2:end)';  
KG = (Pxy/Sy_k')/Sy_k;

%% 6. Calculate innovation
innov = observation - y_k;

%% 7. State update / correct
upd = KG * innov; %ReBeL srukf doesn't do anything special for angles to get upd.

% Update linear parts
x_lin = [1:9 14 15 16];  % pos, vel, accel, avel
s_lin = [1:9 11 13 15];  % pos, vel, accel, avel
filt_struct.x(x_lin) = x_k(x_lin) + upd(s_lin);

x_qinds = 10:13;
s_ang = [10 12 14];
filt_struct.x(x_qinds) = quaternion_multiply(x_k(x_qinds), axisAngle2Quat(upd(s_ang)));

%% 8. Covariance update / correct
%This is equivalent to :  Px = Px_ - KG*Py*KG';
cov_update_vectors = KG * Sy_k;
for j=1:filt_struct.Odim
    Sx_k = cholupdate(Sx_k, cov_update_vectors(:,j), '-');  % Still UPPER
end
filt_struct.S = Sx_k';  % LOWER sqrt-covariance saved for next predict.
end

%% Helper functions - Not used because our observations have linear updates.
function obsmean = observation_mean(filt_struct, obs_sigmapoints)
end

function obd = obsdiff(obs1, obs2)
    obd = bsxfun(@minus, obs1, obs2);
end
