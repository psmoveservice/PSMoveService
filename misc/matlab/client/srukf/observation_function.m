function predicted_obs = observation_function(filt_struct, states, varargin)
%predicted_obs = observation_function(filt_struct, states, zR)
% states is a matrix of state vector columns.
%   State vector: posx, velx, accx, posy, vely, accy, posz, velz, accz, avelx, avely, avelz, qw, qx, qy, qz
%	Units: pos: m, vel: m/s, acc: m/s^2, avel: rad/s, orientation as quaternion
% Return value is a matrix of observation vector columns
%   Observation vector: [a.x, a.y, a.z, g.x, g.y, g.z, m.x, m.y, m.z, pos.x, pos.y, pos.z]
%	Units: acc: g-units, gyro: rad/s, mag: B-units, pos: m

if ~isempty(varargin)
    R = varargin{1};
else
    R = filt_struct.R;
end

%TODO: Check how srukf.m handles observation_noise

% Constants
GRAVITY = filt_struct.consts.GRAVITY;  % Depends on latitude.
gravity_world = [0;GRAVITY;0];
mag_world = filt_struct.consts.MAGFIELD;

nsp = size(states, 2);
predicted_obs = nan(filt_struct.Odim, nsp);  % Output

% From the state estimate of the orientation, get the world-to-controller
% transformation.
x_q = states(filt_struct.x_quat_ix, :);

% Accelerometer = (linear acceleration + gravity) transformed to controller
% frame.
lacc_world = bsxfun(@plus, states([3 6 9], :), gravity_world);  % In m/sec^2
lacc_world_g = lacc_world ./ GRAVITY;  % In G units
predicted_obs(1:3, :) = rotateVector(x_q, lacc_world_g);

% Gyroscope = angular velocity  (both rad/sec)
predicted_obs(4:6, :) = states(10:12, :);
    
% Magnetometer = world_mag_vector transformed to controller reference
% frame
predicted_obs(7:9, :) = rotateVector(x_q, mag_world);

% Predicted optical tracker observation (m) = state position (m)
predicted_obs(10:12, :) = states([1 4 7], :);

% Add observation bias in zR.mu to all sigma points
predicted_obs = bsxfun(@plus, predicted_obs, R.mu);

% Add observation covariance to last 2*R.dim columns.
predicted_obs(:, 1 + 2*filt_struct.S.dim + 2*filt_struct.Q.dim + 1 : end) =...
    predicted_obs(:, 1 + 2*filt_struct.S.dim + 2*filt_struct.Q.dim + 1 : end) + [R.zcov -R.zcov];
end