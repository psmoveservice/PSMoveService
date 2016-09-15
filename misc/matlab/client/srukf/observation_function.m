function predicted_obs = observation_function(filt_struct, states, zR)
% states is a matrix of state vector columns.
% state vector: posx, velx, accx, posy, vely, accy, posz, velz, accz, qw, qx, qy, qz, avelx, avely, avelz
% Units: pos: m, vel: m/s, acc: m/s^2, orient. in quat, avel: rad/s
%
% Return value is a matrix of observation vector columns
% observation vector: a.x, a.y, a.z, g.x, g.y, g.z, m.x, m.y, m.z, pos.x, pos.y, pos.z
% Units: acc: g-units, gyro: rad/s, mag: B-units, pos: cm

%TODO: Check how srukf.m handles observation_noise

% Constants
GRAVITY = filt_struct.consts.GRAVITY;  % Depends on latitude.
gravity_world = [0;GRAVITY;0];
mag_world = filt_struct.consts.MAGFIELD;

nsp = size(states, 2);
predicted_obs = nan(filt_struct.Odim, nsp);  % Output

% From the state estimate of the orientation, get the world-to-controller
% transformation.
quat = states(10:13, :);

% Accelerometer = (linear acceleration + gravity) transformed to controller
% frame.
lacc_world = bsxfun(@plus, states([3 6 9], :), gravity_world);  % In m/sec^2
lacc_world_g = lacc_world ./ GRAVITY;  % In G units
predicted_obs(1:3, :) = rotateVector(quat, lacc_world_g);

% Gyroscope = angular velocity  (both rad/sec)
predicted_obs(4:6, :) = states([14 15 16], :);
    
% Magnetometer = world_mag_vector transformed to controller reference
% frame
predicted_obs(7:9, :) = rotateVector(quat, mag_world);

% Predicted optical tracker observation (m) = state position (m)
predicted_obs(10:12, :) = states([1 4 7], :);

% Any observation bias in zR.mu
predicted_obs = bsxfun(@plus, predicted_obs, zR.mu);

plus_ix = 1 + 2*filt_struct.Sdim + 2*filt_struct.Q.dim + (1:zR.dim);
minus_ix = plus_ix + zR.dim;
predicted_obs(:, plus_ix) = predicted_obs(:, plus_ix) + zR.cov;
predicted_obs(:, minus_ix) = predicted_obs(:, minus_ix) - zR.cov;
end