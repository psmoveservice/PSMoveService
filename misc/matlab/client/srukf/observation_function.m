function predicted_obs = observation_function(filt_struct, states, observation_noise)

%TODO: Check how srukf handles observation_noise
    
GRAVITY = filt_struct.consts.GRAVITY;  % Depends on latitude.
gravity_world = [0;0;-GRAVITY];
mag_world = filt_struct.consts.MAGFIELD;

n_obs = size(states, 2);
predicted_obs = nan(filt_struct.Odim, n_obs);
% Sigma point:
% posx, velx, accx, posy, vely, accy, posz, velz, accz, angx, avelx, angy, avely, angz, avelz
% Observation
% [a.x, a.y, a.z, g.x, g.y, g.z, m.x, m.y, m.z, pos.x, pos.y, pos.z]

% From the state estimate of the orientation, get the world-to-controller
% transformation.
quat = axisAngle2Quat(states([10, 12, 14], :));

% Accelerometer = (linear acceleration + gravity) transformed to controller
% frame.
lacc_world = bsxfun(@plus, states([3 6 9], :), gravity_world);  % In m/sec^2
lacc_world_g = lacc_world ./ GRAVITY;  % In G units
predicted_obs(1:3, :) = rotateAxisAngle(lacc_world_g, quat);

% Gyroscope = angular velocity
predicted_obs(4:6, :) = states([11 13 15], :);
    
% Magnetometer = world_mag_vector transformed to controller reference
% frame
predicted_obs(7:9, :) = rotateAxisAngle(mag_world, quat);

% Predicted optical tracker position = state position
predicted_obs(10:12, :) = states([1 4 7], :);

if ~isempty(observation_noise)
    predicted_obs = predicted_obs + observation_noise;
end