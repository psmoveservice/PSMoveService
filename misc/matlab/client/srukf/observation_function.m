function predicted_obs = observation_function(filt_struct, state_vector, R)
    
GRAVITY = filt_struct.consts.GRAVITY;  % Depends on latitude.
gravity_world = [0;0;-GRAVITY];
mag_world = filt_struct.consts.MAGFIELD;

predicted_obs = nan(size(R.mu));
% Sigma point:
% posx, velx, accx, posy, vely, accy, posz, velz, accz, angx, avelx, angy, avely, angz, avelz
% Observation
% [a.x, a.y, a.z, g.x, g.y, g.z, m.x, m.y, m.z, pos.x, pos.y, pos.z]

% From the state estimate of the orientation, get the world-to-controller
% transformation.
quat = axisAngle2Quat(state_vector([10, 12, 14]));

% Accelerometer = (linear acceleration + gravity) transformed to controller
% frame.
lacc_world = state_vector([2 5 8]) + gravity_world;  % In m/sec^2
lacc_world_g = lacc_world ./ GRAVITY;  % In G units
predicted_obs(1:3) = rotateAxisAngle(lacc_world_g, quat);

% Gyroscope = angular velocity
predicted_obs(4:6) = state_vector([11 13 15]);
    
% Magnetometer = world_mag_vector transformed to controller reference
% frame
predicted_obs(7:9) = rotateAxisAngle(mag_world, quat);

% Predicted optical tracker position = state position
predicted_obs(10:12) = state_vector([1 4 7]);

predicted_obs = predicted_obs + R.mu;