%% Intro
% This script loads PSMoveService observational data, saved using
% python/examples/psmoveclient.py, and then applies a square-root unscented
% Kalman filter (SRUKF). The algorithm used is intended to follow a paper:
% [van der Merwe, Wan, Julier 2004](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.137.734&rep=rep1&type=pdf)
% The authors also have a Matlab toolbox called ReBeL that implements the
% algorithm (http://www.pdx.edu/biomedical-signal-processing-lab/signal-point-kalman-filters-and-the-rebel-toolkit)
% Some of the code here is based off that toolbox, with some
% simplifications and/or complications to accommodate our data.

%% Paths and Constants
addpath(genpath(fullfile(pwd, 'srukf')));

Xdim = 16;      % State vector: posx, velx, accx, posy, vely, accy, posz, velz, accz, avelx, avely, avelz, qw, qx, qy, qz
                % Units: pos: m, vel: m/s, acc: m/s^2, avel: rad/s, orientation as quaternion
x_lin_ix = 1:12;
x_quat_ix = 13:16;
Odim = 12;      % [a.x, a.y, a.z, g.x, g.y, g.z, m.x, m.y, m.z, pos.x, pos.y, pos.z]
                % Units: acc: g-units, gyro: rad/s, mag: B-units, pos: m
Sdim = 15;      % State covariance. Rotational covariance represented with angle-axis instead of quaternion.
                % posx, velx, accx, posy, vely, accy, posz, velz, accz, avelx, avely, avelz, angx, angy, angz
S_lin_ix = 1:12;
S_axang_ix = 13:15;
Qdim = 15;      % Process noise vector same as state covariance.
Q_lin_ix = S_lin_ix;
Q_axang_ix = S_axang_ix;
Rdim = Odim;	% Assume observation noise is same dimensionality as obs vec.
R_lin_ix = 1:Rdim;  % All linear
R_axang_ix = [];
alpha = 0.6;
beta = 2.0;
kappa = 3 - Xdim;
q_scale = 0.00001;	% Scale process noise
r_scale = 0.01;     % Scale observation noise.
s_scale = 0.001;    % Scale initial guess at state covariance.

%% Load data from csv
datadir = fullfile('..','..','test_data');
trainingdata = csvread(fullfile(datadir, 'stationary.csv'));
testdata = csvread(fullfile(datadir, 'movement.csv'));
%load columns are [a.x, a.y, a.z, g.x, g.y, g.z, m.x, m.y, m.z, pos.x, pos.y, pos.z, q.w, q.x, q.y, q.z, time]

% Scale positions from cm to m.
pos_ix = 10:12;
trainingdata(:, pos_ix) = 0.01 * trainingdata(:, pos_ix);
testdata(:, pos_ix) = 0.01 * testdata(:, pos_ix);

% Normalize magnetometer readings.
mag_ix = 7:9;
trainingdata(:, mag_ix) = bsxfun(@rdivide, trainingdata(:, mag_ix), sqrt(sum(trainingdata(:, mag_ix).^2, 2)));
testdata(:, mag_ix) = bsxfun(@rdivide, testdata(:, mag_ix), sqrt(sum(testdata(:, mag_ix).^2, 2)));

% PSMoveService default orientation is with the controller vertical, bulb
% to the sky, with the trigger to the camera. However, asking for the
% rotation from PSMoveState.Pose.Orientation uses the bulb facing the
% camera as the default orientation. We will use the provided orientations
% for testing, so let's undo their rotations first.
quat_ix = 13:16;
artificial_rotation = axisAngle2Quat([-pi/2 0 0]);  % -90 degrees about X
trainingdata(:, quat_ix) = quaternion_multiply(trainingdata(:, quat_ix)', artificial_rotation)';
testdata(:, quat_ix) = quaternion_multiply(testdata(:, quat_ix)', artificial_rotation)';
clear pos_ix mag_ix quat_ix artificial_rotation;

%% Initialize filter_struct

% Initialize state
% Assume zero initial velocity / acceleration / angular velocity.
% Cheat on initial orientation estimate using PSMoveService complementary filter.
quat_init = quaternion_multiply(testdata(1, 13:16)', axisAngle2Quat([-pi/2;0;0]))';
x_init = [...
    testdata(1,10), 0, 0,...	%p.x, v.x, a.x
    testdata(1,11), 0, 0,...	%p.y, v.y, a.y
    testdata(1,12), 0, 0,...	%p.z, v.z, a.z
    0, 0, 0,...,                % 0 initial angular velocity.
    quat_init]';
clear quat_init

% TODO: Initial guess at state (sqrt) covariance from trainingdata?
% State sqrt covariance is required to calculate state sigma points.
% This gets updated on every iteration so initial guess isn't critical.
S_init = struct(...
    'dim', Sdim,...
    'mu', zeros(Sdim, 1),...
    'cov', s_scale * eye(Sdim),...
    'lin_ix', S_lin_ix,...
    'axang_ix', S_axang_ix);
clear s_scale Sdim S_lin_ix S_axang_ix

% Prepare process noise
mean_dt = mean(diff(trainingdata(:, end)));
Q_cov = zeros(Qdim);
Q_3 = q_scale * process_noise(3, mean_dt);
Q_cov(1:3, 1:3) = Q_3;  %Lin-X
Q_cov(4:6, 4:6) = Q_3;  %Lin-Y
Q_cov(7:9, 7:9) = Q_3;  %Lin-Z
% The following is probably ok for angular velocity,
% but probably wrong for orientation
Q_2 = q_scale * process_noise(2, mean_dt);
Q_cov([13 10], [13 10]) = Q_2;  % angx, avelx
Q_cov([14 11], [14 11]) = Q_2;  % angy, avely
Q_cov([15 12], [15 12]) = Q_2;  % angz, avelz
% Q_cov(10:15, 10:15) = 0.01*q_scale*eye(6);  % Overwrite orientation/angvel process noise.
Q_init = struct(...
    'dim', Qdim,...
    'mu', zeros(Qdim, 1),...  % Process noise should be zero-mean, I think.
    'cov', chol(Q_cov, 'lower'),...% sqrt matrix
    'lin_ix', Q_lin_ix,...
    'axang_ix', Q_axang_ix);  
clear q_scale Q_cov Q_3 Q_2 Qdim Q_lin_ix Q_axang_ix

% Prepare observation noise from trainingdata
% Assume all observation variables are independent (no off-diagonals).
R_cov = zeros(Rdim);
R_cov(1:Rdim+1:end) = r_scale * diag(cov(trainingdata(:, 1:Odim)));
R_init = struct(...
    'dim', Rdim,...
    'mu', zeros(Rdim, 1),...
    'cov', sqrt(R_cov),...% Only diagonal so no need to chol.
    'lin_ix', R_lin_ix,...
    'axang_ix', R_axang_ix);  
clear r_scale R_cov Rdim R_lin_ix R_axang_ix
% R_init.mu(1:3) = [0.0133424997, 0.0107941628, 0.0543990135];  %accel_bias from json file
R_init.mu(1:3) = mean(trainingdata(:, 1:3)) - [0 1 0];  %accel_bias
R_init.mu(4:6) = mean(trainingdata(:, 4:6));

% % if doing process noise adaptation, save some variables for later
% ind1 = 1;
% ind2 = Xdim;
% paramdim = Xdim;
% dv = diag(Sv);

filt_struct = struct(...
    'Xdim', Xdim,...
    'Odim', Odim,...
    'alpha', alpha,...
    'beta', beta,...
    'kappa', kappa,...
    'x', x_init,...
    'x_lin_ix', x_lin_ix,...
    'x_quat_ix', x_quat_ix,...
    'S', S_init,...  % Lower-triangular Cholesky factor of state covariance
    'Q', Q_init,...  % Process noise. sqrt expected.
    'R', R_init,...  % Observation noise. sqrt expected.
    'last_obs_time', testdata(1, end) - mean_dt,...
    'intermediates', struct(...
        'X_k', [],...  % Sigma points propagated through process function to time k
        'x_k', [],...  % State estimate = weighted sum of sigma points
        'X_k_r', [],...% Propagated sigma point residuals = (sp - x_k)
        'Sx_k', []),...% Upper-triangular of propagated sp covariance
    'consts', struct(...
        'GRAVITY', 9.806,...  % Approx grav where recordings took place.
        'MAGFIELD', [0.234017432, 0.873125494, 0.42765367]'));  % Approx mag field where recordings took place.
clear Xdim Odim alpha beta kappa x_init S_init Q_init R_init mean_dt x_lin_ix x_quat_ix
%% Filter
%Pre-allocate output variables
observations = testdata(:, 1:filt_struct.Odim);
obs_times = testdata(:, end);
predicted_state = nan(size(observations, 1), filt_struct.Xdim);
o_ix = 1;
%%
for o_ix = 1:size(observations, 1)
    %%
    dt = obs_times(o_ix) - filt_struct.last_obs_time;
    filt_struct = srukf_predict(filt_struct, dt);
    filt_struct = srukf_update(filt_struct, observations(o_ix, :)');
    filt_struct.last_obs_time = observations(o_ix, end);
    
    %Save predicted state
    predicted_state(o_ix, :) = filt_struct.x;
    
    %     % if doing process noise adaptation
    %     if isfield(filt_struct.Q, 'adaptMethod')
    %         %--- update process noise if needed for joint estimation ----------------------
    %         switch filt_struct.Q.adaptMethod
    %             case 'anneal'
    %                 dv = sqrt(max(filt_struct.Q.adaptParams(1)*(dv.^2) , filt_struct.Q.adaptParams(2)));
    %                 Sv(ind1:ind2,ind1:ind2) = diag(dv);
    %
    %             case 'robbins-monro'
    %                 nu = 1/filt_struct.Q.adaptParams(1);
    %                 subKG = filt_struct.KG(end-paramdim+1:end,:);
    %                 dv = sqrt((1-nu)*(dv.^2) + nu*diag(subKG*(subKG*inov*inov')'));
    %                 Sv(ind1:ind2,ind1:ind2) = diag(dv);
    %                 filt_struct.Q.adaptParams(1) = min(filt_struct.Q.adaptParams(1)+1, filt_struct.Q.adaptParams(2));
    %             otherwise
    %                 error('Process noise update method not allowed.');
    %         end
    %         filt_struct.Q.cov = Sv;
    %     end
    
end
clear o_ix dt
%% Test observation function rotations
obs_ix = randi(size(trainingdata, 1));

% Sensor axes are +x = select->start; +y = USB->Bulb;
% +z=trigger->Move
% World axes, when looking at the camera: +x=left->right, +y=down->up,
% +z=far->near
% The two coordinate systems align when the controller is upright with the
% trigger facing the camera.
psm_quat = trainingdata(obs_ix, 13:16)';

% trainingdata recorded while controller was motionless.
% Therefore, accelerometer data should be due to gravity only.
% Controller was in default orientation.
% Therefore, gravity should mostly affect its y-axis.
accelerometer = trainingdata(obs_ix, 1:3)';
expected_accel = rotateVector(psm_quat, [0; 1; 0]);  %1 G = filt_struct.consts.GRAVITY
[accelerometer expected_accel]

%magnetometer data should be filt_struct.consts.MAGFIELD rotated to
%controller frame.
%rotateVector(quaternion_multiply([cosd(45) cosd(45) 0 0]', artificial_rotation), filt_struct.consts.MAGFIELD)
mag_identity = filt_struct.consts.MAGFIELD;
magnetometer = trainingdata(obs_ix, 7:9)';
expected_mag = rotateVector(psm_quat, mag_identity);
[magnetometer expected_mag]

%% Plot results
figure('Position', [0, 200, 1400, 600]);

% Plot the position
subplot(1,2,1)
pos_est = predicted_state(:, [1 4 7]);
pos_opt = testdata(:, 10:12);
plot3(pos_opt(:, 1), pos_opt(:, 3), pos_opt(:, 2), 'r.', 'MarkerSize', 25)
hold on
plot3(pos_est(:, 1), pos_est(:, 3), pos_est(:, 2), 'b', 'LineWidth', 3)
xlabel('x'); ylabel('z'); zlabel('y');
legend('Raw', 'Filtered')
set(gca, 'Color', 'none')
grid on
box on
axis equal
ax = gca;
ax.BoxStyle = 'full';

% Plot the rotation around the x-axis and y-axis
axang_psm = quat2AxisAngle(testdata(:, 13:16)')';
axang_est = quat2AxisAngle(predicted_state(:, 13:16)')';
for ax_ix = 1:2
    subplot(2, 2, 2*(ax_ix-1) + 2)
    plot(obs_times, axang_est(:,ax_ix), 'b', 'LineWidth', 3)
    hold on
    plot(obs_times, axang_psm(:,ax_ix), 'r.', 'MarkerSize', 5)
    axis tight
    set(gca, 'Color', 'none')
    xlabel('Time (s)')
    ylabel('rotation about axis')
end