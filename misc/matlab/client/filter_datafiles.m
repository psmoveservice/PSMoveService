%% Paths and Constants
addpath(genpath(fullfile(pwd, 'srukf')));

Xdim = 16;  % State vector: posx, velx, accx, posy, vely, accy, posz, velz, accz, qw, qx, qy, qz, avelx, avely, avelz
Sdim = 15;  % State covariance. Rotational covariance represented with angle-axis instead of quaternion. See Qdim below.
Odim = 12;  % [a.x, a.y, a.z, g.x, g.y, g.z, m.x, m.y, m.z, pos.x, pos.y, pos.z]
alpha = 0.6;
beta = 2.0;
kappa = 3 - Xdim;
Qdim = 15;  % Process noise vector: posx, velx, accx, posy, vely, accy, posz, velz, accz, angx, avelx, angy, avely, angz, avelz
Rdim = Odim;  % Assume observation noise is same dimensionality as obs vec.
q_scale = 1;  % Scale process noise
r_scale = 10;  % Scale observation noise.

%% Load data from csv
datadir = fullfile('..','..','test_data');
trainingdata = csvread(fullfile(datadir, 'stationary.csv'));
testdata = csvread(fullfile(datadir, 'movement.csv'));
%load columns are [a.x, a.y, a.z, g.x, g.y, g.z, m.x, m.y, m.z, pos.x,
%pos.y, pos.z, q.w, q.x, q.y, q.z, time]

%% Initialize filter_struct

% Initialize state
% Assume zero initial velocity / acceleration / angular velocity.
x_init = [...
    testdata(1,10), 0, 0,...  %p.x, v.x, a.x
    testdata(1,11), 0, 0,...  %p.y, v.y, a.y
    testdata(1,12), 0, 0,...  %p.z, v.z, a.z
    testdata(1,[13 14 15 16]),...,
    0, 0, 0]';

% TODO: Initial guess at state (sqrt) covariance from trainingdata?
% State sqrt covariance is required to calculate state sigma points.
% This gets updated on every iteration so initial guess isn't critical.
S_init = zeros(Sdim);
S_init(1:Sdim+1:end) = 0.1;  % Along the diagonal

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
Q_cov(10:11, 10:11) = Q_2;  % Ang-X
Q_cov(12:13, 12:13) = Q_2;  % Ang-Y
Q_cov(14:15, 14:15) = Q_2;  % Ang-Z
% Q_cov([10 12 14], [10 12 14]) = 0.1;  %Overwrite ax-angle process noise.
Q_init = struct(...
    'dim', Qdim,...
    'mu', zeros(Qdim, 1),...  % Process noise should be zero-mean, I think.
    'cov', chol(Q_cov, 'lower'));  % sqrt matrix
clear Q_cov Q_3 Q_2
% TODO: Check that the axis-angle noise wraps correctly.

% Prepare observation noise from trainingdata
% Assume all observation variables are independent.
R_cov = zeros(Rdim);
R_cov(1:Rdim+1:end) = r_scale * diag(cov(trainingdata(:, 1:Odim)));
R_init = struct(...
    'dim', Rdim,...
    'mu', zeros(Rdim, 1),...  %TODO: Change elements to non-zero for bias.
    'cov', sqrt(R_cov));  % Only diagonal so no need to chol.
clear R_cov

% % if doing process noise adaptation, save some variables for later
% ind1 = 1;
% ind2 = Xdim;
% paramdim = Xdim;
% dv = diag(Sv);

filt_struct = struct(...
    'Xdim', Xdim,...  
    'Sdim', Sdim,...
    'Odim', Odim,...  
    'alpha', alpha,...
    'beta', beta,...
    'kappa', kappa,...
    'x', x_init,...
    'S', S_init,...  % Lower-triangular Cholesky factor of state covariance
    'Q', Q_init,...  % Process noise. sqrt expected.
    'R', R_init,...  % Observation noise. sqrt expected.
    'last_obs_time', testdata(1, end) - mean_dt,...
    'intermediates', struct(...
        'X_t', [],...  % Sigma points at time t = k-1
        'X_k', [],...  % Sigma points propagated through process function to time k
        'x_k', [],...  % State estimate = weighted sum of sigma points
        'X_k_r', [],...% Propagated sigma point residuals = (sp - x_k)
        'Sx_k', []),...% Upper-triangular of propagated sp covariance
    'consts', struct(...
        'GRAVITY', 9.806,...  % Approx grav where recordings took place.
        'MAGFIELD', [0.737549126; 0.675293505; 1]));  % Approx mag field where recordings took place.

%% Filter
%Pre-allocate output variables
observations = testdata(:, 1:Odim);
obs_times = testdata(:, end);
predicted_state = nan(size(observations, 1), Xdim);
o_ix = 1;
%%
for o_ix = 1:size(observations, 1)
    %%
    dt = obs_times(o_ix) - filt_struct.last_obs_time;
    filt_struct = srukf_predict(filt_struct, dt);
    filt_struct = srukf_update(filt_struct, observations(o_ix, 1:Odim)');
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

%% Plot results

figure;

subplot(2,1,1)
pos_est = predicted_state(:, [1 4 7]);
pos_opt = testdata(:, 10:12);
plot3(pos_opt(:, 1), pos_opt(:, 2), pos_opt(:, 3), 'r.', 'MarkerSize', 25)
hold on
plot3(pos_est(:, 1), pos_est(:, 2), pos_est(:, 3), 'b', 'LineWidth', 3)
legend('Raw', 'Filtered')
set(gca, 'Color', 'none')
grid on
box on
ax = gca;
ax.BoxStyle = 'full';

subplot(2,1,2)
axang_compl = quat2AxisAngle(testdata(:, 13:16)')';
axang_est = quat2AxisAngle(predicted_state(:, 10:13)')';
plot(axang_est, 'b', 'LineWidth', 3)
hold on
plot(axang_compl, '.', 'MarkerSize', 25)
axis tight
set(gca, 'Color', 'none')