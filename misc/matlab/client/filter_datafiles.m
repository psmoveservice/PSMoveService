%% Paths and Constants
addpath(genpath(fullfile(pwd, 'srukf')));

Xdim = 15;  % posx, velx, accx, posy, vely, accy, posz, velz, accz, angx, avelx, angy, avely, angz, avelz
special_xinds = [10 12 14];  % angx, angy, angz cannot be added or subtracted
special_state_add = @add_axang;
special_state_sub = @sub_axang;
special_state_mean = @mean_axang;
Odim = 12;  % [a.x, a.y, a.z, g.x, g.y, g.z, m.x, m.y, m.z, pos.x, pos.y, pos.z]
alpha = 0.1;
beta = 2.0;
kappa = 3 - Xdim;
Qdim = Xdim;  % Assume process noise is same dimensionality as state vector
Rdim = Odim;  % Assume observation noise is same dimensionality as obs vec.
q_scale = 0.1;  %Scale process noise
r_scale = 1.0;

%% Load data from csv
datadir = fullfile('..','..','test_data');
trainingdata = csvread(fullfile(datadir, 'stationary.csv'));
testdata = csvread(fullfile(datadir, 'movement.csv'));
%load columns are [a.x, a.y, a.z, g.x, g.y, g.z, m.x, m.y, m.z, pos.x,
%pos.y, pos.z, q.w, q.x, q.y, q.z, time]

%% Initialize filter_struct

% Initialize state
% Assume zero initial velocity.
axang = quat2AxisAngle(testdata(1, [13 14 15 16])');
x_init = [...
    testdata(1,10), 0, 0,...  %p.x, v.x, a.x
    testdata(1,11), 0, 0,...  %p.y, v.y, a.y
    testdata(1,12), 0, 0,...  %p.z, v.z, a.z
    axang(1), 0,...
    axang(2), 0,...
    axang(3), 0,]';
clear axang

% TODO: Initial guess at state covariance from trainingdata?
S_init = zeros(Xdim);
S_init(1:Xdim+1:end) = 0.1;  % Along the diagonal

% Prepare process noise
mean_dt = mean(diff(trainingdata(:, end)));
Q_cov = zeros(Qdim);
Q_3 = q_scale * Q_discrete_white_noise(3, mean_dt);
Q_cov(1:3, 1:3) = Q_3;  %Lin-X
Q_cov(4:6, 4:6) = Q_3;  %Lin-Y
Q_cov(7:9, 7:9) = Q_3;  %Lin-Z
Q_2 = q_scale * Q_discrete_white_noise(2, mean_dt);
Q_cov(10:11, 10:11) = Q_2;  % Ang-X
Q_cov(12:13, 12:13) = Q_2;  % Ang-Y
Q_cov(14:15, 14:15) = Q_2;  % Ang-Z
Q_init = struct(...
    'dim', Qdim,...
    'mu', zeros(Qdim, 1),...  % Process noise should be zero-mean, I think.
    'cov', sqrt(Q_cov));  % TODO: sqrt matrix with chol and '?
clear Q_cov Q_3 Q_2

% Prepare observation noise from trainingdata
% Assume all observation variables are independent.
R_cov = zeros(Rdim);
R_cov(1:Rdim+1:end) = r_scale * diag(cov(trainingdata(:, 1:Odim)));
R_init = struct(...
    'dim', Rdim,...
    'mu', zeros(Rdim, 1),...  %TODO: Change elements to non-zero for bias.
    'cov', sqrt(R_cov));
clear R_cov

% % if doing process noise adaptation, save some variables for later
% ind1 = 1;
% ind2 = Xdim;
% paramdim = Xdim;
% dv = diag(Sv);

filt_struct = struct(...
    'Xdim', Xdim,...  
    'special_xinds', special_xinds,...
    'special_state_add', special_state_add,...
    'special_state_sub', special_state_sub,...
    'special_state_mean', special_state_mean,...
    'Odim', Odim,...  
    'alpha', alpha,...
    'beta', beta,...
    'kappa', kappa,...
    'x', x_init,...
    'S', S_init,...  % Lower-triangular Cholesky factor of state covariance
    'Q', Q_init,...  % Process noise
    'R', R_init,...  % Observation noise
    'last_obs_time', testdata(1, end) - mean_dt,...
    'intermediates', struct(...
        'X_k', [],...
        'X_k_r', [],...
        'x_k', [],...
        'Sx_k', []),...
    'consts', struct(...
        'GRAVITY', 9.806,...  % Approx grav where recordings took place.
        'MAGFIELD', [0.737549126; 0.675293505; 1]));  % Approx mag field where recordings took place.


%% Filter
%Pre-allocate output variables
observations = testdata(:, 1:Odim);
predicted_state = nan(size(observations, 1), Xdim);
o_ix = 1;
%%
for o_ix = 1:size(observations, 1)
    %%
    dt = observations(o_ix, end) - filt_struct.last_obs_time;
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