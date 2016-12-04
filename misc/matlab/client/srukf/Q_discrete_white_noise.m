function [Q] = Q_discrete_white_noise(dim, dt)
%[Q] = Q_discrete_white_noise(dim, dt)
%Returns the Q matrix for the Discrete Constant White Noise Model.
%dim may be either 2 or 3
%dt is the time step
%
%Q is computed as the G * G^T variance, where G is the process noise per
%time step. In other words, G = [[.5dt^2][dt]]^T for the constant velocity
%model.
    
Q = nan(dim);
if dim == 2
    Q = [0.25*dt^4 .5*dt^3; .5*dt^3 dt^2];
elseif dim == 3
    Q = [0.25*dt^4 .5*dt^3 .5*dt^2; .5*dt^3 dt^2 dt; .5*dt^2 dt 1];
end
end