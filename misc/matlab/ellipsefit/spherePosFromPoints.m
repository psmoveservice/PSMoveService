function [sphere_pos_xyz] = spherePosFromPoints(x, y, varargin)
%[sphere_pos_xyz] = spherePosFromPoints(x, y)
%[sphere_pos_xyz] = spherePosFromPoints(x, y, radius)
%default radius is 2.25
%[sphere_pos_xyz] = spherePosFromPoints(x, y, radius, z_plane)
%default z_plane is -1

if nargin < 3
    radius = 2.25;
else
    radius = varargin{1};
end

if nargin < 4
    z_plane = -1;
else
    z_plane = varargin{2};
end

% Every point on the surface of the cone is a vector A_i
% A_i can be projected onto B = A_B
% This yields dot(A, B) = norm(A) * norm(B) * cos(theta)
% dot(A, B)/norm(A) = cos(theta)*norm(B)
% substitute c = cos(theta)*norm(B)
% dot(A, B)/norm(A) = c
% (Ax*Bx + Ay*By + Az*Bz) / norm(A) = c
% Find the sphere/ellipse on a specified z_plane
zz = z_plane*z_plane;
norm_A = sqrt(x.^2 + y.^2 + zz);
% Ax*Bx + Ay*By + zz = norm(A) * c
% Ax*Bx + Ay*By -norm(A)*c = -zz
A = [x y -norm_A];
b = -zz*ones(size(x));

Bx_By_c = A\b;  %pinv(A)*b
Bx = Bx_By_c(1);
By = Bx_By_c(2);
c = Bx_By_c(3);
% c = cos(theta)*norm(B)
norm_norm_B = sqrt(Bx.^2 + By.^2 + zz);
cos_theta = c / norm_norm_B;
% theta = acos(cos_theta);
% 
% sin(theta) = radius/norm_B;
% norm_B = radius / sin(theta);  %but theta = acos(cos_theta), so
norm_B = radius / sqrt(1 - cos_theta*cos_theta);

sphere_pos_xyz = [Bx By z_plane] * norm_B / norm_norm_B;