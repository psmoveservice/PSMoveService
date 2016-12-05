function quat = axisAngle2Quat(axisAngles)        
% Method of implementing this function that is accurate to numerical precision from
% Grassia, F. S. (1998). Practical parameterization of rotations using the exponential map. journal of graphics, gpu, and game tools, 3(3):29?48.
if size(axisAngles, 1) ~= 3
    if size(axisAngles, 2) == 3
        axisAngles = axisAngles';
    end
    %TODO: Else error.
end
theta = sqrt(sum(axisAngles.^2));  %norm
qw = cos(theta*0.5);
sing_bool = theta < 1000*eps;
na = nan(size(theta));
na(sing_bool) = 0.5 + (theta(sing_bool).^2) * (1.0/48.0);
na(~sing_bool) = sin(theta(~sing_bool)*0.5) ./ theta(~sing_bool);
axis = bsxfun(@times, axisAngles, na);
quat = [qw;axis];
qnorm = sqrt(sum(quat.^2));
quat = bsxfun(@rdivide, quat, qnorm);