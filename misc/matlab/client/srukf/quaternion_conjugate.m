function output = quaternion_conjugate(quat)
output = quat;
output(2:end, :) = -quat(2:end, :);