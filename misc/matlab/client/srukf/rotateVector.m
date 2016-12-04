function output = rotateVector(quat, vec)
q = axisAngle2Quat(vec);
rconj = quaternion_conjugate(quat);
output = quat2AxisAngle(quaternion_multiply(quat, quaternion_multiply(q, rconj)));

% Below is taken from Matlab man page on quatrotate, but it doesn't work
% for me.
% nquat = size(quat, 2);
% nvec = size(vec, 2);
% output = nan(3, nvec);
% for v_ix = 1:nvec
%     if nquat == nvec
%         q_ix = v_ix;
%     else
%         q_ix = 1;
%     end
%     qq = quat(:, q_ix) * quat(:, q_ix)';
%     rotMat = 2 * [...
%         0.5 - qq(3,3) - qq(4,4)     qq(2,3) + qq(1,4)           qq(2,4) - qq(1,3);...
%         qq(2,3) - qq(1,4)           0.5 - qq(2,2) - qq(4,4)     qq(3,4) + qq(1,2);...
%         qq(2,4) + qq(1,3)           qq(3,4) - qq(1,2)           0.5 - qq(2,2) - qq(3,3)];
%     output(:, v_ix) = rotMat * vec(:, v_ix);
% end
% output = rotMat * vec(:,1);