function output = mean_quat(quats, weights)
%Calculate mean for axang. Do weighted average quaternion.
%http://stackoverflow.com/a/27410865/1256069

wquats = bsxfun(@times, quats, weights);
neg_bool = weights < 0;
wquats(:, neg_bool) = bsxfun(@times, abs(weights(neg_bool)), quaternion_conjugate(quats(:, neg_bool)));
[eig_vecs, eig_vals] = eig(wquats * wquats');
[~, eig_ix] = max(diag(eig_vals));
q_out = eig_vecs(:, eig_ix);
q_out = q_out / norm(q_out);
output = quat2AxisAngle(q_out);

% TODO: As in srukf...
% Alternatively, calculate difference provided by each quaternion (i.e.,
% subtract quat in position 1), get the weighted sum of the deltas,
% add back to quat in position 1.

% Test both methods on non-propagated sigma points.