function output = mean_axang(axangs, weights)
%Calculate mean for axang
quats = axisAngle2Quat(axangs);
full_weights = [weights(1) ones(1, size(axangs, 2)-1) * weights(2)];
wquats = bsxfun(@times, quats, full_weights);
[eig_vecs, eig_vals] = eig(wquats * wquats');
[~, eig_ix] = max(diag(eig_vals));
q_out = eig_vecs(:, eig_ix);
q_out = q_out / norm(q_out);
output = quat2AxisAngle(q_out);