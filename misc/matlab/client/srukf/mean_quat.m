function output = mean_quat(quats, weights, varargin)
%Calculate weighted average quaternion.
%Two different solutions are available determined by string in 3rd argument
%'mean'     - (default) 
%'pivot'	- 
% TODO: Test both methods on non-propagated sigma points.
if isempty(varargin) || strcmpi(varargin{1}, 'mean')
    %http://stackoverflow.com/a/27410865/1256069
    
    %The input weights scale each column (by 1/L) so their
    %addition results in an average for easy math with linear variables.
    %But we can't add orientations, so we need to unscale weights.
    w = weights * size(quats, 2);
    
%     % From the paper in the SO link...
%     % Correct result but sign is arbitrary (q = -q)
%     M = w(1) * quats(:, 1) * quats(:, 1)';
%     for q_ix = 1:(size(quats, 2) - 1)
%         M = M + w(2) * quats(:, q_ix) * quats(:, q_ix)';
%     end
%     [eig_vecs, eig_vals] = eig(M);
%     [~, eig_ix] = max(diag(eig_vals));
%     q_out = eig_vecs(:, eig_ix);
    
    % From the SO link itself...
    wquats = [w(1)*quats(:, 1) w(2)*quats(:, 2:end)];
    [eig_vecs, eig_vals] = eig(wquats * wquats');
    [~, eig_ix] = max(diag(eig_vals));
    q_out = eig_vecs(:, eig_ix);
    
elseif strcmpi(varargin{1}, 'pivot')
    % Alternatively, as in ReBeL toolbox...
    % 1 - pick a pivot orientation, probably the first column.
    % 2 - Calculate the deviations from the pivot orientation in each column
    % 3 - convert to axang
    % 4 - scale and sum those. This doesn't work if the sum wraps.
    % 5 - convert back to quat
    % 6 - update pivot orientation with quaternion of average deviations
    delta_quats = quaternion_multiply(quats(:, 2:end), quaternion_conjugate(quats(:, 1)));
    delta_axang = sum(weights(2) * quat2AxisAngle(delta_quats), 2);
    delta_quat = axisAngle2Quat(delta_axang);
    q_out = quaternion_multiply(quats(:, 1), delta_quat);
end
output = q_out / norm(q_out);