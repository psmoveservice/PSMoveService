function output = quaternion_multiply(q1, q0)
% q1 = bsxfun(@rdivide, q1, sqrt(sum(q1.^2, 1)));
% q0 = bsxfun(@rdivide, q0, sqrt(sum(q0.^2, 1)));
%Either q1 or q0 must be 4x1, or both must be 4xN
w0 = q0(1, :);
x0 = q0(2, :);
y0 = q0(3, :);
z0 = q0(4, :);
w1 = q1(1, :);
x1 = q1(2, :);
y1 = q1(3, :);
z1 = q1(4, :);


output = [...
    -x1.*x0 - y1.*y0 - z1.*z0 + w1.*w0;...
     x1.*w0 + y1.*z0 - z1.*y0 + w1.*x0;...
    -x1.*z0 + y1.*w0 + z1.*x0 + w1.*y0;...
     x1.*y0 - y1.*x0 + z1.*w0 + w1.*z0];