function [x, y] = createEllipse(input_params)
if length(input_params) == 6
    input_params = convertEllipseParameters(input_params);
end
h = input_params(1);
k = input_params(2);
a = input_params(3);
b = input_params(4);
tau = input_params(5);

t = -pi:pi/32:pi;

% First, create ellipse centered at the origin.
x = a * cos(t);
y = b * sin(t);

% Rotate ellipse
R = [cos(tau) -sin(tau); sin(tau) cos(tau)];
xy = R* [x;y];

% Offset ellipse
x = xy(1, :)' + h;
y = xy(2, :)' + k;

% % Or all in one step
% c = cos(tau);
% s = sin(tau);
% ct = cos(t);
% st = sin(t);
% x = h + c*a*ct - s*b*st;
% y = k + s*a*ct - c*b*st;