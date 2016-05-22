function [output_coeffs] = convertEllipseParameters(input_params)
% Convert parameters to easier form
% Assume input is in standard form
% a*x^2 + b*x*y + c*y^2 + d*x + f*y + g = 0
if length(input_params) == 6
    % Is conic
    output_coeffs = conic2parametric(input_params);
elseif length(input_params) == 5
    output_coeffs = parametric2conic(input_params);
end
end

function [parametric_coeffs] = conic2parametric(conic_params, varargin)

if nargin > 1
    convert_method = varargin{1};
else
    convert_method = 'wolfram';  % 'wolfram', 'cornell'
end

A = conic_params(1);
B = conic_params(2);
C = conic_params(3);
D = conic_params(4);
F = conic_params(5);
G = conic_params(6);

if strcmpi(convert_method, 'wolfram')
    % http://mathworld.wolfram.com/Ellipse.html
    % Eqns 19-23
    
    % Halve b, d, f to make the rest of the calculations slightly easier.
    B = B / 2;
    D = D / 2;
    F = F / 2;
    
    off_d = B^2 - A*C;
    h = (C*D - B*F) / off_d;
    k = (A*F - B*D) / off_d;
    
    semi_n = A*F^2 + C*D^2 + G*B^2 - 2*B*D*F - A*C*G;
    semi_d_2 = sqrt( (A-C)^2 + 4*B^2 );
    semi_d_3 = -1*(A+C);
    a = sqrt(2*semi_n / (off_d * (semi_d_2 + semi_d_3)));
    b = sqrt(2*semi_n / (off_d * (-1*semi_d_2 + semi_d_3)));
    clear off_d semi_n semi_d_2 semi_d_3
    
    % Counter-clockwise rotation
    if B == 0
        tau = 0;
    else
        tau = acot((A-C)/(2*B))/2;
    end
elseif strcmpi(convert_method, 'cornell')
    % https://www.cs.cornell.edu/cv/OtherPdf/Ellipse.pdf
    
    s = (4*A*C - B^2);
    h = (B*F - 2*C*D) / s;
    k = (B*D - 2*A*F) / s;
    tau = acot((A-C)/B) / 2;
    
    M0 = [...
         G  D/2 F/2;...
        D/2  A  B/2;...
        F/2 B/2  C ];
    M = [A B/2; B/2 C];
    lambda = eig(M);
    % TODO: sort lambda
    ab1 = -det(M0)/det(M);
    a = sqrt(ab1/lambda(1));
    b = sqrt(ab1/lambda(2));
    
end

if A > C
    tau = tau + (pi/2);
end

parametric_coeffs = [h, k, a, b, tau];
end


function [conic_params] = parametric2conic(parametric_params)
% https://www.cs.cornell.edu/cv/OtherPdf/Ellipse.pdf
h = parametric_params(1);
k = parametric_params(2);
a = parametric_params(3);
b = parametric_params(4);
tau = parametric_params(5);
c = cos(tau);
s = sin(tau);

A = (b*c)^2 + (a*s)^2;
B = -2*c*s*(a^2 - b^2);
C = (b*s)^2 + (a*c)^2;
D = -2*A*h - k*B;
F = -2*C*k - h*B;
G = -1*(a*b)^2 + A*(h^2) + B*h*k + C*(k^2);

conic_params = [A B C D F G];
end