function [fitB] = sphereFromEllipse(input_params, R, F_PX, varargin)

if nargin > 3
    use_method = varargin{1};
else
    use_method = 'Parametric';
end

if strcmpi(use_method, 'Parametric')
    % Old method
    %https://github.com/cboulay/PSMoveService/wiki/Optical-Tracker-Algorithms#analytic-solution-after-ellipse-fitting
    fitB = nan(1, 3);
    if length(input_params) == 6
        input_params = convertEllipseParameters(input_params);
    end
    
    h = input_params(1);
    k = input_params(2);
    a = input_params(3);
    % b = input_params(4);
    % tau = input_params(5);
    
    L_px = sqrt(h*h + k*k);
    m = L_px / F_PX;
    fl = F_PX / L_px;
    j = (L_px + a) / F_PX;
    l = (j - m) / (1 + j*m);
    D_cm = R * sqrt(1 + l*l) / l;
    fitB(3) = D_cm * fl / sqrt( 1 + fl*fl);
    L_cm = fitB(3) * m;
    fitB(1) = L_cm * h / L_px;
    fitB(2) = L_cm * k / L_px;
    clear L_px k j l D_cm fl L_cm
    
elseif strcmpi(use_method, 'Conic')
    % New Method
    % https://github.com/cboulay/PSMoveService/wiki/Optical-Tracker-Algorithms#cone-fitting
    if length(input_params) == 5
        input_params = convertEllipseParameters(input_params);
    end
    
    if sign(input_params(4)*input_params(5)/input_params(2)) < 0
        input_params = -input_params;
    end

    A = input_params(1);
    B = input_params(2);
    C = input_params(3);
    D = input_params(4);
    F = input_params(5);
    G = input_params(6);
    
    % We know the following equations from ellipseFromSphere are correct
    % k = cos(theta)^2
    % m = k*(Bx^2 + By^2 + Bz^2);
    % A = Bx^2 - m              (cm^2)
    % B = 2*Bx*By               (cm^2)
    % C = By^2 - m              (cm^2)
    % D = 2*Bx*Bz*F_PX          (cm^2*px)
    % F = 2*By*Bz*F_PX          (cm^2*px)
    % G = Bz^2*F_PX^2 - m*F_PX^2 = F_PX^2 * (Bz^z - m)  (cm^2*px^2)
    %
    % Given A B C D F G, solve for Bx, By, Bz
    % (use x,y,z instead of Bx,By,Bz)
    %
    % [from F] y = F / (2 * z * F_PX)
    % [from B] x = B / (2*y) = B / (2 * (F / (2 * z * F_PX)) = B * z * F_PX / F
    % [from D] z = D / (2 * x * F_PX) = D * F / (2*B*F_PX^2*z)
    f_sq = F_PX * F_PX;
    z_sq = D * F / (2*B*f_sq);
    z = sqrt(z_sq);  % Assume positive Z direction
%     x = B * z * F_PX / F;
%     y = F / (2 * z * F_PX);    
    twofz = 2*z*F_PX;
    x = D / twofz;
    y = F / twofz;
    
    
    % Check [x, y, z]
%     figure;
%     B = [x y z];
%     B = B / norm(B);
%     plot3([0 B(1)], [0 B(2)], [0 B(3)])
%     hold on
%     B = [16 12 30];
%     B = B/norm(B);
%     plot3([0 B(1)], [0 B(2)], [0 B(3)])
    
    
    % This x,y,z is the wrong length. Get theta and use R to get the correct
    % length.
    x_sq = x*x;
    y_sq = y*y;
    m = z_sq - G/f_sq;
%     m = x_sq - A;
%     m = y_sq - C;
    
    
    ssq = x_sq + y_sq + z_sq;
    k = m / ssq;
    % theta_rad = acos(sqrt(k));
    % norm_B = R / sin(new_theta_rad);
    norm_B = R / sqrt(1 - k);  % Using trig identity sin(acos(x)) = sqrt(1-x^2)
    fitB = [x y z] * norm_B / sqrt(ssq);
end