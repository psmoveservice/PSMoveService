function [fit_conic_params] = fitEllipse(x, y, varargin)

if nargin > 2
    fitMethod = varargin{1};
else
    fitMethod = 'LSqFit';
end



if strcmpi(fitMethod, 'LSqFit')
%     % http://cseweb.ucsd.edu/~mdailey/Face-Coord/ellipse-specific-fitting.pdf
%     % Build design matrix
%     D = [ xn.*xn xn.*yn yn.*yn xn yn ones(size(xn)) ];
%     % Build scatter matrix
%     S = D'*D;
%     % Build 6x6 constraint matrix
%     C(6,6) = 0; C(1,3) = 2; C(2,2) = -1; C(3,1) = 2;
%     % Solve eigensystem
%     [gevec, geval] = eig(S\C);  % eig(inv(S)*C)
%     % Find the positive eigenvalue
%     [~, PosC] = find(geval > 0 & ~isinf(geval));
%     % Extract eigenvector corresponding to positive eigenvalue
%     params = gevec(:,PosC);

    %http://autotrace.sourceforge.net/WSCG98.pdf
    D1 = [x.^2, x.*y, y.^2]; % quadratic part of the design matrix
    D2 = [x, y, ones(size(x))]; % linear part of the design matrix
    S1 = D1' * D1; % quadratic part of the scatter matrix
    S2 = D1' * D2; % combined part of the scatter matrix
    S3 = D2' * D2; % linear part of the scatter matrix
    T = - inv(S3) * S2'; % -S3\S2' for getting a2 from a1
    M = S1 + S2 * T; % reduced scatter matrix
    M = [M(3, :) ./ 2; - M(2, :); M(1, :) ./ 2]; % premultiply by inv(C1)
    [evec, ~] = eig(M); % solve eigensystem
    cond = 4 * evec(1, :) .* evec(3, :) - evec(2, :).^2; % evaluate a?Ca
    a1 = evec(:, cond > 0); % eigenvector for min. pos. eigenvalue
    fit_conic_params = [a1; T * a1]'; % ellipse coefficients
    
elseif strcmpi(fitMethod, 'MinArea')
    %http://stackoverflow.com/questions/1768197/bounding-ellipse
    tolerance = 0.01;
    P = [x';y'];
    N = length(x);
    d = 2;
    Q = [P; ones(1,N)]; % Add a row of 1s to the 2xN matrix P - so Q is 3xN now.
    
    % Initialize
    count = 1;
    err = 1;
    %u is an Nx1 vector where each element is 1/N
    u = (1/N) * ones(N,1);
    
    % Khachiyan Algorithm
    while err > tolerance
        X = Q*diag(u)*Q';
        M = diag(Q' * inv(X) * Q);
        
        % Find the value and location of the maximum element in the vector M
        [mmax, j] = max(M);
        
        % Calculate the step size for the ascent
        step_size = (mmax-d-1) / ((d+1) * (mmax-1));
        
        % Calculate the new_u:
        % Take the vector u, and multiply all the elements in it by (1-step_size)
        new_u = (1 - step_size)*u;
        
        % Increment the jth element of new_u by step_size
        new_u(j) = new_u(j) + step_size;
        
        err = norm(new_u - u);
        
        % Increment count and replace u
        count = count + 1;
        u = new_u;
    end
    
    U = diag(u);
    
    % Compute the A-matrix
    A = (1/d) * inv(P * U * P' - (P * u)*(P*u)' );
    
    % And the center,
    c = P * u;
    h = c(1);
    k = c(2);
    semi_axes = 1 ./ sqrt(eig(A));
    a = max(semi_axes);
    b = min(semi_axes);
    
    B = 2*A(1, 2);
    C = A(2, 2);
    A = A(1, 1);
    tau = acot((A-C)/B) / 2;
    
    if A > C
        tau = tau + (pi/2);
    end
    
    fit_conic_params = [h k a b tau];

%     s = (4*A*C - B^2);
%     %h = (B*F - 2*C*D) / s;
%     %k = (B*D - 2*A*F) / s;
%     
%     %F = (h*s + 2*C*D)/B
%     %k*s = B*D - 2*A*((h*s + 2*C*D)/B)
%     %k*s = B*D - 2*A*h*s/B - 4*A*C*D/B
%     %k*s + 2*A*h*s/B = D(B - 4*A*C/B) 
%     %B*k*s + 2*A*h*s = -s*D
%     D = -2*A*h - B*k;
%     F = (h*s + 2*C*D)/B;
% TODO: Calculate G
    
end