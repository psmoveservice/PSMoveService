%% Define constants
R = 2.25;  % Sphere radius
F_PX = 554.2563;  % 75 deg diagonal fov, blue dot
% F_PX = 776.3782;  % 56 deg diagonal fov, red dot
% 75 diagonal degrees for about 800 diagonal pixels
% (Actually there are more than 640 x 480 pixels on the sensor)
% Is about 10 pixels per degree.
doc_ok = true;

%% Get parameters of ellipse that slices through known cone
% cone from camera focal point to sphere centre at B
% slice at z = F_PX
% Sphere position in 3D space (used to generate test data)
% B = [40, 30, 200];  % Further away so less resolution.
B = [18, 9, 30]; % Partially occluded by edge of camera FOV
true_conic_params = ellipseFromSphere(B, R, F_PX);
% a*x^2 + b*x*y + c*y^2 + d*x + f*y + g = 0
%Check b*b - 4*a*c < 0

%% Create an ellipse on the image
[truex, truey] = createEllipse(true_conic_params);
plot(truex, truey, 'LineWidth', 3)
set(gca, 'PlotBoxAspectRatio', [640 480 1])
hold on
plot([-320 320 320 -320 -320], [-240 -240 240 240 -240], 'k--')
xlim([-400 400]), ylim([-300 300])

% Clamp to -320:320; -240:240
invis_pixel = truex < -320 | truex > 320 | truey < -240 | truey > 240;
truex(invis_pixel) = [];
truey(invis_pixel) = [];

%% Get a noisy version of x,y
xn = truex;
yn = truey;
xn = xn + rand(size(xn)) - 0.5;
yn = yn + rand(size(yn)) - 0.5;
xn = round(xn);
yn = round(yn);
xy = unique([xn yn], 'rows');
xn = xy(:, 1);
yn = xy(:, 2);
xn = xn(1:floor(length(xn)/8):end);
yn = yn(1:floor(length(yn)/8):end);
scatter(xn, yn, 'o')
clear xy

%% Get the sphere position from the pixels

if doc_ok
    fitB = spherePosFromPoints(xn, yn, R, F_PX);  %Normalize pixel values
else
    
    %% Solve for the parameters of the noisy ellipse
    fit_conic_params = fitEllipse(xn, yn, 'LSqFit');
    % Sanity check returned parameters
    if length(fit_conic_params) == 5
        num2str([convertEllipseParameters(true_conic_params); fit_conic_params])
    else
        num2str([true_conic_params/(true_conic_params(1)); fit_conic_params/fit_conic_params(1)])
    end
    [fitx, fity] = createEllipse(fit_conic_params);
    plot(fitx, fity, 'LineWidth', 3)
    
    %% Recover our sphere position from these parameters
    test_params = fit_conic_params;
    % test_params = true_conic_params/true_conic_params(1);
    fitB = sphereFromEllipse(test_params, R, F_PX, 'Parametric');
end
num2str([B; fitB])