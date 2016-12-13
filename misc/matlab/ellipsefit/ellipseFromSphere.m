function [conic_params] = ellipseFromSphere(B, R, z_pl)
% function [a, b, c, d, f, g] = ellipseFromSphere(B, R, z_pl)
% B is the 3D coordinate of the sphere
% R is the radius of the sphere
% z_pl is the z-depth of the intersecting plane.
%
% Every point on the surface of the cone is a vector A_i
% A_i can be projected onto B = A_B
% This yields dot(A, B) = norm(A) * norm(B_u) * cos(theta)
% (A.*B)^2 - (B.*B)*(A.*A)(cos(theta))^2 = 0
% (Ax*Bx + Ay*By + Az*Bz)^2 - (Bx^2 + By^2 + Bz^2)(Ax^2 + Ay^2 + Az^2)(cos(theta)^2) = 0
%
% Let k = cos(theta)^2
%
% Expand terms:
% Ax^2*Bx^2 + Ay^2*By^2 + Az^2*Bz^2 + 2*Ax*Bx*Ay*By + 2*Ax*Bx*z_pl*Bz +
% 2*Ay*By*z_pl*Bz - k*(Bx^2+By^2+Bz^2)*Ax^2 - k*(Bx^2+By^2+Bz^2)*Ay^2 -
% k*(Bx^2+By^2+Bz^2)*z_pl^2 = 0
%
% Collect terms:
% a: (Bx^2 - k*(Bx^2 + By^2 + Bz^2))*Ax^2  units: cm^2
% b: (2*Bx*By)*(Ax*Ay) units: cm^2
% c: (By^2 - k*(Bx^2 + By^2 + Bz^2))*Ay^2 units: cm^2
% d: (2*Bx*Bz*z_pl)*Ax units: cm^2_px
% f: (2*By*Bz*z_pl)*Ay units: cm^2_px
% g: (Bz^2*z_pl^2 - k*(Bx^2+By^2+Bz^2)*z_pl^2) units: units: cm^2_px^2

% theta_rad = asin(R/norm(B));  % visual angle for half the sphere (half cone aperture)
% k = (cos(theta_rad))^2;
k = 1 - (R/norm(B)).^2;

% There are a couple normalizations we can do to make the algebra easier.
% 1 - Make B a unit vector
% B = B / norm(B);
% 2 - Make B a vector with z-part in the z_pl
% B = B / (B(3)/z_pl);

m = k*(B(1)^2 + B(2)^2 + B(3)^2);
a = B(1)*B(1) - m;
b = 2*B(1)*B(2);
c = B(2)*B(2) - m;
d = 2*B(1)*B(3)*z_pl;
f = 2*B(2)*B(3)*z_pl;
g = z_pl*z_pl * (B(3)*B(3) - m);

conic_params = [a, b, c, d, f, g];