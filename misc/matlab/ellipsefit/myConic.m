function z = myConic(x, y, a, b, theta, h, k)

%Ax2 + By2 + Cxy ? (2Ah + kB)x ? (2Ck + Bh)y + (Ah2 + Bhk + Ck2 ? 1) = 0

A = ((cos(theta))^2)/a^2 + ((sin(theta))^2)/b^2;
B = ((sin(theta))^2)/a^2 + ((cos(theta))^2)/b^2;
C = -2*cos(theta)*sin(theta)*((1/a^2) - (1/b^2));
D = 2*A*h + k*B;
E = -2*C*k - B*h;
F = A*h^2 + B*h*k + C*k^2 - 1;

z = A*x.^2 + B*y.^2 + C*x.*y + D*x + E*y + F;
end