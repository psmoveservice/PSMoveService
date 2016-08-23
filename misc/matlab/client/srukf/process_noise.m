function q = process_noise(N, DELTA)
% http://www.tandfonline.com/doi/abs/10.1080/00207728908910103
my_i = (1:N)';
my_j = 1:N;
my_i_plus_j = my_i(:, ones(N,1)) + my_j(ones(N,1), :);
X = 2*N + 3 - my_i_plus_j;
nums = DELTA.^X;
facs = factorial(N + 1 - my_i);
denoms = (facs * facs') .* X;
q = nums ./ denoms;