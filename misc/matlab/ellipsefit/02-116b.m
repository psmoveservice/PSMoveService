% ALS_FIT - Adjusted least squares fit of data to an ellipsoid
%
% [ah,ch,sh] = als_fit(x,ub,tol)
%
% Input:  x(:,l) is the l-th data point
%         ub  - an upper bound for the noise standard deviation
%         tol - convergence tolerance for the noise variance estimation
% Output: the estimated ellipsoid is (x-ch)'*ah*(x-ch) = 1 
%         sh  - noise standard deviation estimate

function [ah,ch,sh] = als_fit(x,ub,tol)

  [n,m] = size(x);

  % check inputs
  if nargin == 0
    error('Usage: als_fit(x) or als_fit(x,v)')
  end
  if nargin == 1
    % compute upper bound
    mx = mean(x,2);
    ub = sqrt(norm(x - mx(:,ones(1,m)),'fro')^2 / (n*(m-1))); 
  end
  if nargin < 3
    % default tolerance
    tol = 0.001;
  else
    % check if the upper bound is skiped by [] 
    if isempty(ub)
      % compute upper bound
      mx = mean(x,2);
      ub = sqrt(norm(x - mx(:,ones(1,m)),'fro')^2 / (n*(m-1))); 
    end
  end

  na    = (n+1)*n/2; % number of parameters in A
  nbeta = na+n+1;    % total number of parameters

  % indices for svec of an (n+1)x(n+1) matrix
  svec = 1:(n+1)^2; 
  svec = reshape(svec,n+1,n+1); 
  svec = triu(svec);
  svec = svec(:); 
  svec(find(svec==0)) = [];
  % indices for svec of an nxn matrix
  sveca = 1:n^2; 
  sveca = reshape(sveca,n,n); 
  sveca = triu(sveca);
  sveca = sveca(:); 
  sveca(find(sveca==0)) = [];
  % off diagonal elements in an nxn matrix
  i = 1:n;
  off_diag = setdiff(1:na,i.*(i+1)./2);

  % form T
  T = zeros(5,n,m);
  xx   = x.^2;
  xxx  = xx.*x;
  xxxx = xx.*xx;
  T(1,:,:) = ones(n,m);
  T(2,:,:) = x;
  % the rest depends on s and is in the bisection loop

  % form M
  M1 = kron([1:n,0]',ones(1,n+1));
  M1 = M1(svec);
  M2 = kron(ones(n+1,1),[1:n,0]); 
  M2 = M2(svec);
  M  = [ M1 M2 ];

  % form N
  Na = zeros(na,na,n);
  for p = 1:na
    for q = 1:na
      if q >= p
	MpMq = [M(p,:) M(q,:)];
	i    = (1:n)';
	Na(p,q,:) = sum(MpMq(ones(n,1),:) == i(:,ones(1,4)),2) + 1;
      end
    end
  end
  Nb = zeros(na,n,n);
  for p = 1:na
    for q = 1:n
      MpMq = [M(p,:) M(na+q,:)];
      i    = (1:n)';
      Nb(p,q,:) = sum(MpMq(ones(n,1),:) == i(:,ones(1,4)),2) + 1;
    end
  end

  % for the computation of psi
  xxt  = x*x';
  sumx = sum(x,2);

  %%%%%%%%%%%%%
  % Bisection %
  %%%%%%%%%%%%%

  lb = 0;
  s  = (ub+lb)/2;
  while (ub-lb) / s > tol
    
    % form the rest of T
    ss = s^2;
    T(3,:,:) = xx   - ss;
    T(4,:,:) = xxx  - 3*ss*x;
    T(5,:,:) = xxxx - 6*ss*xx + 3*ss*ss;

    % form psi
    psia = zeros(na,na);
    for p = 1:na
      for q = 1:na
	if q >= p
	  for l = 1:m
	    j = 1:n;
	    psia(p,q) = psia(p,q) + prod(diag(T(Na(p,q,j),j,l)));
	  end
	end
      end
    end
    psib = zeros(na,n);
    for p = 1:na
      for q = 1:n
	for l = 1:m
	  j = 1:n;
	  psib(p,q) = psib(p,q) + prod(diag(T(Nb(p,q,j),j,l)));
	end
      end
    end
    psi = zeros(nbeta,nbeta);
    psi(na+1:na+n,end) = sumx;
    psi(end) = m;
    psi(1:na,1:na) = psia;
    psi(1:na,na+1:na+n) = psib;
    psi22 = xxt - m*ss*eye(n);
    psi(na+1:na+n,na+1:na+n) = triu(psi22); 
    psi(1:na,end) = psi22(sveca);

    % make lower-triangular part = upper triangular part
    psi = psi + psi' - diag(diag(psi));
    
    % correction for skron
    psi(:,off_diag) = psi(:,off_diag)*2; 
    psi(off_diag,:) = psi(off_diag,:)*2;

    % EVD
    ev = eig(psi);

    if min(ev) > 0
      lb = s;
    else
      ub = s;
    end
    s = (ub+lb)/2;
  end

  % noise standard deviation estimate
  sh = s;

  %%%%%%%%%%%%%%%%%%%
  % ALS1 estimation %
  %%%%%%%%%%%%%%%%%%%

  % EVD
  [evec,ev]   = eig(psi);
  [min_ev,im] = min(diag(ev));
  sol         = evec(:,im);
  sol         = sol / norm(sol);

  % extract the matrices
  a = zeros(n); 
  a(sveca) = sol(1:na); 
  a = a+a'-diag(diag(a));
  b = sol(na+1:end-1);
  d = sol(end);

  % solve for c and a
  ch = -1/2*(a\b);
  ah =  1/(ch'*a*ch-d)*a;

  % ah > 0 ?
  if ~all(real(eig(ah)) > 0 )
    disp('Warning: projecting to the set of the positive semidefinite matrices.')
    [evec,ev] = eig(ah);
    ev = diag(ev);
    if any(ev>0)
      pos = find(ev>0);
      ah  = evec(:,pos) * diag(ev(pos)) * evec(:,pos)';
    else
      ah = zeros(n);
    end
  end

