function [dq] = ik_dls(J,de,slam,dqlim,phi)

% function [qq] = ik_dls(J,de,slam,dqlim,phi) : solves de = J*dq for dq using
%                                               damped least squares with SVD
%                                               from reference [3].
%   J     = Jacobian matrix of de/dq
%   de    = vector of delta errors
%   slam  = singularity damping factor
%   dqlim = vector of joint rotation delta limits
%   phi   = null space control vector

[m,n]   = size(J);
[U,S,V] = svd(J);
Sinv = zeros(n,m);
for i = 1:m
  Sinv(i,i) = S(i,i)/(S(i,i)^2 + slam^2);
end
Jinv  = V*(Sinv*U');
Jprj  = eye(n) - Jinv*J;
dq    = (Jinv*de' + Jprj*phi')';
dqmax = max(abs(dq));
for i = 1:length(dq)
  if dqmax > dqlim(i)
    dq(i) = dq(i)*(dqlim(i)/dqmax)
  end
end

end
