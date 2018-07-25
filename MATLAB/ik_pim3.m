function [dq] = ik_pim3(J,de,sfac,dqlim,phi)

% function [dq] = ik_pim3(J,de,sfac,dqlim,phi) : solves de = J*dq for dq using
%                                                the pseudo-inverse method from
%                                                reference [3].
%   J     = Jacobian matrix of de/dq
%   de    = vector of delta errors
%   sfac  = singularity threshold factor
%   dqlim = joint rotation delta limit
%   phi   = null space control vector

[m,n]   = size(J);
[U,S,V] = svd(J);

slim = sfac*max(abs(diag(S)));
Sinv = zeros(n,m);
for i = 1:m
  if abs(S(i,i)) > slim;
    Sinv(i,i) = 1.0/S(i,i);
  end
end

Jinv  = V*(Sinv*U');
Jprj  = eye(n) - Jinv*J;
dq    = (Jinv*de' + Jprj*phi')';
dqmax = max(abs(dq));
if dqmax > dqlim
  dq = (dqlim/dqmax)*dq;
end

end
