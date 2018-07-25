function [dq] = ik_pim2(J,de,sdel,dqlim)

% function [dq] = ik_pim2(J,de,sdel,dqlim) : solves de = J*dq for dq using
%                                            the pseudo-inverse method from
%                                            reference [2]
%   J     = Jacobian matrix of de/dq
%   de    = vector of delta errors
%   sdel  = singularity prevention damping constant
%   dtlim = joint rotation delta limit

n     = size(J,2);
A     = J'*J + sdel*eye(n);
b     = J'*de';
dq    = (A \ b)';
dqmax = max(abs(dq));
if dqmax > dqlim
  dq = (dqlim/dqmax)*dq;
end

end
