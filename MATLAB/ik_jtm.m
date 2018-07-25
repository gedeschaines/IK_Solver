function [dq] = ik_jtm(J,de,dqlim)

% function [dq] = ik_jtm(J,de,dqlim) : solves de = J*dq for dq using
%                                      the Jacobian transpose method
%   J     = Jacobian matrix of de/dq
%   de    = vector of delta errors
%   dqlim = joint rotation delta limit

dq    = J'*de';
jdq   = J*dq;
alpha = (de*jdq)/(norm(jdq')^2);
dqmax = max(abs(dq));
beta  = dqlim/dqmax;
dq    = min([alpha,beta])*dq';

end
