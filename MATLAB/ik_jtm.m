function [dq] = ik_jtm(J,de,dqlim)

% function [dq] = ik_jtm(J,de,dqlim) : solves de = J*dq for dq using
%                                      the Jacobian transpose method
%   J     = Jacobian matrix of de/dq
%   de    = vector of delta errors
%   dqlim = vector of joint rotation delta limits

dq    = J'*de';
jdq   = J*dq;
alpha = (de*jdq)/(norm(jdq')^2);
dqmax = max(abs(dq));
beta  = dqlim(1)/dqmax;
dq    = min([alpha,beta])*dq';

end
