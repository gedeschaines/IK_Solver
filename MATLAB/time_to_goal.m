function [tgo] = time_to_goal(pt,vt,w,p,dq)

% function [tgo] = time_to_goal(pt,vt,w,p,dq) : returns estimated time for
%                                               end-effector to reach the
%                                               target goal
%   pt = current position of target in world space
%   vt = current velocity of target in world space
%   w  = set of n joint rotation direction vectors in world space
%   p  = set of n+1 position vectors (n joints + end-effector) in world space
%   dq = vector of n joint rotation rates (radians/sec)

n  = length(dq);
ie = length(p);
% absolute velocity of end effector
ve = zeros(1,3);
for i = n:-1:1
  ve = ve + cross(dq(i)*w{i},p{i+1}-p{i});
end
% closing vector between end-effector and target
vecp = pt - p{ie};
nrmp = norm(vecp);
% estimated time to go
if nrmp > 0.0
  vecn = vecp/nrmp;
  tgo  = nrmp/(dot(ve-vt,vecn));
  if tgo < 0.0
    tgo = 0.0;
  end
else
  tgo = 0.0;
end
  
end
