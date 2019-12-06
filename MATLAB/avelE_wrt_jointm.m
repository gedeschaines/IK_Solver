function [avel] = avelE_wrt_jointm(w,p,dq,m)

% function [avel] = avelE_wrt_jointm(w,p,dq,m) : returns angular velocity of
%                                                line-of-sight for end-effector
%                                                with respect to joint m,
%                                                assuming the first joint 
%                                                position is fixed in world
%                                                space.
%   w  = set of n joint rotation direction vectors in world space
%   p  = set of n+1 position vectors (n joints + end-effector) in world space
%   dq = vector of n joint rotation rates (radians/sec)
%   m  = joint number of interest [1,n]

%try
%  assert m-1 in range(len(dq)), 'value of m-1 not in range(len(dq))'
%catch AssertionError as err
%  ename = err.__class__.__name__
%  fname = 'avelE_wrt_jointm'
%  fprintf('%s in %s: %s\n', ename, fname, err)
%  sys.exit()

n  = length(dq);  % number of joints
ie = length(p);   % index of end effector position vector

% absolute velocity of end-effector
vE = zeros(1,3);
for i = n:-1:1
  vE = vE + cross(dq(i)*w{i},(p{i+1}-p{i}));
end
% absolute velocity of joint m
vJ = zeros(1,3);
for i = 1:1:m-1
  vJ = vJ + cross(dq(i)*w{i},(p{i+1}-p{i}));
end
% relative velocity of end-effector wrt joint m
vEJ = vE - vJ;
% direction vector to end-effector from joint m
vecp = p{ie} - p{m};
nrmp = norm(vecp);
% line-of-sight angular velocity for end-effector wrt joint m
avel = cross(vecp,vEJ)/(nrmp*nrmp);

end