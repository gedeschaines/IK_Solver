function [avel] = avelT_wrt_jointm(pt,vt,w,p,dq,m)

% function [avel] = avelT_wrt_jointm(pt,vt,w,p,dq,m) : returns angular velocity of
%                                                      the target with respect to
%                                                      joint m, assuming the first
%                                                      joint position is fixed in
%                                                      world space.
%   pt = current position of target in world space
%   vt = current velocity of target in world space
%   w  = set of n joint rotation direction vectors in world space
%   p  = set of n+1 position vectors (n joints + end-effector) in world space
%   dq = vector of n joint rotation rates (radians/sec)
%   m  = joint number of interest [1,n]

%try
%  assert m-1 in range(len(dq)), 'value of m-1 not in range(len(dq))'
%catch AssertionError as err
%  ename = err.__class__.__name__
%  fname = 'avelT_wrt_jointm'
%  fprintf('%s in %s: %s\n', ename, fname, err)
%  sys.exit()
%end

n  = length(dq);  % number of joints
ie = length(p);   % index of end effector position vector

% absolute velocity of joint m
vJ = zeros(1,3);
for i = 1:1:m-1
  vJ = vJ + cross(dq(i)*w{i},(p{i+1}-p{i}));
end
% relative velocity of target wrt joint m
vTJ = vt - vJ;
% direction vector to target from joint m
vecp = pt - p{m};
nrmp = norm(vecp);
% angular velocity of the targer wrt to joint m
avel = cross(vecp,vTJ)/(nrmp*nrmp);

end
