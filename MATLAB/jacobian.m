function [J] = jacobian(n,r,p,e)

% function [J] = jacobian(n,r,p,e) : returns de/dq Jacobian matrix for
%                                    e representing a 1x3 vector in
%                                    world space [x,y,z] coordinates
%   r = set of n joint rotation direction vectors (i.e., a set
%       {r1, r2, r3, ... rn} where r's are vectors of the form 
%       [rx,ry,rz]).
%   p = set of at least n-1 link joint position vectors (i.e., 
%       {p1, p2, p3, ...} where p's are vectors of the form 
%       [px,py,pz]).
%   e = desired effector position vector (i.e., [ex,ey,ez]).

J = zeros(3,n);
for i = 1:n
  j      = cross(r{i},(e - p{i}));  % de/dq = r x (e - p)
  J(1,i) = j(1);  % | dex/dq1  dex/dq2  dex/dq3 ... dex/dqn |
  J(2,i) = j(2);  % | dey/dq1  dey/dq2  dey/dq3 ... dey dqn |
  J(3,i) = j(3);  % | dez/dq1  dez/dq2  dez/dq3 ... dez/dqn |
end

end
