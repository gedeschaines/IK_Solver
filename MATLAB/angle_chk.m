function angle_chk(q,u,p,et)

% function angle_chk(q,u,p,et)
%   q  = vector of n joint rotations (radians)
%   u  = set of n joint local rotation axis unit vectors
%   p  = set of n link joint position vectors
%   et = end effector target position vector
 
global  dpr;

display('* Angle Check:');
n = length(q);
k = 1;
a = q(k)*dpr;
fprintf('joint %1d rotation angle from inertial x-axis  = %12.6f\n',k,a);
for k = 2:n 
  a = q(k)*dpr;
  fprintf('joint %1d rotation angle from joint %1d x-axis   = %12.6f\n',k,k-1,a));
end
asum = sum(q)*dpr;
fprintf('angle to joint %1d x-axis from inertial x-axis = %12.6f\n',n,asum));
etp = et - p{n};  % vector from joint n to target
det = norm(etp);  % distance from joint n to target
at3 = acos((etp*(rotation(sum(q),u{n})*[1;0;0]))/det)*dpr;
atI = acos((etp*[1;0;0])/det)*dpr;
fprintf('angle to target from joint %1d x-axis          = %12.6f\n',n,at3));
fprintf('angle to target from inertial x-axis         = %12.6f\n',atI));

end
