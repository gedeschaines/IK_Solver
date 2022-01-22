function [yaw,pitch,roll] = get_endeff_ypr(nq,q,u)

% function [yaw,pich,roll] = get_endeff_ypr(nq,q,u) : get current end effector yaw,
%                                                     pitch and roll angles in radians
%                                                     for Euler rotations about the last
%                                                     link segment zyx axes respectively.
%                                                     Assumes the pitch angle never
%                                                     reaches +/- 90 degrees.
%
%   nq = number of joints
%   q  = vector of joint rotations (rads)
%   u  = set of joint rotation axis unit vectors

% Rotation matrix from world space to last joint local space
R = eye(3,3);
for i = 1 : nq
  Ri = transpose(rotation(q(i),u{i}));
  R  = Ri*R;
end
% Euler rotation angles of last link segment
yaw   =  atan2(R(1,2),R(1,1));
pitch = -asin(R(1,3));
roll  =  atan2(R(2,3),R(3,3));

end
