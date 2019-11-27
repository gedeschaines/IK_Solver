function [phiZ] = get_endeff_rot(nq,q,u)

% function [phiZ] = get_endeff_rot(nq,q,u) : get current end effector rotation angle
%                                            in radians for rotation about the last
%                                            link segment z-axis.
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
% World space X-axis in last joint local space
vecX = transpose(R*transpose([1.0,0.0,0.0]));
% Angle between last link segment and X-axis
phiZ = atan2(vecX(2),vecX(1));

end
