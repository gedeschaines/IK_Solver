function [R] = rotation(r,u)

% function [R] = rotation(r,u) : returns local to global rotation matrix
%   r = joint rotation (radians)
%   u = joint rotation axis unit vector

theta  = r/2;
costh = cos(theta);
sinth = sin(theta);

q0 = costh;
q1 = sinth*u(1);
q2 = sinth*u(2);
q3 = sinth*u(3);

q00 = q0*q0;
q01 = q0*q1;
q02 = q0*q2;
q03 = q0*q3;
q11 = q1*q1;
q12 = q1*q2;
q13 = q1*q3;
q22 = q2*q2;
q23 = q2*q3;
q33 = q3*q3;

R = [q00+q11-q22-q33, 2*(q12-q03), 2*(q13+q02);
     2*(q12+q03), q00-q11+q22-q33, 2*(q23-q01);
     2*(q13-q02), 2*(q23+q01), q00-q11-q22+q33];
   
end
