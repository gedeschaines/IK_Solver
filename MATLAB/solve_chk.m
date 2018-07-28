function [i] = solve_chk(ni,ilim,p,ec,et,dxy,dz,derr,perr)

% function [i] = solve_chk(ni,ilim,p,ec,et,dxy,dz,derr,perr) : returns 0 or 1
%   ni   = number of iterations
%   ilim = iteration limit
%   p    = set of n link joint position vectors
%   ec   = end effector current position vector
%   et   = end effector target position vector
%   dxy   = length of fully extended link chain in xy plane
%   dz   = lenght of fully extended link chain in z direction
%   derr = allowable effector to target distance error
%   perr = allowable effector to target pointing error

i = 0;  % assume effector has not reached target

if ni == ilim
  % encountered iteration limit
  i = 1;
else
  etp = et-p{1};           % vector from link chain base to target
  ecp = ec-p{1};           % vector from link chain base to effector
  detxy = norm(etp(1:2));  % distance from base to target in xy plane
  detz = abs(etp(3));      % distance from base to target in z direction
  decxy = norm(ecp(1:2));  % distance from base to effector in xy plane
  dotxy = (ecp(1:2)*etp(1:2)')/detxy;  % portion of ecp along etp in xy plane

  if ((detxy - dxy) > derr) || ((detz - dz) > derr)
    % target currently beyond reach of effector; but is it
    % within allowable distance and xy pointing error?
    if (abs(decxy-dxy) <= derr) && (acos(dotxy/decxy) <= perr)
      i = 1;
    end 
  else
    % target currently not beyond reach of effector; but is it
    % within allowable distance?
    if norm(et-ec) <= derr
      i = 1;
    end
  end
  
end
