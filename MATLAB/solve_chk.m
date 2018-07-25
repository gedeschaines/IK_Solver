function [i] = solve_chk(ni,ilim,p,ec,et,d,derr,perr)

% function [i] = solve_chk(ni,ilim,p,ec,et,d,derr,perr) : returns 0 or 1
%   ni   = number of iterations
%   ilim = iteration limit
%   p    = set of n link joint position vectors
%   ec   = end effector current position vector
%   et   = end effector target position vector
%   d    = length of fully extended link chain
%   derr = allowable effector to target distance error
%   perr = allowable effector to target pointing error

i = 0;  % assume effector has not reached target

if ni == ilim
  % encountered iteration limit
  i = 1;
else
  etp = et-p{1};         % vector from link chain base to target
  ecp = ec-p{1};         % vector from link chain base to effector
  det = norm(etp);       % distance from base to target
  dec = norm(ecp);       % distance from base to effector
  dot = (ecp*etp')/det;  % portion of dec along det
  if det > d
    % effector cannot possibly reach target
    if (abs(d-dec) <= derr) && (acos(dot/dec) <= perr)
      i = 1;
    end 
  else
    % target not beyond reach of effector
    if norm(et-ec) <= derr
      i = 1;
    end
  end
  
end
