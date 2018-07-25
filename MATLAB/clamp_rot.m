function [r] = clamp_rot(n,r,rmin,rmax)

% function [r] = clamp_rot(n,r,rmin,rmax) : returns r within [rmin,rmax]
%   r    = vector of n joint rotations (radians)
%   rmin = vector of n joint rotation minimums (radians)
%   rmax = vector of n joint rotation maximums (radians)

global p360rad;
global n360rad;
global p180rad;
global n180rad;

%global dpr;
%display(sprintf('clamp_rot: r= %8.2f, %8.2f %8.2f', ...
%    r(1)*dpr, r(2)*dpr, r(3)*dpr));

for i = 1:n
  if abs(rmax(i)-rmin(i)) >= p360rad 
    if r(i) < n180rad
      r(i) = p360rad + r(i);
    end
    if r(i) > p180rad
      r(i) = r(i) + n360rad;
    end
  end
  if r(i) < rmin(i)
    r(i) = rmin(i);
  elseif r(i) > rmax(i)
    r(i) = rmax(i);
  end
end

end
