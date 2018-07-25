function [p,w] = transform(n,r,u,a)

% function [p,w] = transform(n,r,u,a) : transforms a and u to p and w
%   r = vector of n-1 joint rotations (radians)
%   u = set of n-1 joint rotation axis unit vectors
%   a = set of n link joint attachment location vectors to transform

p = a;
for i = 1:1:n
  p{i} = a{i};
  j    = i - 1;
  while j > 0
    p{i} = (rotation(r(j),u{j})*p{i}')';
    p{i} = p{i} + a{j};
    j    = j - 1;
  end
end

w = u;
for i = 1:1:n-1
  j = i;
  while j > 0
    w{i} = (rotation(r(j),u{j})*w{i}')';
    j    = j - 1;
  end
  w{i} = w{i}/norm(w{i});
end
