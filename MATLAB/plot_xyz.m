function [X,Y,Z] = plot_xyz(n,p)

% function [X,Y,Z] = plot_xyz(n,p)
%   p  = set of n link joint position vectors

X = zeros(n,1);
Y = zeros(n,1);
Z = zeros(n,1);
for i = 1:n
  X(i) = p{i}(1);
  Y(i) = p{i}(2);
  Z(i) = p{i}(3);
end

end
