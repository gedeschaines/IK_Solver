function [gain] = closing_gain(ikm,ec,pt,vt)

% function [gain] = closing_gain(ikm,ec,pt,vt) : returns end effector
%                                                closing gain
%   ikm = IKmethod
%   ec  = end effector current position in world space
%   pt  = position of target in world space
%   vt  = velocity of target in world space

de  = ec - pt;
nde = norm(de);
nvt = norm(vt);
if (nde == 0.0) || (nvt == 0.0)
  gain = 1.0;
else
  % compute cosine of approach angle relative to target velocity vector
  aacos = dot(de/nde,vt/nvt);
  if aacos <= 0.0
    gain = 1.6;
  else
    if aacos <= 0.7071
      gain = 1.2;
    else
      if ikm < 6
        gain = 1.0;  % without null space control
      else
        gain = 0.8;  % with null space control
      end
    end
  end
end

end
