function [ fpath ] = putimage(h, i, fmt, p3D)
%% Saves image associated with given figure handle 'h'and frame index
%% 'i' in format specified by 'fmt' and size according to p3D. Returns
%% path to saved image file.

  fpath = sprintf ("./images/%ss/img_%04d.%s", fmt, i, fmt);
  pdev = sprintf('"-d%s"', fmt);
  if p3D == 0
    psiz = "-S570,450";
  else
    psiz = "-S560,420";
  end
  print(h, fpath, psiz)
  
endfunction
