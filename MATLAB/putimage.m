function [ fpath ] = putimage(h, i, fmt, iW, iH)
%% Saves image associated with given figure handle 'h'and frame index
%% 'i' in format specified by 'fmt' and size by iW x iH. Returns path
%% to saved image file.

  fpath = sprintf ("./images/%ss/img_%04d.%s", fmt, i, fmt);
  psiz = sprintf("-S%d,%d", iW, iH);
  print(h, fpath, psiz);
  
endfunction
