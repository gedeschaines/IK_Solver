function [ fpath ] = putimage(h, i, fmt)
%% Saves image associated with given figure handle 'h'and frame index
%% 'i' in format specified by 'fmt'. Returns path to saved image file.

  fpath = sprintf ("./images/%ss/img_%04d.%s", fmt, i, fmt);
  saveas(h, fpath, fmt);
  
endfunction