function [ img ] = getimage (fname)
%% Reads and returns image from named file.
%%
%% see putimage

  [img, map, alpha]  = imread(fname);

endfunction