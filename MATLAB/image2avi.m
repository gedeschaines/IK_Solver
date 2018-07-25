function [status] = image2avi (fmt, fps, w, h, name)
%% Utilizes 'avconv' libav-tool to convert sequence of images
%% with given format type 'fmt' to named AVI movie file with
%% frame rate 'fps' and frame size 'w' x 'h'. Returns avconv
%% exit status.
%%
%% see putimage

ffmt = "-y -f image2 ";
imgs = sprintf ("-i ./images/%ss/img_%s.%s ", fmt, '%04d', fmt);
rate = sprintf ("-r %d ", fps);
size = sprintf ("-s %dx%d ", w,  h);
rdir = " 2>&1";
scmd = ["/usr/bin/avconv ", ffmt, imgs, rate, size, name, rdir];

[status, output] = system (scmd, true, "sync");

if status ~= 0
  printf ("%s", output);
end

endfunction