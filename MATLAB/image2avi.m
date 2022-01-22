function [status] = image2avi (fmt, fps, w, h, name)
%% Utilizes 'avconv' libav-tool to convert sequence of images
%% with given format type 'fmt' to named AVI movie file with
%% frame rate 'fps' and frame size 'w' x 'h'. Returns avconv
%% exit status.
%%
%% see putimage

if exist("/usr/bin/avconv", 'file')
  ffmt = "-y -f image2 ";
  rate = sprintf ("-r %d ", fps);
  imgs = sprintf ("-i ./images/%ss/img_%s.%s ", fmt, '%04d', fmt);
  cdec = "-c mjpeg ";
  imsz = sprintf ("-s %dx%d ", w,  h);
  rdir = " 2>&1";
  scmd = ["/usr/bin/avconv ", ffmt, rate, imgs, cdec, rate, imsz, name, rdir];

  [status, output] = system (scmd, true, "sync");

  if status ~= 0
    printf ("%s", output);
  end

else
  printf ("requires avconv libav-tool to create movie.\n");
end

endfunction
