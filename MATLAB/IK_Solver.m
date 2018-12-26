% File: IK_Solver.m
% Auth: G. E. Deschaines
% Date: 27 Feb 2015
% Prog: Inverse Kinematics (IK) Solver
% Desc: Applies user selected IK solving technique to determine and
%       plot the motion of a jointed n-link chain in 2D or 3D space.
%
% This MATLAB program presents a 3D implementation of inverse kinematic
% solutions to effector movement for a n-link, revolute-joint chain
% towards a target using cyclic coordinate descent (CCD) [1], Jacobian
% transpose method (JTM) [3], pseudo-inverse method (PIM) [1,2,3] or 
% damped least squares (DLS) [3] as detailed in the listed references.
% The program is comprised of the following script files.
%
%   IK_Solver.m    - main program utilizing the following functions:
%   ik_jtm.m       - apply IK Jacobian transpose method
%   ik_pim2.m      - apply IK pseudo-inverse method from ref [2]
%   ik_pim3.m      - apply IK pseudo-inverse method from ref [3]
%   ik_dls         - apply IK damped least squares technique
%   solve_chk.m    - IK solver solution check
%   time_to_goal.m - time to reach target
%   jacobian.m     - compute Jacobian matrix
%   rotation.m     - compute coordinate rotation matrix
%   transform.m    - transform coordinates from local to world space  
%   clamp_rot.m    - clamp joint rotations to specified limits
%   plot_xy.m      - extract link plot x,y coordinates for 2D plotting
%   plot_xyz.m     - extract link plot x,y,z coordinates for 3D plotting
%   angle_chk.m    - debug display of joint rotation angle checks
%   isOctave.m     - returns true if script is being executed by Octave
%
%   The following functions are not required by MATLAB, but are used
%   by Octave to emulate MATLAB getframe and movie2avi functions. These
%   functions do not allow smooth animation as in MATLAB. There will
%   be disruptive graphic flickering and figure jumping as putimage
%   saves an image of the figure to a file.
%
%   putimage.m  - writes animation frame image to file
%   getimage.m  - reads animation frame image from file
%   image2avi.m - converts sequence of frame image files to avi video
%
% The IK solution technique is user selectable, and the program can be 
% configured to record the plotted link chain motion for animation 
% playback and saving as a video file.
%
% References:
%
% [1] Benjamin Kenwright, "Practical 01: Inverse Kinematics", September
%     2014, School of Computer Science, Edinburgh Napier University, 
%     United Kingdom, Physics-Based Animation practical web available at
%     http://games.soc.napier.ac.uk/study/pba_practicals/Practical%2001%20-%20Inverse%20Kinematics.pdf
%
% [2] Ben Kenwright, "Real-Time Character Inverse Kinematics using
%     the Gauss-Seidel Iterative Approximation Method", July 2012, 
%     CONTENT 2012, The Fourth International Conference on Creative 
%     Content Technologies, web available at
%     http://www.thinkmind.org/index.php?view=article&articleid=content_2012_4_10_60013
%
% [3] Samuel R. Buss, "Introduction to Inverse Kinematics with Jacobian
%     Transpose, Pseudoinverse and Damped Least Squares methods", October
%     2009, Department of Mathematics, University of California, San
%     Diego, unpublished article web available at
%     http://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
%
% Disclaimer:
%
% See IK_Solver/DISCLAIMER

clear variables
close all

more off

%#ok<*NOPTS>

global rpd;
global dpr;

rpd = pi/180;  % radians per degree conversion constant
dpr = 180/pi;  % degrees per radian conversion constant

global p360rad;  % globals used in clamp_rot function
global n360rad;  %
global p180rad;  %
global n180rad;  %

p360rad =  360*rpd;
n360rad = -360*rpd;
p180rad =  180*rpd;
n180rad = -180*rpd;

Plot3D = 0;  % plot in 3D flag
Record = 0;  % record movie flag

UseCCD   = 1;  % use cyclic coordinate descent
UseJTM   = 2;  % use jacobian transpose method
UsePIM2  = 3;  % use pseudo-inverse method from ref [2]
UsePIM3  = 4;  % use pseudo-inverse method from ref [3]
UseDLS   = 5;  % use damped least squares from ref [3]
IKmethod = 0;  % selected IK solver method

prompt = 'Select IK Solver (1=CCD,2=JTM,3=PIM2,4=PIM3,5=DLS)? ';
while IKmethod == 0
  str = input(prompt,'s');  
  switch str
    case '1'
      title = 'IK_Solver - Cyclic Coordinate Descent';
      IKmethod = UseCCD;
    case '2'
      title = 'IK_Solver - Using Jacobian Transpose Method';
      IKmethod = UseJTM;
    case '3'
      title = 'IK_Solver - Using Pseudo-Inverse Method [ref 2]';
      IKmethod = UsePIM2;
    case '4'
      title = 'IK_Solver - Using Pseudo-Inverse Method [ref 3]';
      IKmethod = UsePIM3;
    case '5'
      title = 'IK_Solver - Using Damped Least Squares [ref 3]';
      IKmethod = UseDLS;
    otherwise
      fprintf('Invalid entry\n');
      IKmethod = 0;
  end
end
fprintf('%s\n',title);
fprintf('On the plot, use the mouse buttons:\n');
if Plot3D == 0
  fprintf('  + left to select target location\n');
else
  fprintf('  + left to randomly locate target\n');
end
fprintf('  + right to stop the IK solver\n');
fprintf('Wait until program exits to close plot window.\n');

% Jointed N-Link Chain Pictogram
%                                                                 +    
%      uz1          uz2          uz3              uzN          . (a)N+1 
%       | uy1        | uy2        | uy3            | uyN    .     ^end
%       | /          | /          | /              | /   .        | effector
%       |/           |/           |/               |/ .           |
%  base>+------>ux1 -+------>ux2 -+------>ux3 ... -+------>uxN    |
%      (a)0         (a)1         (a)2             (a)N            |     
%       ^joint_1     ^joint_2     ^joint_3         ^joint_N       |
%       | & link_1   | & link_2   | & link_3       | & link_N     |
%       |            |            |                |              |
%  <world space>     +----< preceding link body space [x,y,z] >---+

if Plot3D == 0
  a = {[0.0,0.0,0.0],...  % <- joint 1 + set of link chain attachment
       [2.0,0.0,0.0],...  % <- joint 2 | points in body coordinates,
       [2.0,0.0,0.0],...  % <- joint 3 | except for the base given in 
       [1.0,0.0,0.0],...  % <- joint 4 / world space coordinates
       [1.0,0.0,0.0]};    % <- end effector
  u = {[0.0,0.0,1.0],...  % <- joint 1 + set of joint rotation axis unit
       [0.0,0.0,1.0],...  % <- joint 2 | vectors in link body coordinates
       [0.0,0.0,1.0],...  % <- joint 3 | 
       [0.0,0.0,1.0]};    % <- joint 4 /
  q0    = [  25*rpd,...   % <- joint 1 + initial joint rotations (radians)
             15*rpd,...   % <- joint 2 |
             10*rpd,...   % <- joint 3 |
              5*rpd];     % <- joint 4 /
  qmin  = [-360*rpd,...   % <- joint 1 + joint minimum rotations (radians)
           -135*rpd,...   % <- joint 2 |
            -60*rpd,...   % <- joint 3 |
            -60*rpd];     % <- joint 4 /
  qmax  = [ 360*rpd,...   % <- joint 1 + joint maximum rotations (radians)
            135*rpd,...   % <- joint 2 |
             60*rpd,...   % <- joint 3 |
             60*rpd];     % <- joitn 4 /
  dqlim = [  30*rpd,...   % <- joint 1 + joint delta rotation limit (rad/sec)
             30*rpd,...   % <- joint 2 |
             30*rpd,...   % <- joint 3 |
             30*rpd];     % <- joitn 4 /
else
  a = {[0.0,0.0,0.0],...  % <- joint 1 + set of link chain attachment
       [0.0,0.0,2.0],...  % <- joint 2 | points in body coordinates,
       [2.0,0.0,0.0],...  % <- joint 3 | except for the base given in 
       [2.0,0.0,0.0],...  % <- joint 4 / world space coordinates
       [0.5,0.0,0.0]};    % <- end effector
  u = {[0.0,0.0,1.0],...  % <- joint 1 + set of joint rotation axis unit
       [0.0,1.0,0.0],...  % <- joint 2 | vectors in link body coordinates
       [0.0,1.0,0.0],...  % <- joint 3 | 
       [0.0,1.0,0.0]};    % <- joint 4 /
  q0    = [   0*rpd,...   % <- joint 1 + initial joint rotations (radians)
            -45*rpd,...   % <- joint 2 |
             45*rpd,...   % <- joint 3 |
              0*rpd];     % <- joint 4 /
  qmin  = [-360*rpd,...   % <- joint 1 + joint minimum rotations (radians)
           -135*rpd,...   % <- joint 2 |
           -130*rpd,...   % <- joint 3 |
            -85*rpd];     % <- joint 4 /
  qmax  = [ 360*rpd,...   % <- joint 1 + joint maximum rotations (radians)
            135*rpd,...   % <- joint 2 |
            135*rpd,...   % <- joint 3 |
             85*rpd];     % <- joitn 4 /
  dqlim = [  30*rpd,...   % <- joint 1 + joint delta rotation limit (rad/sec)
             30*rpd,...   % <- joint 2 |
             30*rpd,...   % <- joint 3 |
             30*rpd];     % <- joitn 4 /
end
na = length(a);   % number of link body attachment points
np = na;          % number of link attachment world positions
nq = length(q0);  % number of link chain joints
s = zeros(1,nq);  % create array of link sizes
for i = 1 : na-1
  s(i) = norm(a{i+1});
end
efd = s(end);     % size of end effector link

% generalizations used in distance checks of iteration solve_chk function;
% assumes joint rotation limits do not prevent a full linear extension.
if Plot3D == 0
  dxy = sum(s(1:end));  % length of fully extended chain in xy plane
  dz  = 0.0;            % length of fully extended chain in z direction
else
  % following assumes first link strictly in the z direction
  dxy = sum(s(2:end));  % length of fully extended chain in xy plane
  dz  = sum(s(1:end));  % length of fully extended chain in z direction
end

q     = q0;                   % vector of current joint rotations
[p,w] = transform(na,q,u,a);  % joint positions/rotations in world space
ec    = p{na};                % end effector current position vector

% specify end effector target position and velocity
if Plot3D == 0
  et = [ 3.0, 4.0, 0.0];  % end effector target position vector
  vt = [-0.1, 0.0, 0.0];  % target velocity vector
else
  et = [ 3.0, 3.0, 2.0];  % end effector target position vector
  vt = [ 0.0,-0.2, 0.0];  % target velocity vector
end

pt    = et;                   % predicted target intercept position vector
h     = 0.04;                 % IK iteration step size
FPS   = 10;                   % IK animation frame rate
kfps  = round((1/FPS)/h);     % IK iteration count per frame for FPS rate
ilim  = round(30/h);          % IK iteration limit for 30 seconds
ni    = 0;                    % iteration counter
tsim  = 0.0;                  % simulation time (sec)
tgo   = h;                    % time to goal (sec)
sdel  = 0.001;                % PIM singularity damping constant from ref [2]
sfac  = 0.01;                 % PIM singularity threshold factor from ref [3]
slam  = 1.1;                  % DLS singularity damping factor from ref [3]
dH    = zeros(1,nq);          % null space control vector
derr  = 0.04*efd;             % allowable effector to target distance error
perr  = atan(derr/efd);       % allowable effector to target pointing error

text_tsim = @(t){ sprintf('time = %.3f', t) };

lw = 'linewidth';
fig1 = figure('Name',title,'NumberTitle','Off');
if Plot3D == 0
  [X0,Y0] = plot_xy(np,p);
  plot([X0(na),et(1)],[Y0(na),et(2)], ...
        X0,Y0,'-og', ...
        pt(1),pt(2),' *m', et(1),et(2),' *r');
  xlabel('X');ylabel('Y');
  axis([-6 6 -6 6]);
  axis square;
  text_x = -6.0;
  text_y =  6.5;
else
  et = [4.0, 0.0, 2.0];
  [X0,Y0,Z0] = plot_xyz(np,p);
  ViewAz = 45.0;
  ViewEl = 45.0;
  plot3(X0,Y0,Z0,'-og', ...
        pt(1),pt(2),pt(3),' *m', et(1),et(2),et(3),' *r');
  view(ViewAz,ViewEl);
  [ViewAz,ViewEl] = view();
  xlabel('X');ylabel('Y');zlabel('Z');
  axis([-5 5 -5 5  0 5]);
  text_x =  8.0;
  text_y = -2.0;
end
grid on;
hold on;
set(gca,'NextPlot','ReplaceChildren');
text(text_x,text_y,text_tsim(tsim),'fontweight','bold');
drawnow;

FMT = "jpg";  % Set to "png" or "jpg"
if Record == 1
  if ~exist("./images", 'dir')
    mkdir("./images");
  end
  if isOctave
    sdir = ["./images/", FMT, "s"];
    if ~exist(sdir, 'dir')
      mkdir(sdir);
    end
  end
  k = 1;
  if isOctave
    F(k,:) = putimage(fig1, k, FMT, Plot3D);
  else
    F(k) = getframe(fig1);
  end
end

% Load random number sequence. (For run reproducibility)
if Plot3D > 0
  nnum = 0;
  if exist("rnum.mat", 'file')
    load("rnum.mat", "rnum");
    nnum = length(rnum);
  end
  if nnum == 0
    nnum = 100;
    if isOctave
      rand ("seed", 987654321);  % Octave
      rnum = rand(nnum,1);
    else
      rng(987654321);            % Matlab
      rnum = rand(nnum,1);
    end
  end
  inum = 0;
end

%% Main loop over IK solutions

button = 1;
while button == 1
  ik = 1;  % iteration count till next plot update
  while solve_chk(ni,ilim,p,ec,et,dxy,dz,derr,perr) == 0
    % Calculate IK solution
    if IKmethod == UseCCD
      dq = zeros(1,nq);
      for i = nq:-1:1
        pt    = et + tgo*vt - p{i};
        npt   = norm(pt);
        pc    = ec - p{i};
        npc   = norm(pc);
        ut    = cross(cross(w{i},pc/npc), cross(w{i},pt/npt));
        ut    = ut/norm(ut);
        dqmax = min([acos((pt*(pc/npc).')/norm(pt)),dqlim(i)]);
        dq(i) = dqmax*(ut*(w{i}).');
      end
    else
      de = et + tgo*vt - ec;
      % Jt = jacobian(nq,w,p,et);
      Jc = jacobian(nq,w,p,ec);
      J  = Jc;
      switch IKmethod
        case UseJTM
          [dq] = ik_jtm(J,de,dqlim);
        case UsePIM2
          [dq] = ik_pim2(J,de,sdel,dqlim);  
        case UsePIM3
          [dq] = ik_pim3(J,de,sfac,dqlim,dH);
        case UseDLS
          [dq] = ik_dls(J,de,slam,dqlim,dH);
      end
    end
    % Update link chain angles and positions
    q     = q + h*dq;
    [q]   = clamp_rot(nq,q,qmin,qmax);
    [p,w] = transform(na,q,u,a);
    %angle_chk(q,u,p,et);
    % Update end effector current and target positions
    ec  = p{na};
    et  = et + h*vt;
    tgo = time_to_goal(et,vt,w,p,dq);
    pt  = et + tgo*vt;
    % Increment iteration and simulation time counters
    ni   = ni + 1;
    tsim = tsim + h;
    % Decrement iteration count till next plot update
    ik = ik - 1;
    if ik == 0
      if Plot3D == 0
        [X,Y] = plot_xy(np,p);
        plot([X0(na),pt(1)],[Y0(na),pt(2)],' -k', ...
              X0,Y0,'-og', X,Y,'-ob', ...
              et(1),et(2),' *r',  pt(1),pt(2),' *m', lw,2);
      else
        [X,Y,Z] = plot_xyz(np,p);
        [ViewAz,ViewEl] = view();
        plot3(X0,Y0,Z0,'-og', X,Y,Z,'-ob', ...
              et(1),et(2),et(3),' *r', pt(1),pt(2),pt(3),' *m', lw,2);
        view(ViewAz,ViewEl);
      end
      text(text_x,text_y,text_tsim(tsim),'fontweight','bold');
      drawnow;
      if Record == 1
        k = k + 1;
        if isOctave
          F(k,:) = putimage(fig1, k, FMT, Plot3D);
        else
          F(k) = getframe(fig1);
        end
      end
      ik = kfps; % reset iteration count till next plot update
    end
  end % end of IK iteration loop 
  
  % Display solution time and plot 
  error = norm(ec - et);
  fprintf('Iteration time (sec) = %8.3f\nEffector position error = %8.4f\n',...
                  tsim, error);
  if Plot3D == 0
    [X,Y] = plot_xy(np,p);
    plot([X0(na),et(1)],[Y0(na),et(2)],' -k', ...
          X0,Y0,'-og', X,Y,'-ob', et(1),et(2),' *r', lw,2);
    X0 = X; Y0 = Y;
  else  
    [X,Y,Z] = plot_xyz(np,p);
    [ViewAz,ViewEl] = view();
    plot3(X0,Y0,Z0,'-og', X,Y,Z,'-ob', et(1),et(2),et(3),' *r', lw,2);
    view(ViewAz,ViewEl)
    X0 = X; Y0 = Y; Z0 = Z;
  end
  text(text_x,text_y,text_tsim(tsim),'fontweight','bold');
  drawnow;
  if Record == 1
    k = k + 1;
    if isOctave
      F(k,:) = putimage(fig1, k, FMT, Plot3D);
    else
      F(k) = getframe(fig1);
    end
  end
  % Wait for button press
  [Xm,Ym,button] = ginput(1);
  if Plot3D == 0
    et(1) = min(max(Xm,-6),6);
    et(2) = min(max(Ym,-6),6);
    et(3) = 0;
  else
    inum = inum + 1;
    if inum > nnum 
      inum = 1;
    end
    theta = 2*pi*rnum(inum);
    r     = 1.5 + 2.5*rnum(inum);
    et(1) = r*cos(theta);
    et(2) = r*sin(theta);
    et(3) = 1 + 4*rnum(inum);
  end
  pt = et;
  % Reset iteration counter and simulation time
  ni   = 0;
  tsim = 0.0;
  tgo  = h;
end % end of button press loop

%% Animation playback

if Record == 1
  fprintf('IK_Solver - Playback of recorded movie frames\n');
  fprintf('On the figure, use the mouse buttons:\n');
  fprintf('  + left to replay the movie\n');
  fprintf('  + right to exit the program\n');
  figM = figure('Name','IK_Solver - Recorded Frames','NumberTitle','Off');
  grid off;
  button = 1;
  while button == 1
    if isOctave
      for k = 1:length(F)
        img = getimage(F(k,1:end));
        image(img, 'clipping', "on");
        axis("off");
        pause(1.0/FPS);
      end
    else
      axis off
      movie(figM,F);
    end
    [Xm,Ym,button] = ginput(1);
  end
  filename = sprintf('IK_Solver_%d.avi',IKmethod);
  filepath = sprintf('./images/%s',filename);
  fprintf('Recorded movie frames saved as file %s\n',filepath);
  if isOctave
    info = imfinfo(F(1,1:end));
    image2avi(FMT, FPS, info.Width, info.Height, filepath(1:end));
  else
    v = VideoWriter(filepath, 'Motion JPEG AVI');
    v.FrameRate = FPS;
    open(v);
    writeVideo(v,F);
    close(v);
  end
end

fprintf('Program exit.\n');
