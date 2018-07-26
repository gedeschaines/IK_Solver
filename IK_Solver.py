# File: IK_Solver.py
# Auth: G. E. Deschaines
# Date: 17 Apr 2015
# Prog: Inverse Kinematics (IK) Solver
# Desc: Applies user selected IK solving technique to determine and
#       plot the motion of a revolute jointed n-link chain in 2D or
#       3D space.
#
# This Python program presents a 3D implementation of inverse kinematic
# solutions to end effector movement for a n-link, revolute-joint chain
# towards a target using cyclic coordinate descent (CCD) [1], Jacobian
# transpose method (JTM) [3], pseudo-inverse method (PIM) [1,2,3] or 
# damped least squares (DLS) [3] as detailed in the listed references.
#
# The program source is comprised of the following Python files.
#
#   IK_Solver.py       - main routine and animation plot functions
#   IK_Solver_nlink.py - IK solver n-link chain definition function
#   IK_Solver_class.py - IK solver iteration and state data accessors 
#   IK_Solver_funcs.py - IK solver math methods and support functions
#
# The IK solution technique is user selectable, and the program can be 
# configured to record the plotted link chain motion to a video file 
# for animation playback (requires ffmpeg or avconv).
#
# References:
#
# [1] Benjamin Kenwright, "Practical 01: Inverse Kinematics", September
#     2014, School of Computer Science, Edinburgh Napier University, 
#     United Kingdom, Physics-Based Animation practical web available at
#     http://games.soc.napier.ac.uk/study/pba_practicals/Practical%2001%20-%20Inverse%20Kinematics.pdf
#
# [2] Ben Kenwright, "Real-Time Character Inverse Kinematics using
#     the Gauss-Seidel Iterative Approximation Method", July 2012, 
#     CONTENT 2012, The Fourth International Conference on Creative 
#     Content Technologies, web available at
#     http://www.thinkmind.org/index.php?view=article&articleid=content_2012_4_10_60013
#
# [3] Samuel R. Buss, "Introduction to Inverse Kinematics with Jacobian
#     Transpose, Pseudoinverse and Damped Least Squares methods", October
#     2009, Department of Mathematics, University of California, San
#     Diego, unpublished article web available at
#     http://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
#
# Disclaimer:
#
# See DISCLAIMER

import sys
import os
import time

from math import ceil

import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

try:
  import numpy        as np
  import scipy.linalg as la
except ImportError:
  print("* Error: NumPy and SciPy packages required.")
  print("         Suggest installing the SciPy stack.")
  sys.exit()

try:
  import matplotlib.pyplot          as plt
  import matplotlib.animation       as animation
  from   matplotlib.lines           import Line2D
  from   mpl_toolkits.mplot3d.art3d import Line3D
except ImportError:
  print("* Error: matplotlib package required.")
  print("         Suggest installing the SciPy stack.")
  sys.exit()

try:
  from IK_Solver_nlink import *
  from IK_Solver_class import *
except ImportError:
  print("* Error: IK_Solver_class.py required.")
  sys.exit()
  
# Handle Python 2.x and 3.x raw_input.

try:
  type(raw_input)
except NameError:
  def raw_input(prompt):
     return input(prompt)
     
# Animation plotting functions.

def init2D():
  """
  init_func for 2D plotting
  """
  global line1, line2, line3, line4, time_text
  
  line1.set_data([], [])
  line2.set_data([], [])
  line3.set_data([], [])
  line4.set_data([], [])
  time_text.set_text('')
  return line4, line1, line3, line2, time_text

def animate2D(n):
  """
  animation function for 2D plotting
  """
  global ik_solver, tdel
  global line1, line2, line3, line4, time_text
  
  if n == 0 :
    ik_solver.set_target_pos( 3.0, 4.0, 0.0)
    ik_solver.set_target_vel(-0.1, 0.0, 0.0)
    
  ik_solver.step(tdel)
  if n == 0 :
    ik_solver.zero()
    
  (X0,Y0) = ik_solver.get_points_x0y0()
  (X,Y)   = ik_solver.get_points_xy()
  (XT,YT) = ik_solver.get_target_xy()
  
  i   = np.size(X0) - 1
  X0T = np.array([X0[i],XT[0]])
  Y0T = np.array([Y0[i],YT[0]])
  line1.set_data(X0,Y0)
  line2.set_data(X,Y)
  line3.set_data(XT,YT)
  line4.set_data(X0T,Y0T)
  time_text.set_text('time = %.3f' % ik_solver.tsolve)
  return line4, line1, line3, line2, time_text

def init3D():
  """
  init_func for 3D plotting
  """
  global line1, line2, line3, line4, line5,\
         line6, line7, line8, line9, time_text

  line1.set_data([], [])
  line1.set_3d_properties([])
  line2.set_data([], [])
  line2.set_3d_properties([])
  line3.set_data([], [])
  line3.set_3d_properties([])
  line4.set_data([], [])
  line4.set_3d_properties([])
  line5.set_data([], [])
  line5.set_3d_properties([])
  line6.set_data([], [])
  line6.set_3d_properties([])
  line7.set_data([], [])
  line7.set_3d_properties([])
  line8.set_data([], [])
  line8.set_3d_properties([])
  line9.set_data([], [])
  line9.set_3d_properties([]) 
  time_text.set_text('')
  return line7, line8, line9, line4, line5,\
         line6, line1, line3, line2, time_text

def animate3D(n):
  """
  animation function for 3D plotting
  """
  global ik_solver, tdel
  global line1, line2, line3, line4, line5,\
         line6, line7, line8, line9, time_text
         
  if n == 0 :
    ik_solver.set_target_pos( 3.0, 2.5, 2.0)
    ik_solver.set_target_vel( 0.0,-0.2, 0.0)
    
  ik_solver.step(tdel)
  if n == 0 :
    ik_solver.zero()
    
  (X0,Y0,Z0) = ik_solver.get_points_x0y0z0()
  (X,Y,Z)    = ik_solver.get_points_xyz()
  (XT,YT,ZT) = ik_solver.get_target_xyz()
  (XE,YE,ZE) = ik_solver.get_endeff_xyz()
  
  line1.set_data(X0,Y0)
  line1.set_3d_properties(Z0)
  line2.set_data(X,Y)
  line2.set_3d_properties(Z)
  line3.set_data(XT,YT)
  line3.set_3d_properties(ZT)
  i   = np.size(X0) - 1
  X0P = np.array([X0[i],X0[i],ax.get_xlim()[0]])
  Y0P = np.array([Y0[i],ax.get_ylim()[1],Y0[i]])
  Z0P = np.array([ax.get_zlim()[0],Z0[i],Z0[i]])
  line4.set_data(X0P,Y0P)
  line4.set_3d_properties(Z0P) 
  XEP = np.array([XE,XE,ax.get_xlim()[0]])
  YEP = np.array([YE,ax.get_ylim()[1],YE])
  ZEP = np.array([ax.get_zlim()[0],ZE,ZE])
  line5.set_data(XEP,YEP)
  line5.set_3d_properties(ZEP)
  XTP = np.array([XT,XT,ax.get_xlim()[0]])
  YTP = np.array([YT,ax.get_ylim()[1],YT])
  ZTP = np.array([ax.get_zlim()[0],ZT,ZT])
  line6.set_data(XTP,YTP)
  line6.set_3d_properties(ZTP)
  X0TP0 = np.array([X0P[0],XTP[0]])
  Y0TP0 = np.array([Y0P[0],YTP[0]])
  Z0TP0 = np.array([Z0P[0],ZTP[0]])
  line7.set_data(X0TP0,Y0TP0)
  line7.set_3d_properties(Z0TP0)
  X0TP1 = np.array([X0P[1],XTP[1]])
  Y0TP1 = np.array([Y0P[1],YTP[1]])
  Z0TP1 = np.array([Z0P[1],ZTP[1]])
  line8.set_data(X0TP1,Y0TP1)
  line8.set_3d_properties(Z0TP1)
  X0TP2 = np.array([X0P[2],XTP[2]])
  Y0TP2 = np.array([Y0P[2],YTP[2]])
  Z0TP2 = np.array([Z0P[2],ZTP[2]])
  line9.set_data(X0TP2,Y0TP2)
  line9.set_3d_properties(Z0TP2)  
  time_text.set_text('time = %.3f' % ik_solver.tsolve)
  return line7, line8, line9, line4, line5,\
         line6, line1, line3, line2, time_text

# IK_Solver main.

if __name__ == '__main__':
  
  # Processing selectors
  
  IKmethod = 0  # selected IK solver method
  Plot3D   = 0  # plot in 3D flag
  Record   = 0  # record movie flag
  
  # Check program arguments for IK method, Plot3D and Record values.
  
  sval = '0'
  if ( len(sys.argv) > 1 ) :
    sval = sys.argv[1]
  if ( len(sys.argv) > 2 ) :
    Plot3D = eval(sys.argv[2])
  if ( len(sys.argv) > 3 ) :
    Record = eval(sys.argv[3])
  
  # Request IK method selection (if not already given).
  
  prompt = 'Select IK Solver (1=CCD,2=JTM,3=PIM2,4=PIM3,5=DLS)? '
  while IKmethod == 0 :
    if sval == '0' :
      sval = raw_input(prompt)
    if sval == '1' :
      IKmethod = UseCCD
      title = 'IK_Solver - Cyclic Coordinate Descent'  
    elif sval == '2' :
      IKmethod = UseJTM
      title = 'IK_Solver - Using Jacobian Transpose Method'
    elif sval == '3' :
      IKmethod = UsePIM2
      title = 'IK_Solver - Using Pseudo-Inverse Method [ref 2]'
    elif sval == '4' :
      IKmethod = UsePIM3
      title = 'IK_Solver - Using Pseudo-Inverse Method [ref 3]'
    elif sval == '5' :
      IKmethod = UseDLS
      title = 'IK_Solver - Using Damped Least Squares [ref 3]'
    else :
      sval     = '0'
      IKmethod = 0
      
  # Define link chain structure and parameters.
  
  (a, u, q0, qmin, qmax, dqlim) = get_nlink_chain(Plot3D)      
  
  # Instantiate an IK_Solver.
  
  ik_solver = IK_Solver(Plot3D,IKmethod,a,u,q0,qmin,qmax,dqlim)
  
  # If recording animation, assign an animation writer.
  
  Writer = None
  if Record == 1 :
    try:
      Writer = animation.writers['ffmpeg']
    except KeyError:
      try:
        Writer = animation.writers['avconv']
      except KeyError:
        print("Cannot record animation, no animation writers available!")
        print("Try installing ffmpeg or avconv.")
        Record = 0
  
  # Instantiate a matplotlib pyplot figure.
  
  if Record == 0 :
    fig = plt.figure(figsize=(8,6), dpi=80)
  else :
    fig = plt.figure(figsize=(6,4), dpi=80)
    
  # Specify plotting objects and parameters.
  
  if Plot3D == 0 :
    ax = fig.add_subplot(111, autoscale_on=False)
    line1 = Line2D([],[],color='g',ls='-',lw=1.0,    # link chain at start
                   marker='o',mew=1.0,mec='k',mfc='g')
    line2 = Line2D([],[],color='b',ls='-',lw=2.0,    # link chain in motion
                   marker='o',mew=1.0,mec='k',mfc='b')
    line3 = Line2D([],[],color='r',ls=' ',lw=2.0,    # target location
                   marker='x',mew=2.0,mec='r',mfc='r')
    line4 = Line2D([],[],color='k',ls=':',lw=1.0,    # eff to tgt line
                   marker=' ')
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
    ax.set_title(title)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_xlim(-6,6)
    ax.set_ylim(-6,6)
    ax.set_aspect('equal')
    ax.grid()
    ax.add_line(line1)
    ax.add_line(line2)
    ax.add_line(line3)
    ax.add_line(line4)
    fig.canvas.draw()
  else :
    ax = fig.add_subplot(111, projection='3d')
    line1 = Line3D([],[],[],color='g',ls='-',lw=1.0,  # link chain start in 3D
                   marker='o',mew=1.0,mec='k',mfc='g')  
    line2 = Line3D([],[],[],color='b',ls='-',lw=2.0,  # link chain motion in 3D
                   marker='o',mew=1.0,mec='k',mfc='b')  
    line3 = Line3D([],[],[],color='r',ls=' ',lw=2.0,  # target in 3D space
                   marker='x',mew=2.0,mec='r',mfc='r')
    line4 = Line3D([],[],[],color='g',ls=' ',lw=1.0,  # eff start in XYZ planes
                   marker='o',mew=1.0,mec='g',mfc='w')                           
    line5 = Line3D([],[],[],color='b',ls=' ',lw=1.0,  # eff motion in XYZ planes
                   marker='o',mew=1.0,mec='b',mfc='w')                          
    line6 = Line3D([],[],[],color='r',ls=' ',lw=1.0,  # target in XYZ planes 
                   marker='x',mew=1.0,mec='r',mfc='r')
    line7 = Line3D([],[],[],color='k',ls=':',lw=1.0,  # eff to tgt line in XY plane 
                   marker=' ')
    line8 = Line3D([],[],[],color='k',ls=':',lw=1.0,  # eff to tgt line in XZ plane
                   marker=' ')
    line9 = Line3D([],[],[],color='k',ls=':',lw=1.0,  # eff to tgt line in YZ plane 
                   marker=' ')                                                             
    time_text = ax.text3D(-6.0, -7.0, 6.5, '')
    ax.set_title(title)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim3d([-5,5])
    ax.set_ylim3d([-5,5])
    ax.set_zlim3d([ 0,5])
    #ax.set_aspect('equal')
    ax.grid()
    ax.add_line(line1)
    ax.add_line(line2)
    ax.add_line(line3)
    ax.add_line(line4)
    ax.add_line(line5)
    ax.add_line(line6)
    ax.add_line(line7)
    ax.add_line(line8)
    ax.add_line(line9)
    fig.canvas.draw()

  # Determine animation interval time.
   
  FPS       = 10
  tdel      = 1.0/FPS
  tdel_msec = 1000.0*tdel
  tmsec0    = 1000.0*time.clock()
  if Plot3D == 0 :
    animate2D(0)
  else :
    animate3D(0)
  tmsec1    = 1000.0*time.clock()
  tstep     = tmsec1 - tmsec0
  interval  = ceil(tmsec1-tmsec0)  # allows faster than real-time
  #interval  = tdel_msec - tstep   # approximates real-time
  """
  print("tdel_msec = %8.3f" % tdel_msec)
  print("t1 - t0   = %8.3f" % tstep)
  print("interval  = %8.3f" % interval)
  """
  
  # Assign mouse button press handler.
  
  cid = fig.canvas.mpl_connect('button_press_event', ik_solver.onclick)
  
  # Specify animation parameters and assign functions.
   
  nframes = None
  blit    = False  # blitting not working good with Matplotlib v1.5.1+
  if Record == 1 :
    nframes = 14*FPS
    blit    = False
  
  if Plot3D == 0 :
    anim = animation.FuncAnimation(fig, animate2D, init_func=init2D,
                                   frames=nframes, blit=blit,  
                                   interval=interval, repeat=False)
  else :
    anim = animation.FuncAnimation(fig, animate3D, init_func=init3D,
                                   frames=nframes, blit=blit,
                                   interval=interval, repeat=False)

  # Begin.

  if Record == 0 or Writer is None:
    if Plot3D == 0 :
      print("Move cursor into plot region and press middle mouse button")
      print("to initiate IK solver for selected target location.")
    else :
      print("With cursor in the plot region, press middle mouse button")
      print("to initiate IK solver for randomly positioned target.")
    plt.show()  
  else :
    plt.ioff()
    print("Creating animation video; presentation will begin shortly ...")
    writer = Writer(fps=FPS, codec='libx264',
                    metadata=dict(artist='IK_Solver'),
                    bitrate=-1)
    print("Performing IK solver iteration...")
    filename = 'IK_Solver_' + str(IKmethod) + '.mp4'
    anim.save(filename, writer=writer)
    print("Animation saved in file %s" % filename)
    plt.show()

  # Terminate and exit.
  
  del ik_solver
