# File: IK_Solver_client.py
# Auth: G. E. Deschaines
# Date: 27 Apr 2015
# Prog: Inverse Kinematics (IK) Solver client
# Desc: Applies user selected IK solving technique to determine and
#       plot the motion of a jointed n-link chain in 2D or 3D space.
#
# This Python program presents a 3D implementation of inverse kinematic
# solutions to end effector movement for a n-link, revolute-joint chain
# towards a target using cyclic coordinate descent (CCD) [1], Jacobian
# transpose method (JTM) [3], pseudo-inverse method (PIM) [1,2,3] or 
# damped least squares (DLS) [3] as detailed in the listed references.
#
# The program source is comprised of the following Python files.
#
#   IK_Solver_client.py - main routine and animation plot functions
#   IK_Solver_nlink.py  - IK solver n-link chain definition function
#   IK_Solver_class.py  - IK solver iteration and state data accessors 
#   IK_Solver_funcs.py  - IK solver math methods and support functions
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
from socket import *
from array  import *
from struct import *

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
     
# Sockets data packing/unpacking functions

def pack_mouse_bxy(buffer,b,x,y):
  ioff = 0
  pack_into('c',buffer,ioff,'M'.encode())
  ioff = ioff + calcsize('c')
  pack_into('I',buffer,ioff,b)
  ioff = ioff + calcsize('I')
  pack_into('ff',buffer,ioff,x,y)
  ioff = ioff + calcsize('ff')
  return ioff

def pack_target_pos(buffer,x,y,z):
  ioff = 0
  pack_into('c',buffer,ioff,'P'.encode())
  ioff = ioff + calcsize('c')
  pack_into('fff',buffer,ioff,x,y,z)
  ioff = ioff + calcsize('fff')
  return ioff

def pack_target_vel(buffer,vx,vy,vz):
  ioff = 0
  pack_into('c',buffer,ioff,'V'.encode())
  ioff = ioff + calcsize('c')
  pack_into('fff',buffer,ioff,vx,vy,vz)
  ioff = ioff + calcsize('fff')
  return ioff

def unpack_points_xy(buffer,ioff):
  (n,) = unpack_from('I',buffer,ioff)
  ioff = ioff + calcsize('I')
  X = np.zeros(n)
  Y = np.zeros(n)
  for i in range(0,n) :
    (X[i],Y[i])= unpack_from('ff',buffer,ioff)
    ioff = ioff + calcsize('ff')
  return (ioff, X, Y)
  
def unpack_points_xyz(buffer,ioff):
  (n,) = unpack_from('I',buffer,ioff)
  ioff = ioff + calcsize('I')
  X = np.zeros(n)
  Y = np.zeros(n)
  Z = np.zeros(n)
  for i in range(0,n) :
    (X[i],Y[i],Z[i])= unpack_from('fff',buffer,ioff)
    ioff = ioff + calcsize('fff')
  return (ioff, X, Y, Z)
  
def sendTgtVel(n):
  global Plot3D, tcpCliSock, BUFSIZ
  # Set target's velocit based on frame number.
  if Plot3D == 0 :
    tVel = np.array([-0.1, 0.0, 0.0])
  else :
    tVel = np.array([ 0.0,-0.2, 0.0])
  # Flush buffer
  buffer = array('B',(0 for i in range(0,16)))  
  # Pack target position and send
  ioff = pack_target_vel(buffer,tVel[0],tVel[1],tVel[2])
  tcpCliSock.send(buffer)
  # Wait for acknowledge
  data = tcpCliSock.recv(16)
  
def sendTgtPos(n):
  global Plot3D, tcpCliSock, BUFSIZ    
  # Set target's initial position
  if Plot3D == 0 :
    tPos = np.array([3.0,4.0,0.0])  
  else :
    tPos = np.array([3.0,2.5,2.0])    
  # Flush buffer
  buffer = array('B',(0 for i in range(0,16)))  
  # Pack target position and send
  ioff = pack_target_pos(buffer,tPos[0],tPos[1],tPos[2])
  tcpCliSock.send(buffer)  
  # Wait for acknowledge
  data = tcpCliSock.recv(16)
  
# Animation plotting functions.

def onclick(e):
  """
  button press event handler
  """
  tcpCliSock, BUFSIZ
  
  # Flush buffer
  buffer = array('B',(0 for i in range(0,16)))
  
  # Pack mouse event and send
  ioff = pack_mouse_bxy(buffer,e.button,e.xdata,e.ydata)
  tcpCliSock.send(buffer)
  
  # Wait for acknowledge
  data = tcpCliSock.recv(16)
  
def init2D():
  """
  init_func for 2D plotting
  """
  global line1, line2, line3, line4, line5, time_text
  
  line1.set_data([], [])
  line2.set_data([], [])
  line3.set_data([], [])
  line4.set_data([], [])
  line5.set_data([], [])
  time_text.set_text('')

  # return artists in drawing order
  return line5, line1, line3, line4, line2, time_text

def animate2D(n):
  """
  animation function for 2D plotting
  """
  global Server, tcpCliSock, BUFSIZ
  global ik_solver, tdel
  global line1, line2, line3, line4, line5, time_text
  
  # IK Solver update
  
  if Server == 0 :
    if n == 0 :
      ik_solver.set_target_pos( 3.0, 4.0, 0.0)
      ik_solver.set_target_vel(-0.1, 0.0, 0.0)
    ik_solver.step(tdel)
    if n == 0 :
      ik_solver.zero()
    time      = ik_solver.tsolve
    (X0,Y0)   = ik_solver.get_points_x0y0()
    (X,Y)     = ik_solver.get_points_xy()
    (XT,YT)   = ik_solver.get_target_xy()
    (XPT,YPT) = ik_solver.get_predtgt_xy()
  else :
    if n == 0 :
      sendTgtPos(n)
      sendTgtVel(n)
    data = 'update' + ' ' + str(n)
    tcpCliSock.send(data.encode())
    buffer = tcpCliSock.recv(BUFSIZ)
    ioff = 0
    (time,)= unpack_from('f',buffer,ioff)
    ioff = ioff + calcsize('f')
    (ioff,X0,Y0)   = unpack_points_xy(buffer,ioff)
    (ioff,X,Y)     = unpack_points_xy(buffer,ioff)
    (ioff,XT,YT)   = unpack_points_xy(buffer,ioff)
    (ioff,XPT,YPT) = unpack_points_xy(buffer,ioff)

  # Set line data and time text for plotting
  
  i   = np.size(X0) - 1
  X0PT = np.array([X0[i],XPT[0]])
  Y0PT = np.array([Y0[i],YPT[0]])
  
  line1.set_data(X0,Y0)
  line2.set_data(X,Y)
  line3.set_data(XT,YT)
  line4.set_data(XPT,YPT)
  line5.set_data(X0PT,Y0PT)
  time_text.set_text('time = %.3f' % time)

  # return artists in drawing order
  return line5, line1, line3, line4, line2, time_text

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

  # return artists in drawing order
  return line7, line8, line9, line4, line5,\
         line6, line1, line3, line2, time_text
         
def animate3D(n):
  """
  animation function for 3D plotting
  """
  global Server, tcpClisock, BUFSIZ
  global ik_solver, tdel
  global line1, line2, line3, line4, line5,\
         line6, line7, line8, line9, time_text
         
  # IK Solver update
  
  if Server == 0 :
    if n == 0 :
      ik_solver.set_target_pos( 3.0, 2.5, 2.0)
      ik_solver.set_target_vel( 0.0,-0.2, 0.0)
    ik_solver.step(tdel)
    if n == 0 :
      ik_solver.zero()
    time          = ik_solver.tsolve
    (X0,Y0,Z0)    = ik_solver.get_points_x0y0z0()
    (X,Y,Z)       = ik_solver.get_points_xyz()
    (XT,YT,ZT)    = ik_solver.get_target_xyz()
    (XE,YE,ZE)    = ik_solver.get_endeff_xyz()
    (XPT,YPT,ZPT) = ik_solver.get_predtgt_xyz()
  else :
    if n == 0 :
      sendTgtPos(n)
      sendTgtVel(n)
    data = 'update' + ' ' + str(n)
    tcpCliSock.send(data.encode())
    buffer = tcpCliSock.recv(BUFSIZ)
    ioff = 0
    (time,)= unpack_from('f',buffer,ioff)
    ioff = ioff + calcsize('f')
    (ioff,X0,Y0,Z0)    = unpack_points_xyz(buffer,ioff)
    (ioff,X,Y,Z)       = unpack_points_xyz(buffer,ioff)
    (ioff,XT,YT,ZT)    = unpack_points_xyz(buffer,ioff)
    (ioff,XE,YE,ZE)    = unpack_points_xyz(buffer,ioff)
    (ioff,XPT,YPT,ZPT) = unpack_points_xyz(buffer,ioff)

  # Set line data and time text for plotting
  
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
  XTP = np.array([XPT,XPT,ax.get_xlim()[0]])
  YTP = np.array([YPT,ax.get_ylim()[1],YPT])
  ZTP = np.array([ax.get_zlim()[0],ZPT,ZPT])
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
  time_text.set_text('time = %.3f' % time)

  # return artists in drawing order
  return line7, line8, line9, line4, line5,\
         line6, line1, line3, line2, time_text
         
  
# IK_Solver main.

if __name__ == '__main__':
  
  # Processing selectors
  
  Server   = 1  # use IK Solver server
  IKmethod = 0  # selected IK solver method
  Plot3D   = 0  # plot in 3D flag
  Record   = 0  # record movie flag
  
  # IK Server tcp connection parameters
  
  HOST   = '127.0.0.1'
  PORT   = 21000
  ADDR   = (HOST, PORT)
  BUFSIZ = 512
  
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
    elif sval == '6' :
      IKmethod = UsePIM3dH
      title = 'IK_Solver - Using Pseudo-Inverse Method with null space control [ref 3]'
    elif sval == '7' :
      IKmethod = UseDLSdH
      title = 'IK_Solver - Using Damped Least Squares with null space control [ref 3]'
    else :
      sval     = '0'
      IKmethod = 0
    
  # Define link chain structure and parameters.
  
  (a, u, q0, qmin, qmax, dqlim) = get_nlink_chain(Plot3D)
  
  # Instantiate an IK_Solver or connect to an IK Solver server.
  
  ik_server = 0
  if Server == 0 :
    tcpCliSock = 0
    ik_solver = IK_Solver(Plot3D,IKmethod,a,u,q0,qmin,qmax,dqlim)
  else :
    tcpCliSock = socket(family=AF_INET, type=SOCK_STREAM)
    try :
      tcpCliSock.connect(ADDR)
      data = 'start' + ' ' + str(Plot3D) + ' ' + str(IKmethod)
      tcpCliSock.send(data.encode())
      raw_data = tcpCliSock.recv(BUFSIZ)
      data     = raw_data.decode()
      print(data)
    except : 
      print('Could not connect to an IK Solver server.')
      print('Will use a locally instantiated IK Solver.')
      tcpCliSock = 0
      Server = 0
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
    line3 = Line2D([],[],color='r',ls=' ',lw=2.0,    # current target location
                   marker='x',mew=2.0,mec='r',mfc='r')
    line4 = Line2D([],[],color='m',ls=' ',lw=2.0,    # predicted target location
                   marker='x',mew=2.0,mec='m',mfc='m')
    line5 = Line2D([],[],color='k',ls=':',lw=1.0,    # eff to predicted tgt line
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
    ax.add_line(line5)
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
    line6 = Line3D([],[],[],color='m',ls=' ',lw=1.0,  # predicted target in XYZ planes
                   marker='x',mew=1.0,mec='m',mfc='m')
    line7 = Line3D([],[],[],color='k',ls=':',lw=1.0,  # eff to pred. tgt line in XY plane
                   marker=' ')
    line8 = Line3D([],[],[],color='k',ls=':',lw=1.0,  # eff to pred. tgt line in XZ plane
                   marker=' ')
    line9 = Line3D([],[],[],color='k',ls=':',lw=1.0,  # eff to pred. tgt line in YZ plane
                   marker=' ')                                                             
    time_text = ax.text(-6.0, -7.0, 6.5, '',)
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
  print("tdel_msec = %8.3f" % tdel_msec)
  print("t1 - t0   = %8.3f" % tstep)
  print("interval  = %8.3f" % interval)

  # Assign mouse button press handler.
  
  if Server == 0 :
    cid = fig.canvas.mpl_connect('button_press_event', ik_solver.onclick)
  else :
    cid = fig.canvas.mpl_connect('button_press_event', onclick)
  
  # Specify animation parameters and assign functions.
   
  nframes = None
  blit    = False  # blitting not working well with Matplotlib v1.5.1+
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
  
  if ik_server :
    del ik_server
  if tcpCliSock :
    prompt = 'Do you want to stop the IK_Solver server [Y|y]? '
    sval = raw_input(prompt)
    if sval == 'Y' or sval == 'y' :   
      data = 'quit'
    else :
      data = 'stop'
    tcpCliSock.send(data.encode())
    tcpCliSock.close()
