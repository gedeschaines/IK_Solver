# File: IK_Solver_server.py
# Auth: G. E. Deschaines
# Date: 17 Apr 2015
# Prog: Inverse Kinematics (IK) Solver server
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
#   IK_Solver_server.py - main routine and socket connection functions
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

try:
  import numpy        as np
  import scipy.linalg as la
except ImportError:
  print("* Error: NumPy and SciPy packages required.")
  print("         Suggest installing the SciPy stack.")
  sys.exit()

try:
  from IK_Solver_nlink import *
  from IK_Solver_class import *
except ImportError:
  print("* Error: IK_Solver_class.py required.")
  sys.exit()

def pack_points_xy(buffer,ioff,n,X,Y):
  
  pack_into('I',buffer,ioff,n)
  ioff = ioff + calcsize('I')
  for i in range(0,n) :
    pack_into('ff',buffer,ioff,X[i],Y[i])
    ioff = ioff + calcsize('ff')
  return ioff

def pack_points_xyz(buffer,ioff,n,X,Y,Z):
  
  pack_into('I',buffer,ioff,n)
  ioff = ioff + calcsize('I')
  for i in range(0,n) :
    pack_into('fff',buffer,ioff,X[i],Y[i],Z[i])
    ioff = ioff + calcsize('fff')
  return ioff

def pack_joints_rot(buffer,ioff,n,Q):
  
  pack_into('I',buffer,ioff,n)
  ioff = ioff + calcsize('I')
  for i in range(0,n) :
    pack_into('f',buffer,ioff,Q[i])
    ioff = ioff + calcsize('f')
  return ioff

def unpack_mouse_bxy(buffer,ioff):
  
  (b,) = unpack_from('I',buffer,ioff)
  ioff = ioff + calcsize('I')
  (x,y)= unpack_from('ff',buffer,ioff)
  ioff = ioff + calcsize('ff')
  return (ioff, b, x, y)
  
def unpack_target_pos(buffer,ioff):
  (x,y,z)= unpack_from('fff',buffer,ioff)
  ioff = ioff + calcsize('fff')
  return (ioff, x, y, z)
  
def unpack_target_vel(buffer,ioff):
  (vx,vy,vz)= unpack_from('fff',buffer,ioff)
  ioff = ioff + calcsize('fff')
  return (ioff, vx, vy, vz)
  
def ik_update2D(i):
  """
  IK solver update function for 2D plotting
  """
  global BUFSIZ, ik_solver, tdel
  
  ik_solver.step(tdel)
  if i == 0 :
    ik_solver.zero()
    
  # Get data from IK solver
  time      = ik_solver.tsolve
  (X0,Y0)   = ik_solver.get_points_x0y0()
  (X,Y)     = ik_solver.get_points_xy()
  (XT,YT)   = ik_solver.get_target_xy()
  (XPT,YPT) = ik_solver.get_predtgt_xy()
  # Flush buffer
  buffer = array('B',(0 for i in range(0,BUFSIZ)))
  
  # Pack data to send to client
  ioff = 0
  pack_into('f',buffer,ioff,time)
  ioff = ioff + calcsize('f')
  n = np.size(X0)
  ioff = pack_points_xy(buffer,ioff,n,X0,Y0)
  n = np.size(X)
  ioff = pack_points_xy(buffer,ioff,n,X,Y)
  n = np.size(XT)
  ioff = pack_points_xy(buffer,ioff,n,XT,YT)
  n = np.size(XPT)
  ioff = pack_points_xy(buffer,ioff,n,XPT,YPT)
  return (ioff, buffer)
  
def ik_update3D(i):
  """
  IK solver update function for 3D plotting
  """
  global BUFSIZ, ik_solver, tdel
  
  ik_solver.step(tdel)
  if i == 0 :
    ik_solver.zero()
    
  # Get data from IK solver
  time          = ik_solver.tsolve
  (X0,Y0,Z0)    = ik_solver.get_points_x0y0z0()
  (X,Y,Z)       = ik_solver.get_points_xyz()
  (XT,YT,ZT)    = ik_solver.get_target_xyz()
  (XE,YE,ZE)    = ik_solver.get_endeff_xyz()
  (XPT,YPT,ZPT) = ik_solver.get_predtgt_xyz()

  # Flush buffer
  buffer = array('B',(0 for i in range(0,BUFSIZ)))
  
  # Pack data to send to client
  ioff = 0
  pack_into('f',buffer,ioff,time)
  ioff = ioff + calcsize('f')
  n = np.size(X0)
  ioff = pack_points_xyz(buffer,ioff,n,X0,Y0,Z0)
  n = np.size(X)
  ioff = pack_points_xyz(buffer,ioff,n,X,Y,Z)
  n = np.size(XT)
  ioff = pack_points_xyz(buffer,ioff,n,XT,YT,ZT)
  n = np.size(XE)
  ioff = pack_points_xyz(buffer,ioff,n,XE,YE,ZE)
  n = np.size(XPT)
  ioff = pack_points_xyz(buffer,ioff,n,XPT,YPT,ZPT)
  return (ioff, buffer)

def ik_updateBL(i):
  """
  IK solver update function for Blender animation
  """
  global BUFSIZ, ik_solver, tdel
  
  ik_solver.step(tdel)
  if i == 0 :
    ik_solver.zero()
    
  # Get data from IK solver
  time       = ik_solver.tsolve
  (XT,YT,ZT) = ik_solver.get_target_xyz()
  (XE,YE,ZE) = ik_solver.get_endeff_xyz()
  (Q)        = ik_solver.get_joints_rot()
  
  # Flush buffer
  buffer = array('B',(0 for i in range(0,BUFSIZ)))
  
  # Pack data to send to client
  ioff = 0
  pack_into('f',buffer,ioff,time)
  ioff = ioff + calcsize('f')
  n = np.size(XT)
  ioff = pack_points_xyz(buffer,ioff,n,XT,YT,ZT)
  n = np.size(XE)
  ioff = pack_points_xyz(buffer,ioff,n,XE,YE,ZE)
  n = np.size(Q)
  ioff = pack_joints_rot(buffer,ioff,n,Q)
  
  return (ioff, buffer)

# IK_Solver server main.

if __name__ == '__main__':
  
  # Processing selectors
  
  IKmethod = 1  # selected IK solver method
  Plot3D   = 0  # plot in 3D flag
  
  # TCP/IP sockets parameters
  
  HOST   = '127.0.0.1'
  PORT   = 21000
  ADDR   = (HOST, PORT)
  BUFSIZ = 512
  
  buffer = array('B',(0 for i in range(0,BUFSIZ)))
  
  tcpSerSock = socket(family=AF_INET, type=SOCK_STREAM)
  tcpSerSock.bind(ADDR)
  tcpSerSock.listen(5)
  
  FPS       = 10
  tdel      = 1.0/FPS
  ik_solver = 0
  
  # Main processing loop over client connections
  
  quit = False
  while not quit:
    print('waiting for connection...')
    tcpCliSock, addr = tcpSerSock.accept()
    print("...connected from: %s:%d" % (addr[0], addr[1]))
    # Client session initiated.
    while True :
      raw_data = tcpCliSock.recv(BUFSIZ)
      if not raw_data :
        break
      #print('recv:', raw_data)
      if raw_data[0:5] == 'start'.encode('utf-8') :
        data = raw_data.decode('utf-8').strip()
        (s, p1, p2) = data.split(' ',2)
        Plot3D   = int(p1)
        IKmethod = int(p2)
        # define link chain structure and parameters.
        (a, u, q0, qmin, qmax, dqlim) = get_nlink_chain(Plot3D)
        # instantiate an IK_Solver.
        ik_solver = IK_Solver(Plot3D,IKmethod,a,u,q0,qmin,qmax,dqlim)
        data = 'Starting IK_Solver (Plot3D=' + p1 + ',IKmethod=' + p2 + ')'
        print(data)
        tcpCliSock.send(data.encode())
      elif raw_data[0:6] == 'update'.encode('utf-8') :
        data = raw_data.decode('utf-8').strip()
        (s, p1) = data.split(' ',1)
        i = int(p1)
        if Plot3D == 0 :
          (n, data) = ik_update2D(i)
        elif Plot3D == 1 :
          (n, data) = ik_update3D(i)
        elif Plot3D == 2 :
          (n, data) = ik_updateBL(i)
        tcpCliSock.send(data)
      elif raw_data[0] == 'M'.encode('utf-8')[0] :
        (ioff,b,x,y) = unpack_mouse_bxy(raw_data,1)
        ik_solver.mouse_click(b,x,y)
        data = 'ack'
        tcpCliSock.send(data.encode())
      elif raw_data[0] == 'P'.encode('utf-8')[0] :
        (ioff,x,y,z) = unpack_target_pos(raw_data,1)
        ik_solver.set_target_pos(x,y,z)
        data = 'ack'
        tcpCliSock.send(data.encode())
      elif raw_data[0] == 'V'.encode('utf-8')[0] :
        (ioff,vx,vy,vz) = unpack_target_vel(raw_data,1)
        ik_solver.set_target_vel(vx,vy,vz)
        data = 'ack'
        tcpCliSock.send(data.encode())
      elif raw_data[0:4] == 'stop'.encode('utf-8') :
        print('Stopping IK_Solver')
        del ik_solver
        ik_solver = 0
        break
      elif raw_data[0:4] == 'quit'.encode('utf-8') :
        print('Stopping IK_Solver')
        del ik_solver
        ik_solver = 0
        quit = True
        break
      else:
        print('Unknown message received from client:', raw_data)
      raw_data = ''
           
    # Client session concluded.
    tcpCliSock.close()
    if ik_solver != 0 :
      print('Stopping IK_Solver')
      del ik_solver
      ik_solver = 0
      
  # Terminate processing and exit.
  
  tcpSerSock.close()
