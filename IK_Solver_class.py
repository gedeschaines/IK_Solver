# File: IK_Solver_class.py
# Auth: G. E. Deschaines
# Date: 27 Apr 2015
# Prog: Inverse Kinematics (IK) Solver class
# Desc: Applies user selected IK solving technique to determine and
#       plot the motion of a jointed n-link chain in 2D or 3D space.
#
# This file provides the following IK_Solver class methods intended
# for public use 
#
#   __init__          - IK_Solver class object constructor
#   step              - performs IK solver iteration time step
#   zero              - sets IK solver state to initial conditions
#   set_target_pos    - sets target position [x,y,z] coordinates
#   set_target_vel    - sets target velocity [x,y,z] components
#   get_points_x0y0   - gets initial link coordinates for 2D plots
#   get_points_x0y0z0 - gets initial link coordinates for 3D plots
#   get_points_xy     - gets current link coordinates for 2D plots
#   get_points_xyz    - gets current link coordinates for 3D plots
#   get_target_xy     - gets current target coordinates for 2D plots
#   get_target_xyz    - gets current target coordinates for 3D plots
#   get_predtgt_xy    - gets predicted target coordinates for 2D plots
#   get_predtgt_xyz   - gets predicted target coordinates for 3D plots
#   get_endeff_xyz    - gets current end effector coordinates for 3D plots
#   get_endeff_rot    - gets current end effector rotation about its z-axis
#   get_endeff_ypr    - gets current end effector yaw,pitch,roll rotations
#   get_joints_rot    - gets current joints rotation angle
#   mouse_click       - sets target position based on mouse click 
#   onclick           - mouse button press handler
#
# and these methods intended only for private use.
#
#   done               - applies IK solver completion criteria
#   null_space_control - computes null space control vector
#   set_points_x0y0z0  - sets starting link coordinates for plotting
#
# Disclaimer:
#
# See DISCLAIMER

import sys
import random

from math import atan,cos,sin,floor

try:
  import numpy        as np
  import scipy.linalg as la
except ImportError:
  print("* Error: NumPy and SciPy packages required.")
  print("         Suggest installing the SciPy stack.")
  sys.exit()

try:
  from IK_Solver_funcs import *
except ImportError:
  print("* Error: IK_Solver_funcs.py required.")
  sys.exit()
  
UseCCD    = 1  # use cyclic coordinate descent (CCD) from ref [1]
UseJTM    = 2  # use jacobian transpose method from ref [1,2]
UsePIM2   = 3  # use pseudo-inverse method from ref [2]
UsePIM3   = 4  # use pseudo-inverse method from ref [3]
UseDLS    = 5  # use damped least squares from ref [3]
UsePIM3dH = 6  # use pseudo-inverse method from ref [3] with null space control
UseDLSdH  = 7  # use damped least squares from ref [3] with null space control

class IK_Solver:
  
  def __init__(self,p3d,ik,a,u,q0,qmin,qmax,dqlim):
    """
    IK_Solver class - object constructor
    """
    self.Plot3D   = p3d    # plot mode (0=2D, 1=3D)
    self.IKmethod = ik     # selected IK solver method
    self.a        = a      # set of link joint attachment location vectors
    self.u        = u      # set of joint rotation axis unit vectors
    self.q0       = q0     # vector of initial joint rotations (radians)
    self.qmin     = qmin   # vector of joint minimum rotations (radians)
    self.qmax     = qmax   # vector of joint maximum rotations (radians)
    self.dqlim    = dqlim  # joints delta rotation limit (radians)
    
    self.na = len(self.a)         # number of link body attachment points
    self.ia = self.na - 1         # index of end effector attachment point
    self.nq = len(self.q0)        # number of link chain joints
    self.s  = np.zeros(self.nq)   # create array of link sizes
    for i in range(0,self.na-1):
      self.s[i] = la.norm(a[i+1])
    self.efd = self.s[-1]         # size of end effector link

    # generalizations used in distance checks of iteration done function;
    # assumes joint rotation limits do not prevent a full linear extension.
    if p3d == 0:
      self.dxy = np.sum(self.s[0:])  # length of fully extended chain in xy plane
      self.dz  = 0.0                 # length of fully extended chain in z direction
    else:
      # following assumes first link strictly in the z direction
      self.dxy = np.sum(self.s[1:])  # length of fully extended chain in xy plane
      self.dz  = np.sum(self.s[0:])  # length of fully extended chain in z direction

    self.q  = self.q0            # vector of current joint rotations
    self.dq = np.zeros(self.nq)  # vector of current joint delta rotations
    
    # transform joint positions/rotations from link body coordinates 
    # to world space
    (self.p,self.w) = transform(self.na,self.q,self.u,self.a)
     
    # get end effector current position vector
    self.ec = self.p[self.ia]  
     
    # specify end effector target position and velocity
    if self.Plot3D == 0 :
      self.et0 = np.array([ 3.0, 4.0, 0.0])  # position vector
      self.vt0 = np.array([-0.1, 0.0, 0.0])  # velocity vector
    else :
      self.et0 = np.array([ 3.0, 2.5, 2.0])  # position vector
      self.vt0 = np.array([ 0.0,-0.2, 0.0])  # velocity vector
    self.et = self.et0  # end effector target position in world space
    self.vt = self.vt0  # velocity of target in world space
    self.pt = self.et0  # predicted target position in world space

    self.h      = 0.04                      # IK iteration step size
    self.ilim   = int(30.0/self.h)          # IK iteration limit for 30 seconds
    self.sdel   = 0.001                     # PIM singularity damping constant from ref [2]
    self.sfac   = 0.01                      # PIM singularity threshold factor from ref [3]
    self.slam   = 1.1                       # DLS singularity damping factor from ref [3]
    self.dH     = np.zeros(self.nq)         # null space control vector
    self.derr   = 0.04*self.efd             # allowable effector to target distance error
    self.perr   = atan(self.derr/self.efd)  # allowable effector to target xy pointing error
    self.tsolve = 0.0                       # time to solve
    self.tgo    = self.h                    # time to goal
    self.ni     = 0                         # number of iterations
    self.lastni = 0                         # save of last ni on done and print
    self.button = 0                         # mouse button pressed indicator
    self.X0     = np.zeros(len(self.p))     # link chain initial x coordinates
    self.Y0     = np.zeros(len(self.p))     # link chain initial y coordinates
    self.Z0     = np.zeros(len(self.p))     # link chain initial y coordinates
    self.set_points_x0y0z0()
  
    random.seed(987654321)
    
  def zero(self):
    """
    IK_Solver class - zero object state function
    """
    self.q          = self.q0
    self.dq         = np.zeros(self.nq)
    (self.p,self.w) = transform(self.na,self.q,self.u,self.a)
    self.ec         = self.p[self.ia]
    self.et         = self.et0
    self.vt         = self.vt0
    self.pt         = self.et0
    self.dq         = np.zeros(self.nq)
    self.dH         = np.zeros(self.nq)
    self.tsolve     = 0.0
    self.tgo        = self.h
    self.ni         = 0
    self.lastni     = 0
    self.button     = 0
    self.set_points_x0y0z0()

  def null_space_control(self):
    """
    IK solver class - null space control vector computation function
    """
    if self.Plot3D == 0 :
      '''
      print("dq  = %8.5f %8.5f %8.5f %8.5f" % \
              (self.dq[0],self.dq[1],self.dq[2],self.dq[4]) )
      '''
      avT1 = avelT_wrt_jointm(self.et,self.vt,self.w,self.p,self.dq,1)
      avT2 = avelT_wrt_jointm(self.et,self.vt,self.w,self.p,self.dq,2)
      avT3 = avelT_wrt_jointm(self.et,self.vt,self.w,self.p,self.dq,3)
      avT5 = avelT_wrt_jointm(self.et,self.vt,self.w,self.p,self.dq,5)
      avT  = np.array([avT1[2], avT2[2], avT3[2], 0.0, avT5[2]])
      '''
      print("avT = %8.5f %8.5f %8.5f %8.5f" % \
              (avT[0],avT[1],avT[2],avT[4]) )
      '''
      avE1 = avelE_wrt_jointm(self.w,self.p,self.dq,1)
      avE2 = avelE_wrt_jointm(self.w,self.p,self.dq,2)
      avE3 = avelE_wrt_jointm(self.w,self.p,self.dq,3)
      avE5 = avelE_wrt_jointm(self.w,self.p,self.dq,5)
      avE  = np.array([avE1[2], avE2[2], avE3[2], 0.0, avE5[2]])
      '''
      print("avE = %8.5f %8.5f %8.5f %8.5f" % \
              (avE[0],avE[1],avE[2],avE[4]) )
      '''
      self.dH[0] = avE[0]
      self.dH[1] = avE[1]
      self.dH[2] = avE[2]
      #self.dH[4] = avE[4]
              
      phiZ = self.get_endeff_rot()
      '''
      print("phiZ = %8.5f" % (phiZ*dpr))
      '''
      self.dH[self.nq-1] = phiZ
    else :
      '''
      print("dq  = %8.5f %8.5f %8.5f" % \
              (self.dq[0],self.dq[1],self.dq[2]) )
      '''
      avT1 = avelT_wrt_jointm(self.et,self.vt,self.w,self.p,self.dq,1)
      avT2 = avelT_wrt_jointm(self.et,self.vt,self.w,self.p,self.dq,2)
      avT3 = avelT_wrt_jointm(self.et,self.vt,self.w,self.p,self.dq,3)
      avT  = np.array([avT1[2], avT2[1], avT3[1], 0.0, 0.0])
      '''
      print("avT = %8.5f %8.5f %8.5f" % (avT[0],avT[1],avT[2]) )
      '''
      avE1 = avelE_wrt_jointm(self.w,self.p,self.dq,1)
      avE2 = avelE_wrt_jointm(self.w,self.p,self.dq,2)
      avE3 = avelE_wrt_jointm(self.w,self.p,self.dq,3)
      avE  = np.array([avE1[2], avE2[1], avE3[1], 0.0, 0.0])
      '''
      print("avE = %8.5f %8.5f %8.5f" % (avE[0],avE[1],avE[2]) )
      '''
      self.dH[0] = avE[0]
      self.dH[1] = avE[1]
      self.dH[2] = avE[2]
       
      (psi,theta,phi) = self.get_endeff_ypr()
      '''
      print("psi,theta,phi = %8.5f, %8.5f, %8.5f" % \
              (psi*dpr,theta*dpr,phi*dpr) )
      '''
      self.dH[self.nq-2] = -theta
      self.dH[self.nq-1] = -psi

  def done(self,ni):
    """
    IK Solver class - iteration done check function
    """
    if ni == self.ilim : return True
       
    etp   = self.et-self.p[0]    # vector from link chain base to target
    ecp   = self.ec-self.p[0]    # vector from link chain base to effector
    detxy = la.norm(etp[0:2])    # distance from base to target in xy plane
    detz  = abs(etp[2])          # distance from base to target in z direction
    decxy = la.norm(ecp[0:2])    # distance from base to effector in xy plane
    dotxy = np.dot(ecp[0:2],etp[0:2])/detxy  # portion of ecp along etp in xy plane
    
    if ((detxy - self.dxy) > self.derr) or ((detz - self.dz) > self.derr):
      # target currently beyond reach of effector; but is it
      # outside allowable distance or xy pointing error?
      if (abs(decxy - self.dxy) > self.derr) or \
         (np.arccos(dotxy/decxy) > self.perr) :
        return False
      print("Encountered done condition 1.")
    else :
      # target currently not beyond reach of effector; but is it
      # outside allowable distance?
      if (la.norm(self.et - self.ec) > self.derr) :
        return False
      print("Encountered done condition 2.")
    return True
    
  def step(self,tdel):
    """
    IK solver class - iteration time step function
    """
    if ( self.button == -1 ) : return
       
    icnt = max(floor(tdel/self.h),1)
    
    while icnt > 0 :
      if self.done(self.ni) :
        if self.lastni != self.ni:
          print("Iteration time (sec) = %8.3f" % (self.tsolve) )
          error = la.norm(self.ec - self.et)
          print("Effector position error = %8.4f" % (error) )
          if self.Plot3D == 0 :
            theta = self.get_endeff_rot()
            print("Effector rotation angle = %8.3f " % (theta*dpr) )
          else :
            (psi,theta,phi) = self.get_endeff_ypr()
            print("Effector yaw,pitch,roll = %8.3f, %8.3f, %8.3f" % \
                  (psi*dpr, theta*dpr, phi*dpr) )
          self.lastni = self.ni  # prevent done reprint of last IK solution
        self.button = -1
        icnt        = 0
      else :
        # Calculate IK solution for dq
        if self.IKmethod == UseCCD :
          # predicted target position at tgo relative to link
          gain = closing_gain(self.IKmethod, self.ec, self.et, self.vt)
          ept =  self.et + gain*self.tgo*self.vt
          self.dq = np.zeros(self.nq)
          for i in range(self.nq-1,-1,-1) :
            pt  =  ept - self.p[i]
            npt = la.norm(pt)
            pc  = self.ec - self.p[i]
            npc = la.norm(pc)
            ut  = np.cross(np.cross(self.w[i],pc/npc),\
                           np.cross(self.w[i],pt/npt))
            if la.norm(ut) > 0.0 :
              ut         = ut/la.norm(ut)
              dq         = np.arccos(np.dot(pt,vT(pc/npc))/npt)
              dqmax      = np.amin([dq,self.dqlim[i]])
              self.dq[i] = dqmax*(np.dot(ut,vT(self.w[i])))
            else :
              self.dq[i] = 0.0
        else :
          # error from effector to predicted target position at tgo
          gain = closing_gain(self.IKmethod, self.ec, self.et, self.vt)
          de = self.et + gain*self.tgo*self.vt - self.ec
          #de = self.et - self.ec
          #Jt = jacobian(self.nq,self.w,self.p,self.et+self.tgo*self.vt)
          Jc = jacobian(self.nq,self.w,self.p,self.ec)
          J  = Jc
          if self.IKmethod == UseJTM :
            self.dq = ik_jtm(J,de,self.dqlim)
          elif self.IKmethod == UsePIM2 :
            self.dq = ik_pim2(J,de,self.sdel,self.dqlim)  
          elif self.IKmethod == UsePIM3 :
            self.dq = ik_pim3(J,de,self.sfac,self.dqlim,self.dH)
          elif self.IKmethod == UseDLS :
            self.dq = ik_dls(J,de,self.slam,self.dqlim,self.dH)
          elif self.IKmethod == UsePIM3dH :
            # Use dH to coerce end-effector orientation
            self.null_space_control()
            self.dq = ik_pim3(J,de,self.sfac,self.dqlim,self.dH)
          elif self.IKmethod == UseDLSdH :
            # Use dH to coerce end-effector orientation
            self.null_space_control()
            self.dq = ik_dls(J,de,self.slam,self.dqlim,self.dH)
        # Update link chain angles and positions
        self.q = self.q + self.h*self.dq
        self.q = clamp_rot(self.nq,self.q,self.qmin,self.qmax)
        (self.p,self.w) = transform(self.na,self.q,self.u,self.a)
        #angle_chk(self.q,self.u,self.p,self.et)
        # Update end effector current and target positions
        self.ec  = self.p[self.ia]
        self.et  = self.et + self.h*self.vt
        self.tgo = time_to_goal(self.et,self.vt,self.w,self.p,self.dq)
        gain = closing_gain(self.IKmethod, self.ec, self.et, self.vt)
        self.pt = self.et + gain*self.tgo*self.vt
        # Increment iteration counters and time
        self.lastni = self.ni
        self.ni     = self.ni + 1
        self.tsolve = self.tsolve + self.h
        icnt        = icnt - 1
      
  def set_points_x0y0z0(self):
    """
    IK Solver - set initial position (X,Y,Z) coordinates
    """
    n = len(self.p)
    for i in range(n) :
      self.X0[i] = self.p[i][0]
      self.Y0[i] = self.p[i][1]
      self.Z0[i] = self.p[i][2]
      
  def set_target_pos(self, x, y, z):
    """
    IK Solver - set target position (X,Y,Z) coordinates
    """
    self.et[0] = x
    self.et[1] = y
    self.et[2] = z
    
  def set_target_vel(self, vx, vy, vz):
    """
    IK Solver - set target velocity (X,Y,Z) components
    """
    self.vt[0] = vx
    self.vt[1] = vy
    self.vt[2] = vz
    
  def get_points_x0y0(self):
    """
    IK Solver - get initial position (X,Y) coordinates
    """
    n  = len(self.p)
    X0 = np.zeros(n)
    Y0 = np.zeros(n)
    for i in range(n) :
      X0[i] = self.X0[i]
      Y0[i] = self.Y0[i]
    return (X0,Y0)
    
  def get_points_x0y0z0(self):
    """
    IK Solver - get initial position (X,Y,Z) coordinates
    """
    n  = len(self.p)
    X0 = np.zeros(n)
    Y0 = np.zeros(n)
    Z0 = np.zeros(n)
    for i in range(n) :
      X0[i] = self.X0[i]
      Y0[i] = self.Y0[i]
      Z0[i] = self.Z0[i]
    return (X0,Y0,Z0)
    
  def get_points_xy(self):
    """
    IK Solver - get current position (X,Y) coordinates
    """
    n = len(self.p)
    X = np.zeros(n)
    Y = np.zeros(n)
    for i in range(n) :
      X[i] = self.p[i][0]
      Y[i] = self.p[i][1]
    return (X,Y)
    
  def get_points_xyz(self):
    """
    IK Solver - get current position (X,Y,Z) coordinates
    """
    n = len(self.p)
    X = np.zeros(n)
    Y = np.zeros(n)
    Z = np.zeros(n)
    for i in range(n) :
      X[i] = self.p[i][0]
      Y[i] = self.p[i][1]
      Z[i] = self.p[i][2]
    return (X,Y,Z)
    
  def get_target_xy(self):
    """
    IK Solver - get current target (X,Y) coordinates
    """
    X = np.array([self.et[0]])
    Y = np.array([self.et[1]])
    return (X,Y)

  def get_target_xyz(self):
    """
    IK Solver - get current target (X,Y,Z) coordinates
    """
    X = np.array([self.et[0]])
    Y = np.array([self.et[1]])
    Z = np.array([self.et[2]])
    return (X,Y,Z)

  def get_predtgt_xy(self):
    """
    IK Solver - get predicted target (X,Y) coordinates
    """
    X = np.array([self.pt[0]])
    Y = np.array([self.pt[1]])
    return (X, Y)

  def get_predtgt_xyz(self):
    """
    IK Solver - get predicted target (X,Y,Z) coordinates
    """
    X = np.array([self.pt[0]])
    Y = np.array([self.pt[1]])
    Z = np.array([self.pt[2]])
    return (X, Y, Z)

  def get_endeff_xyz(self):
    """
    IK Solver - get current end effector (X,Y,Z) coordinates
    """
    X = np.array([self.ec[0]])
    Y = np.array([self.ec[1]])
    Z = np.array([self.ec[2]])
    return (X,Y,Z)
    
  def get_endeff_rot(self):
    """
    IK Solver - get current end effector rotation angle  
                in radians for rotation about the last 
                link segment's z-axis.
    """
    # Rotation matrix from world space to last joint local space 
    R = np.eye(3,dtype=np.float64)
    for i in range(self.nq) :
      Ri = np.transpose(rotation(self.q[i],self.u[i]))
      R  = np.dot(Ri,R)
    # World space X-axis in last joint local space
    vecX = vT(np.dot(R,vT([1.0,0.0,0.0])))
    # Angle between last link segment and X-axis
    phiZ = np.arctan2(vecX[1],vecX[0])
    return phiZ
  
  def get_endeff_ypr(self):
    """
    IK Solver - get current end effector yaw, pitch and roll 
                angles in radians for Euler rotations about 
                the last link segment's zyx axes respectively. 
                Assumes the pitch angle never reaches +/- 90 
                degrees.
    """
    # Rotation matrix from world space to last joint local space 
    R = np.eye(3,dtype=np.float64)
    for i in range(self.nq) :
      Ri = np.transpose(rotation(self.q[i],self.u[i]))
      R  = np.dot(Ri,R)
    # Euler rotation angles of last link segment
    yaw   =  np.arctan2(R[0,1],R[0,0])
    pitch = -np.arcsin(R[0,2])
    roll  =  np.arctan2(R[1,2],R[2,2])
    return (yaw,pitch,roll)
    
  def get_joints_rot(self):
    """
    IK Solver - get current joints rotation angle in radians
    """
    n = len(self.q)
    Q = np.zeros(n)
    for i in range(n) :
      Q[i] = self.q[i]
    return (Q)
    
  def mouse_click(self, b, x, y):
    """
    IK_Solver - store button and (x,y) states for mouse click
    """
    self.button = b
    if self.button == 2 :
      if self.Plot3D == 0 :
        self.et0[0] = x
        self.et0[1] = y
      else :
        theta = 2*pi*random.random()
        r     = random.uniform(2,4)
        self.et0[0] = r*cos(theta)
        self.et0[1] = r*sin(theta)
        self.et0[2] = random.uniform(1,3)
      self.et     = self.et0
      self.pt     = self.et0
      self.dq     = np.zeros(self.nq)
      self.dH     = np.zeros(self.nq)
      self.tsolve = 0.0
      self.tgo    = self.h
      self.ni     = 0
      self.set_points_x0y0z0()
      
  def onclick(self, event):
    """
    IK Solver - mouse button pressed handler
    """
    b = event.button
    x = event.xdata
    y = event.ydata
    self.mouse_click(b, x, y)
