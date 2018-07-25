# File: IK_Solver_funcs.py
# Auth: G. E. Deschaines
# Date: 27 Apr 2015
# Prog: Inverse Kinematics (IK) Solver functions
# Desc: Functions utilized by the IK_Solver class.
#
# This file provides the following math methods and support functions 
# utilized by the IK_Solver class.
#
#   rotation  - compute coordinate rotation matrix using quaternion math
#   vT        - perform vector transpose of NumPy 1D arrays 
#   transform - transform coordinates from link body to world space
#   jacobian  - compute Jacobian matrix (see IK_Solve.py ref [1,2])
#   ik_jtm    - apply IK Jacobian transpose method (see IK_Solve.py ref [3])
#   ik_pim2   - apply IK pseudo-inverse method (see ref [2]) 
#   ik_pim3   - apply IK pseudo-inverse method (see ref [3])
#   ik_dls    - apply IK damped least squares technique (see ref [3])
#   clamp_rot - clamp joint rotations to specified limits
#   angle_chk - debug display of joint rotation angle checks
#
#   time_to_goal     - estimates time for end-effector to reach the 
#                      target goal
#   avelT_wrt_jointm - angular velocity of the target with respect
#                      to joint m
#   avelE_wrt_jointm - angular velocity of end-effector with respect
#                      to joint m
#
# Disclaimer:
#
# See DISCLAIMER

import sys

from math import pi

try:
  import numpy        as np
  import scipy.linalg as la
except ImportError:
  print("* Error: NumPy and SciPy packages required.")
  print("         Suggest installing the SciPy stack.")
  sys.exit()
  
rpd = pi/180  # radians per degree conversion constant
dpr = 180/pi  # degrees per radian conversion constant

p360rad =  360*rpd  # globals used in clamp_rot function
n360rad = -360*rpd  # 
p180rad =  180*rpd  #
n180rad = -180*rpd  #

# Compute coordinate transformation rotation matrix
def rotation(r,u):
  """
  function R = rotation(r,u) : returns local to global rotation matrix
    r = joint rotation (radians)
    u = joint rotation axis unit vector
  """
  theta = r/2
  costh = np.cos(theta)
  sinth = np.sin(theta)

  q0 = costh
  q1 = sinth*u[0]
  q2 = sinth*u[1]
  q3 = sinth*u[2]

  q00 = q0*q0
  q01 = q0*q1
  q02 = q0*q2
  q03 = q0*q3
  q11 = q1*q1
  q12 = q1*q2
  q13 = q1*q3
  q22 = q2*q2
  q23 = q2*q3
  q33 = q3*q3

  R = np.array([[q00+q11-q22-q33, 2*(q12-q03), 2*(q13+q02)],
                [2*(q12+q03), q00-q11+q22-q33, 2*(q23-q01)],
                [2*(q13-q02), 2*(q23+q01), q00-q11-q22+q33]])
  return R

# Vector Transpose for 1D array
def vT(v) :
  """
  function v^T = vT(v) : returns the transpose of a vector
    v = vector stored in a (1,n) or (n,1) array
  """
  s = np.shape(v)
  if np.size(s) == 1 :
    return np.reshape(v,(s[0],-1))
  else :
    return np.ravel(v)
    
# Transform points and rotations from link body to world space 
def transform(n,r,u,a):
  """
  function (p,w) = transform(n,r,u,a) : transforms a and u to p and w
    r = vector of n-1 joint rotations (radians)
    u = set of n-1 joint rotation axis unit vectors
    a = set of n link joint attachment location vectors to transform
  """
  #
  # Transform point (a) on link body to point (p) in world space
  p = a.copy()
  for i in range(n) :
    p[i] = a[i]
    j    = i - 1
    while j >= 0 :
      R    = rotation(r[j],u[j])
      p[i] = vT(np.dot(R,vT(p[i])))
      p[i] = p[i] + a[j]
      j    = j - 1
  #
  # Transform rotation axis (u) of link body to axis (w) in world space
  w = u.copy()
  for i in range(n-1) :
    j = i
    while j >= 0 :
      R    = rotation(r[j],u[j])
      w[i] = vT(np.dot(R,vT(w[i])))
      w[i] = w[i]/la.norm(w[i])
      j    = j - 1
  return (p,w)
  
# Compute Jacobian matrix J of de/dq to solve de=J*dq for dq.
def jacobian(n,r,p,e):
  """
  function J = jacobian(n,r,p,e) : returns de/dq Jacobian matrix for
                                   e representing a 1x3 vector in
                                   world space [x,y,z] coordinates
    r = set of n joint rotation direction vectors (i.e., a set
        {r1, r2, r3, ... rn} where r's are vectors of the form 
        [rx,ry,rz]).
    p = set of at least n-1 link joint position vectors (i.e., 
        {p1, p2, p3, ...} where p's are vectors of the form 
        [px,py,pz]).
    e = desired effector position vector (i.e., [ex,ey,ez]).
  """
  J = np.zeros((3,n),dtype=np.float64)
  for i in range(n) :
    j      = np.cross(r[i],e-p[i])  #  de/dq = r x (e - p)
    J[0,i] = j[0]  # | dex/dq1  dex/dq2  dex/dq3 ... dex/dqn |
    J[1,i] = j[1]  # | dey/dq1  dey/dq2  dey/dq3 ... dey dqn |
    J[2,i] = j[2]  # | dez/dq1  dez/dq2  dez/dq3 ... dez/dqn |
  return J

# IK Jacobian Transpose method to solve dq=inv(J)*de as presented 
# in reference [3]
def ik_jtm(J,de,dqlim):
  """
  function dq = ik_jtm(J,de,dqlim) : solves de = J*dq for dq using
                                     the Jacobian transpose method
    J     = Jacobian matrix of de/dq
    de    = vector of delta errors
    dqlim = vector of joint rotation delta limits
  """
  dq    = np.dot(np.transpose(J),vT(de))
  jdq   = np.dot(J,dq)
  alpha = np.dot(de,jdq)/pow(la.norm(np.transpose(jdq)),2)
  dqmax = np.amax(np.absolute(dq))
  beta  = dqlim[0]/dqmax
  dq    = np.amin([alpha,beta])*vT(dq)
  return dq

# IK Pseudo-Inverse Method to solve dq=inv(J)*de as presented in 
# reference [2]
def ik_pim2(J,de,sdel,dqlim):
  """
  function dq = ik_pim2(J,de,sdel,dqlim) : solves de = J*dq for dq using
                                           the pseudo-inverse method from
                                           reference [2]
    J     = Jacobian matrix of de/dq
    de    = vector of delta errors
    sdel  = singularity prevention damping constant
    dqlim = vector of joint rotation delta limits
  """
  n     = np.size(J,1)
  A     = np.dot(np.transpose(J),J) + sdel*np.eye(n)
  b     = np.dot(np.transpose(J),vT(de))
  dq    = vT(np.dot(la.inv(A),b))
  dqmax = np.amax(np.absolute(dq))
  if dqmax > dqlim[0] :
    dq = (dqlim[0]/dqmax)*dq
  return dq

# IK Pseudo-inverse Method to solve dq=inv(J)*de as presented in 
# reference [3]
def ik_pim3(J,de,sfac,dqlim,phi):
  """
  function dq = ik_pim3(J,de,sfac,dqlim,phi) : solves de = J*dq for dq using
                                               the pseudo-inverse method from
                                               reference [3].
    J     = Jacobian matrix of de/dq
    de    = vector of delta errors
    sfac  = singularity threshold factor
    dqlim = vector of joint rotation delta limits
    phi   = null space control vector
  """
  try:
    U,s,Vh = la.svd(J, full_matrices=True)  # NOTE: Vh here is V' in MATLAB
  except LinAlgError :
    print("SVD computation does not converge.")
    
  (m,n) = J.shape
  """
  %S = np.zeros((m,n),dtype=np.float64)
  for i in range(m) :
    S[i,i] = s[i]
  assert np.allclose(J, np.dot(U, np.dot(S, V)))
  """
  slim = sfac*np.amax(np.absolute(s))
  Sinv = np.zeros((n,m),dtype=np.float64)
  for i in range(m) :
    if abs(s[i]) > slim :
      Sinv[i,i] = 1.0/s[i]

  Jinv  = np.dot(np.transpose(Vh),np.dot(Sinv,np.transpose(U)))
  Jprj  = np.eye(n,dtype=np.float64) - np.dot(Jinv,J)
  dq    = np.dot(Jinv,vT(de)) + np.dot(Jprj,vT(phi))
  dqmax = np.amax(np.absolute(dq))
  for i in range(len(dq)) :
    if dqmax > dqlim[i] :
      dq[i] = dq[i]*(dqlim[i]/dqmax)
  return vT(dq)

# IK Damped Least Squares technique to solve dq=inv(J)*de as presented
# in reference [3]
def ik_dls(J,de,slam,dqlim,phi):
  """
  function dq = ik_dls(J,de,slam,dqlim,phi) : solves de = J*dq for dq using
                                              damped least squares with SVD
                                              from reference [3].
    J     = Jacobian matrix of de/dq
    de    = vector of delta errors
    slam  = singularity damping factor
    dqlim = vector of joint rotation delta limits
    phi   = null space control vector
  """
  try:
    U,s,Vh = la.svd(J,full_matrices=True)  # NOTE: Vh here is V' in MATLAB
  except LinAlgError :
    print("SVD computation does not converge.")
    
  (m,n) = J.shape
  """
  S = np.zeros((m,n),dtype=np.float64)
  for i in range(m) :
    S[i,i] = s[i]
  assert np.allclose(J, np.dot(U, np.dot(S, V)))  
  """
  Sinv = np.zeros((n,m),dtype=np.float64)
  for i in range(m) :
    Sinv[i,i] = s[i]/(pow(s[i],2) + pow(slam,2))
  
  Jinv  = np.dot(np.transpose(Vh),np.dot(Sinv,np.transpose(U)))
  Jprj  = np.eye(n,dtype=np.float64) - np.dot(Jinv,J)
  dq    = np.dot(Jinv,vT(de)) + np.dot(Jprj,vT(phi))
  dqmax = np.amax(np.absolute(dq))
  for i in range(len(dq)) :
    if dqmax > dqlim[i] :
      dq[i] = dq[i]*(dqlim[i]/dqmax)
  return vT(dq)

# Clamp given rotation angles to specified limits 
def clamp_rot(n,r,rmin,rmax):
  """
  function [r] = clamp_rot(n,r,rmin,rmax) : returns r within [rmin,rmax]
    r    = vector of n joint rotations (radians)
    rmin = vector of n joint rotation minimums (radians)
    rmax = vector of n joint rotation maximums (radians)
  """
  global p360rad, n360rad, p180rad, n180rad
  
# global dpr
# print "clamp_rot: r= %8.2f, %8.2f %8.2f" % (r[0]*dpr, r[1]*dpr, r[2]*dpr)
  
  for i in range(n) :
    if abs(rmax[i]-rmin[i]) >= p360rad :
      if r[i] < n180rad :
        r[i] = p360rad + r[i]
      if r[i] > p180rad :
        r[i] = r[i] + n360rad
    if r[i] < rmin[i] :
      r[i] = rmin[i]
    elif r[i] > rmax[i] :
      r[i] = rmax[i]    
  return r

def time_to_goal(pt,vt,w,p,dq):
  """
  function [avel] = time_to_goal(pt,vt,w,p,dq) : returns estimated time for
                                                 end-effector to reach the 
                                                 target goal
    pt = current position of target in world space
    vt = current velocity of target in world space
    w  = set of n joint rotation direction vectors in world space
    p  = set of n+1 position vectors (n joints + end-effector) in world space
    dq = vector of n joint rotation rates (radians/sec)
  """
  n  = len(dq)
  ie = len(p) - 1
  # absolute velocity of end effector
  ve = np.zeros(3)
  for i in range(n-1,-1,-1) :
    ve = ve + np.cross(dq[i]*w[i],(p[i+1]-p[i]))
  # closing vector between end-effector and target
  vecp = pt - p[ie]
  nrmp = la.norm(vecp)
  # estimated time to go
  if nrmp > 0.0 :
    vecn = vecp/nrmp
    tgo  = nrmp/(np.dot(ve-vt,vecn))
    if tgo < 0.0 : tgo = 0.0
  else :
    tgo = 0.0
  return tgo

def avelT_wrt_jointm(pt,vt,w,p,dq,m):
  """
  function [avel] = avelT_wrt_jointm(pt,vt,w,p,dq,m) : returns angular velocity of
                                                       the target with respect to
                                                       joint m, assuming joint 1's
                                                       position is fixed in world 
                                                       space.
    pt = current position of target in world space
    vt = current velocity of target in world space
    w  = set of n joint rotation direction vectors in world space
    p  = set of n+1 position vectors (n joints + end-effector) in world space
    dq = vector of n joint rotation rates (radians/sec)
    m  = joint number of interest [1,n]
  """
  try :
    assert m-1 in range(len(dq)), 'value of m-1 not in range(len(dq))' 
  except (AssertionError, args) :
    ename = args.__class__.__name__
    fname = 'avelT_wrt_jointm'
    print("%s in %s: %s" % (ename, fname, args) )
    sys.exit()
    
  n  = len(dq)
  ie = len(p) - 1
  # absolute velocity of joint m 
  vJ = np.zeros(3)
  for i in range(0,m-1,1) :
    vJ = vJ + np.cross(dq[i]*w[i],(p[i+1]-p[i]))
  # relative velocity of target wrt joint m
  vTJ = vt - vJ
  # direction vector to target from joint m
  vecp = pt - p[m-1]
  nrmp = la.norm(vecp)
  # angular velocity of the targer wrt to joint m
  avel = np.cross(vecp,vTJ)/(nrmp*nrmp)
  return avel

def avelE_wrt_jointm(w,p,dq,m):
  """
  function [avel] = avelE_wrt_jointm(w,p,dq,m) : returns angular velocity of
                                                 end-effector with respect to 
                                                 joint m, assuming joint 1's
                                                 position is fixed in world 
                                                 space.
    w  = set of n joint rotation direction vectors in world space
    p  = set of n+1 position vectors (n joints + end-effector) in world space
    dq = vector of n joint rotation rates (radians/sec)
    m  = joint number of interest [1,n]
  """
  try :
    assert m-1 in range(len(dq)), 'value of m-1 not in range(len(dq))' 
  except (AssertionError, args) :
    ename = args.__class__.__name__
    fname = 'avelE_wrt_jointm'
    print("%s in %s: %s" % (ename, fname, args) )
    sys.exit()
    
  n  = len(dq)
  ie = len(p) - 1
  # absolute velocity of end-effector
  vE = np.zeros(3)
  for i in range(n-1,-1,-1) :
    vE = vE + np.cross(dq[i]*w[i],(p[i+1]-p[i]))
  # absolute velocity of joint m 
  vJ = np.zeros(3)
  for i in range(0,m-1,1) :
    vJ = vJ + np.cross(dq[i]*w[i],(p[i+1]-p[i]))
  # relative velocity of end-effector wrt joint m
  vEJ = vE - vJ
  # direction vector to end-effector from joint m
  vecp = p[ie] - p[m-1]
  nrmp = la.norm(vecp)
  # angular velocity of end-effector wrt joint m
  avel = np.cross(vecp,vEJ)/(nrmp*nrmp)
  return avel

# Debug print for checking computed joint rotation angles. 
def angle_chk(q,u,p,et):
  """
  function angle_chk(q,u,p,et)
    q  = vector of n joint rotations (radians)
    u  = set of n joint local rotation axis unit vectors
    p  = set of n link joint position vectors
    et = target position vector
  """
  global  dpr

  print("* Angle Check:")
  n = len(q)
  k = 1
  a = q[k-1]*dpr
  print("joint %1d rotation angle from inertial x-axis  = %12.6f" % (k,a) )
  for i in range(1,n) :
    a = q[i]*dpr
    print("joint %1d rotation angle from joint %1d x-axis   = %12.6f" % (i+1,i,a) )
  asum = np.sum(q)*dpr
  print("angle to joint %1d x-axis from inertial x-axis = %12.6f" % (n,asum) )

  ept = et - p[n-1]   # vector from joint n to target
  npt = la.norm(ept)  # distance from joint n to target
  ux  = np.array([1.0,0.0,0.0])
  uxw = vT(np.dot(rotation(np.sum(q),u[n-1]),vT(ux)))
  at3 = np.arccos(np.dot(ept,uxw)/npt)*dpr
  atI = np.arccos(np.dot(ept,ux)/npt)*dpr
  print("angle to target from joint %1d x-axis          = %12.6f" % (n,at3) )
  print("angle to target from inertial x-axis         = %12.6f" % (atI) )

