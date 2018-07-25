# File: IK_Solver_nlink.py
# Auth: G. E. Deschaines
# Date: 17 Apr 2015
# Prog: Inverse Kinematics (IK) Solver n-link chain definition.
# Desc: Functions utilized by the IK_Solver class, server and client.
#
# This file provides the n-link chain parameters definition function
# utilized by the IK_Solver_server and IK_Solver_client modules.
#
# Disclaimer:
#
# See DISCLAIMER

import sys

try:
  from IK_Solver_funcs import *
except ImportError:
  print("* Error: IK_Solver_funcs.py required.")
  sys.exit()
  
def get_nlink_chain(Plot3D) :
  """
  Jointed n-Link Chain Pictogram
                                                                 +    
       uz0          uz1          uz2              uzN-1        . (a)N 
        | uy0        | uy1        | uy2            | uyN-1  .     ^end 
        | /          | /          | /              | /   .        | effector
        |/           |/           |/               |/ .           |
   base>+------>ux0 -+------>ux1 -+------>ux2 ... -+------>uxN-1  |
       (a)0         (a)1         (a)2            (a)N-1           |     
        ^joint_1     ^joint_2     ^joint_3        ^joint_N        |
        | & link_1   | & link_2   | & link_3      | & link_N      |
        |            |            |               |               |
   <world space>     +----< preceding link body space [x,y,z] >---+
  """
  if Plot3D == 0 :
    # 2D 4-link chain
    a = {0:np.float64([0.0,0.0,0.0]), # <- joint 1 + set of link chain attachment
         1:np.float64([2.0,0.0,0.0]), # <- joint 2 | points in body coordinates,
         2:np.float64([2.0,0.0,0.0]), # <- joint 3 | except for the base given in
         3:np.float64([1.0,0.0,0.0]), # <- joint 4 | world space coordinates
         4:np.float64([0.0,0.0,0.0]), # <- joint 5 /
         5:np.float64([1.0,0.0,0.0])} # <- end effector
    u = {0:np.float64([0.0,0.0,1.0]), # <- joint 1 + set of joint rotation axis
         1:np.float64([0.0,0.0,1.0]), # <- joint 2 | unit vectors in link body
         2:np.float64([0.0,0.0,1.0]), # <- joint 3 | coordinates
         3:np.float64([1.0,0.0,0.0]), # <- joint 4 |
         4:np.float64([0.0,0.0,1.0])} # <- joint 5 /
    q0   = np.float64([  25*rpd,  # <- joint 1 + initial joint rotations (radians)
                         15*rpd,  # <- joint 2 |
                         10*rpd,  # <- joint 3 |
                          0*rpd,  # <- joint 4 |
                          5*rpd]) # <- joint 5 /
    qmin = np.float64([-360*rpd,  # <- joint 1 + joint minimum rotations (radians)
                       -135*rpd,  # <- joint 2 |
                        -60*rpd,  # <- joint 3 |
                       -360*rpd,  # <- joint 4 |
                       -160*rpd]) # <- joint 5 /
    qmax = np.float64([ 360*rpd,  # <- joint 1 + joint maximum rotations (radians)
                        135*rpd,  # <- joint 2 |
                         60*rpd,  # <- joint 3 |
                        360*rpd,  # <- joint 4 |
                        160*rpd]) # <- joint 5 /
    dqlim = np.float64([ 30*rpd,  # <- joint 1 + joint delta rotation limit (radians)
                         30*rpd,  # <- joint 2 |
                         30*rpd,  # <- joint 3 |
                         30*rpd,  # <- joint 4 |
                         30*rpd]) # <- joint 5 /
  else:
    # 3D 4-link chain
    a = {0:np.float64([0.0,0.0,0.0]), # <- joint 1 + set of link chain attachment
         1:np.float64([0.0,0.0,2.0]), # <- joint 2 | points in body coordinates,
         2:np.float64([2.0,0.0,0.0]), # <- joint 3 | except for the base given in
         3:np.float64([2.0,0.0,0.0]), # <- joint 4 | world space coordinates
         4:np.float64([0.0,0.0,0.0]), # <- joint 5 /
         5:np.float64([0.5,0.0,0.0])} # <- end effector
    u = {0:np.float64([0.0,0.0,1.0]), # <- joint 1 + set of joint rotation axis
         1:np.float64([0.0,1.0,0.0]), # <- joint 2 | unit vectors in link body
         2:np.float64([0.0,1.0,0.0]), # <- joint 3 | coordinates
         3:np.float64([0.0,1.0,0.0]), # <- joint 4 |
         4:np.float64([0.0,0.0,1.0])} # <- joint 5 /
    q0   = np.float64([   0*rpd,  # <- joint 1 + initial joint rotations (radians)
                        -45*rpd,  # <- joint 2 |
                         45*rpd,  # <- joint 3 |
                          0*rpd,  # <- joint 4 |
                         45*rpd]) # <- joint 5 /
    qmin = np.float64([-360*rpd,  # <- joint 1 + joint minimum rotations (radians)
                       -135*rpd,  # <- joint 2 |
                       -135*rpd,  # <- joint 3 |
                        -85*rpd,  # <- joint 4 |
                       -120*rpd]) # <- joint 5 /
    qmax = np.float64([ 360*rpd,  # <- joint 1 + joint maximum rotations (radians)
                        135*rpd,  # <- joint 2 |
                        135*rpd,  # <- joint 3 |
                         85*rpd,  # <- joint 4 |
                        120*rpd]) # <- joint 5 /
    dqlim = np.float64([ 30*rpd,  # <- joint 1 + joint delta rotation limit (radians)
                         30*rpd,  # <- joint 2 |
                         30*rpd,  # <- joint 3 |
                         30*rpd,  # <- joint 4 |
                         30*rpd]) # <- joint 5 /
    
  return (a, u, q0, qmin, qmax, dqlim)

