File: README.txt
Auth: G. E. Deschaines
Date: 17 Apr 2015
Prog: Inverse Kinematics (IK) Solver
Desc: Instructions on using the IK_Solver programs.

The IK_Solver program can be executed in two different configurations, 
as a single executable program, or as client/server programs.

1. To invoke as a single program, execute the following command in 
   a terminal window.
   
     > python IK_Solver.py $1 $2 $3
     
   where the command line arguments $1, $2 and $3 can be as follows. 
     
     $1 - The IK solver method option [1,2,3,4,5,6,7] where:
            1 = Cyclic Coordinate Descent (CCD)
            2 = Jacobian Transpose Method (JTM)
            3 = Pseudo-inverse Jacobian method 2 (PIM2)
            4 = Pseudo-inverse Jacobian method 3 (PIM3)
            5 = Damped Least Squares (DLS)
            6 = PIM3 solver with null space control
            7 = DLS solver with null space control
     $2 - The reference frame and plotting mode (0=2D, 1=3D).
     $3 - The animation recording switch (0=Off, 1=On). Note that
          animation recording requires ffmpeg or avconv.
     
2. To invoke as a client/server programs, first start the server
   program in a terminal window using the following command.
   
     > python IK_Solver_server.py
     
   Next, start the client program in another terminal window using
   the command:
   
     > python IK_Server_client.py $1 $2 $3
    
   where the command line arguments are the same as those presented
   above for the single executable program.
   
The n-link chain is defined in the IK_Solver_nlink.py file, and the
target's position and velocity are hard-coded in the IK_Solver_class.py
file. For a 2D case, the pseudo-inverse null space may be used to coerce
the orientation of the end-effector link segment to point in the +X 
direction for the PIM3 and DLS methods. Similarly for the 3D case,
the null space may be used to coerce the end-effector link segment to 
point in the +X direction.
