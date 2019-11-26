File: README.txt
Auth: G. E. Deschaines
Date: 19 Apr 2015
Prog: Blender fourlink3d robot with IK_Solver python scripts.
Desc: Instructions on using Blender with fourlink3d.blend
      and IK_Solver python scripts.

For this Blender model, the motion of a 3D four link serial chain
(actually 5 links but only 4 visible) comprised of revolute joints 
defined in the scripts/IK_Solver_nlink.py file is determined by 
application of "Damped Least Squares" algorithm from the initial
manipulator arm orientation to that obtained when the end effector
is placed sufficiently close to the predicted intercept point of a
moving target represented by a blue sphere linked to an empty frame.

These instructions are for using Blender as the rendering interface
to the IK_Solver program. The IK_Solver program can be executed with 
a matplotlib graphics interface by following instructions in the
IK_Solver/README.txt file.

1) Start the IK_Solver Server in a terminal window as:

   > python ../../IK_Solver_server.py
   
   Should see "waiting for connection..." displayed in the 
   terminal window.

2) Start Blender with the fourlink3d.blend file in a
   terminal window as:

   > blender fourlink3d.blend

3) To create keyframes of robot motion in Blender.
   
   (A) The middle-left window pane must be a "3D View" type.
   (B) The 3D-View mode must be in "Object Mode".
   (C) The active (selected) object must be "Armature1". The
       quickest way to select "Armature1" is by using the 
       upper right window pane, which should be a "Outliner" 
       type. "Armature1" is a sub-object of "Scene" and by
       clicking the + button to the left of the "Scene" label
       the hierarchy of connected objects will be displayed. 
       If the "Armature1" label is not high-lighted in white,
       a left click on "Armature1" should select it, indicated
       by a white high-lighted label.
   (D) Press "Ctrl-Alt-U" keys to bring up the "User Preferences"
       window and check the "Auto Run Python Scripts" box on the 
       "File" tab page.
   (E) Change the lower-left window pane to "Text Editor" type.
   (F) The "IK_SolverPanel.py" script should be displayed in
       the active text box and "Register" should be checked.
   (G) Click on the "Run Script" button.
   (H) The "IK Solver Server Tools" menu pane should become
       visible at the bottom of the lower right window pane,
       which should be displaying data properties for 
       "Armature1" if steps (A), (B) and (C) were satisfied and
       the armature icon (stick-man) is high-lighted.
   (I) Click on the "IK Solver Start Operator" button to initiate 
       a connection to the IK_Solver Server and create frame 0.
       Should see the robot move to it's initial state in the 3D 
       View window pane, and the following messages displayed in 
       the terminal window in which the IK_Solver Server was 
       started:
         ...connected from: 127.0.0.1:port#
         Starting IK_Solver (Client3D=1,IKmethod=7)
       The last message line should also be displayed in the 
       terminal window in which Blender was started.
   (J) Click on "IK Solver Update Operator" button to solve inverse
       kinematics from initial manipulator arm orientation to end
       effector contact with moving empty frame.
   (K) Click on "IK Solver Stop Operator" button to disconnect from
       the IK_Solver Server, or click on the "IK Solver Server Quit"
       button to stop the IK_Solver Server before closing the socket
       connection to the server.

4) To view keyframes of robot motion in Blender.

   (A) The middle-left window pane must be a "3D View" type.
   (B) The 3D-View mode must be in "Object Mode".
   (C) The active (selected) object should be "Armature1" if
       Layer 1 is selected.
   (D) Change the lower-left window pane to "Timeline" type.
   (E) Use the play, stop, frame advance buttons to see the robot's
       movement in the 3D View window pane. You can change view point
       while in Object or Pose mode using the numeric keypad as such:
              5 - Toggle between orthographic/perspective mode
              1 - Look down the +Y global axis (forward view)
              3 - Look down the -X global axis (right side view)
              7 - Look down the -Z global axis (top view)
              + - Zoom in
              - - Zoom out
              0 - Use active camera
         Ctrl-0 - Use selected camera as active camera
       You can also use the mouse to pan, tilt, shift and zoom.
   (F) To create an animation of robot manipulator arm IK motion as
       viewed by the camera, switch the lower right window from scene
       "Armature1" skeleton pose (stick-man) mode to scene Render 
       (camera) mode and select the "Animation" button.
   (G) Motion paths can be added for the end effector and target by
       selecting each object in the 3D viewport's "Object Mode" and
       enter "Calculate" in the "Animation" tabbed panel. Alternately,
       each object may be selected in the upper right "Outliner" editor
       window and "Update Paths" selected under Motion Path" in the
       lower right "Properties" editor window if in object mode. Be sure
       to set the frame limits to match those presented in the bottom
       "Timeline" editor window.
   (H) To clear armature and empty animations, click on "Animation"
       under "Armature1" and "Empty" in the upper left "Outliner" editor
       window using the right mouse button and select "Clear Animation
       Data."

NOTES:

1) The global reference frame in Blender and IK_Solver are right-handed
   XYZ coordinate systems, the rest orientation of the robot arm is not
   equivalently specified. In Blender the robot armature rest position
   is defined in the YZ plane, while in IK_Solver it is defined in the 
   XZ as shown in the following pictograms.

             +Z Bone 2                       +Z Link 2  
              ^ |     Joint 3                 ^ |     Joint 3        
              | |-X  /Bone 3  End-Effector    | |+Y  /Link 3  End-Effector
              | |/  / |       |               | |/  / |       | 
      Joint 2-O====O======O===<       Joint 2-O====O======O===<
      Bone 1--I/           \ \        Link 1--I/           \ \
              O-------> +Y  \ Bone 4          O-------> +X  \ Links 4&5
             / \             Joint 4         / \             Joints 4&5
            /   Joint 1                     /   Joint 1
          +X                              -Y 
     
      Robot Arm in Blender            Robot Arm in IK_Solver
     
   Movement in the -X and +Y directions in Blender is movement in +Y
   and +X directions respectively in IK_Solver.

2) Joint rotations in Blender are defined in local xyz reference 
   frames for each bone as shown in the following pictogram.
   
          +z (2) +z (3)   +z (4)    The bone rotation axes:
           |/     |/       |/         (1) About +y axis
           O->+y==O->+y====O->+y=<    (2) About +x axis
          /I     /        /           (3) About +x axis
        +x I   +x       +x            (4) About +x and +z axes.
          +y
           ^ (1)
           |/
      +z<--O------->+Y
          / \
        +x   Armature Base
        /
      +X
      
   Positive rotation about the +x axis is in the opposite direction
   to rotation about the +y axis in IK_Solver.
   
3) Joint rotations in IK_Solver are defined in local xyz reference 
   frames for each link segment as shown in the following pictogram.
   
            +z +y  +z +y    +z +y      The joint rotation axes:
             |/     |/       |/          1) About +z axis
             O->+x==O->+x====O->+x=<     2) About +y axis
            /I     /        /            3) About +y axis
     Joint 2 I    Joint 3  Joints 4&5    4) About +y axis
            +z +y and +Y                 5) About +z axis
             |/
             O-->+x and +X
            /
     Joint 1
            
   Positive rotation about the +y axis is in the opposite direction
   to rotation about the +x axis in Blender.

4) The target's position and velocity are updated based on frame number
   and sent to the IK_Solver by calls to sendTgtPos() and sendTgtVel()
   functions respectively from the IK_SolverUpdate() function in the 
   IK_SolverPanel.py file.
