The ./FourLink3D and ./FourLink3D_BGE subdirectories contain Blender files and 
associated Python scripts to model and render the motion of a three dimensional,
five revolute joint, 4-link serial chain defined in the IK_Solver_nlink.py script
which represents a notional robotic manipulator arm.

The fourlink3d.blend, fourlink3d_BGE.blend files and associated Python scripts were
originally developed on a MS Windows 7 Pro operating system with Blender 2.68 and 
Python 3.3, and have been successfully used with Ubuntu operating system, Blender
and Python versions listed in the following table.

    Ubuntu  Blender  Python
    ------  -------  ------
     14.04    2.74    3.4.2
     16.04    2.74    3.4.2
     18.04    2.81    3.7.4

FourLink3D.blend - inverse kinematics solved using the IK_Solver program
------------------------------------------------------------------------

The robot model's initial pose is specified by the IK_Solver class methods, and its
motion is determined by applying Damped Least Squares algorithm with null space
control in response to movement of the end-effector target modeled as a blue sphere
linked to an Empty object which is constrained to move along a straight line path 
passing through the XYZ point [3.0, 2.5, 2.0] and parallel to the world space Y axis.

The ./FourLink3D/scripts/IK_SolverPanel.py script is utilized to create a Blender
control panel; providing means for Blender to connect as a client with an executing
IK_Solver_server.py program in order to control inverse kinematics of the armature
skeleton rig for the Blender robot model.

FourLink3D_BGE.blend - inverse kinematics solved with Blender's IK solvers
--------------------------------------------------------------------------

The robot model's initial pose is specified by the user selected Blender's built-in
IK solver method and specified parameters, and its motion is determined by applying 
the IK solver algorithm in response to movement of the end-effector target modeled as
a blue sphere linked to an Empty object which is constrained to move along a straight
line path passing through the XYZ point [3.0, 2.5, 2.0] and parallel to the world 
space Y axis.

In Blender Render mode, motion of the end effector target along its NURB curve path 
is controlled by animation keyframe buttons in a Timeline editor panel.

The ./FourLink3D/scripts/timeText.py script is run once while in Blender Render mode
to register a handler for animating frame time text in the 3D viewport.

In Blender Game mode with Blender Game Engine activated, motion of the end effector 
target is controlled by user keypresses as specified in the game logic editor panel
associated with the Empty object and as documented in the moveEmpty.py script. 

The ./FourLink3D/scripts/moveEmpty.py script is utilized in Blender Game mode to
control the motion of an Empty object assigned as the target for the armature
end effector as directed by user keypresses.

