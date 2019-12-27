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
     18.04    2.79    3.7.0  (Blender Game Engine provided by UPBGE v0.2.4b)
     18.04    2.81    3.7.4  (Blender Game Engine no longer in Blender v2.8x)

FourLink3D.blend - inverse kinematics solved using the IK_Solver program
------------------------------------------------------------------------

The robot model's initial pose is specified by the IK_Solver class methods, and its
motion is determined by applying Damped Least Squares algorithm with null space
control in response to movement of the end-effector target modeled as a blue sphere
linked to an Empty frame object which is constrained to move along a straight line
path starting at the XYZ point [3.0, 2.5, 2.0] and parallel to the world space Y axis.

The ./FourLink3D/scripts/IK_SolverPanel.py script is utilized to create a Blender
control panel; providing means for Blender to connect as a client with an executing
IK_Solver_server.py program in order to control inverse kinematics of the armature
skeleton rig for the Blender robot model as described in the ./FourLink3D/README.txt
file.

FourLink3D_BGE.blend - inverse kinematics solved with Blender's IK solvers
--------------------------------------------------------------------------

The robot model's initial pose is specified by the user selected Blender's built-in
IK solver method and specified parameters, and its motion is determined by applying 
the IK solver algorithm in response to movement of the end-effector target modeled as
a blue sphere linked to an Empty frame object which is constrained to move along a
straight line path parallel to the world space Y axis; starting at the XYZ point
[3.0, 4.0, 2.0] and ending at the point [3.0, -4.0, 2.0].

In Blender Render mode, motion of the end effector target along its NURB curve path 
is controlled by animation keyframe buttons in a Timeline editor panel. The frame
count from 0 to 500 as the target moves from its initial to final position along
the target path represents 40 seconds at 12.5 FPS; which corresponds to a target
vecocity of 0.2 units/seconds (8 units traversed in 40 seconds).

The ./FourLink3D_BGE/scripts/timeText.py script is run once while in Blender Render
mode to register a handler for animating frame time text in the 3D viewport.

In Blender Game mode with Blender Game Engine activated, motion of the end effector 
target is controlled by user keypresses as specified in the game logic editor panel
associated with the Empty object and as documented in the moveEmpty.py script. 

The ./FourLink3D_BGE/scripts/moveEmpty.py script is utilized in Blender Game mode
to control the motion of an Empty object assigned as the target for the armature
end effector as directed by user keypresses. Each left or right arrow keypress
adjusts the target velocity by -/+ 0.2 units/second respectively. Pressing the
spacebar resets the target's initial XYZ location to [3.0, 4.0, 2.0] with zero
linear velocity.
