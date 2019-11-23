The ./FourLink3D subdirectory contains a Blender file and associated Python scripts
to model and render the motion of a three dimensional, five revolute joint, 4-link
serial chain defined in the IK_Solver_nlink.py script which represents a notional
robotic manipulator arm. The ./FourLink3D/scripts/IK_SolverPanel.py file is utilized
to create a Blender control panel; providing means for Blender to connect as a client
with an executing IK_Solver_server.py program in order to control inverse kinematics
of the armature skeleton rig for the Blender robot model.

The robot model's initial pose is specified by the IK_Solver class methods and it's
motion is determined by applying Damped Least Squares algorithm with null space
control in responce to movement of the end-effector target modeled as a blue sphere
linked to an Empty object.