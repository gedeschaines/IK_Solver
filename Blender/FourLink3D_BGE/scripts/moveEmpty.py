# FILE: moveEmpty.py
# PROG: fourlink3d_BGE.blend
# DESC: Blender Game Engine logic block controller script for
#       keyboard control of Empty frame motion utilized as the
#       FourLink3D robotic manipulator arm end effector IK target.
# 
# NOTE: Select layer 1 (solids) and layer 2 (armature)
#       in 3D View before pressing 'P' key to initiate
#       game engine. Press 'space' key to position the
#       empty target at the TargetPath start point.
#
# Control Keys
# ------------
#
#   left arrow  - increase target velocity along the 
#                 TargetPath curve in -Y direction.
#   right arrow - increase target velocity along the
#                 TargetPath curve in +Y direction.
#   space bar   - position target at start of the
#                 TargetPath curve with zero velocity.

import bge
from mathutils import Vector

# Acquire current controller and its owner object.
cont = bge.logic.getCurrentController()
obj  = cont.owner

# Acquire controller's keyboard keypress sensors.
pressL = cont.sensors["left"]
pressR = cont.sensors["right"]
pressS = cont.sensors["space"]

# Acquire controller's actuator and its linear velocity.
move = cont.actuators["move"]
velX = move.linV[0]
velY = move.linV[1]
velZ = move.linV[2]

# Based on which keypress sensor exhibits a positive signal:
#
# 1) adjust (decr., incr. or zero) actuator linear velocity
# 2) force immediate update of owner object linear velocity
# 3) activate or deactivate the actuator

if pressL.positive:
    # decrement world space linear velocity
    velX = velX - 0.0
    velY = velY - 0.2
    velZ = velZ - 0.0
    move.linV = Vector((velX, velY, velZ))
    obj.setLinearVelocity(move.linV, False)
    cont.activate(move)

elif pressR.positive:
    # increment world space linear velocity
    velX = velX + 0.0
    velY = velY + 0.2
    velZ = velZ + 0.0
    move.linV = Vector((velX, velY, velZ))
    obj.setLinearVelocity(move.linV, False)
    cont.activate(move)
    
elif pressS.positive:
    # reset world space linear velocity and position
    velX = 0.0
    velY = 0.0
    velZ = 0.0
    move.linV = Vector((velX, velY, velZ))
    obj.setLinearVelocity(move.linV, False)
    obj.worldPosition = Vector((3.0, 4.0, 2.0))
    cont.deactivate(move)
    
