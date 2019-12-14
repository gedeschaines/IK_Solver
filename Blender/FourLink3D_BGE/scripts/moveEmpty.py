import bge

# Use with Blender Game Engine
# ============================
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
           
cont = bge.logic.getCurrentController()
obj  = cont.owner

pressL = cont.sensors["left"]
pressR = cont.sensors["right"]
pressS = cont.sensors["space"]

move = cont.actuators["move"]
velX = move.dLoc[0]
velY = move.dLoc[1]
velZ = move.dLoc[2]
    
if pressL.positive:
    cont.activate(move)
    velX = velX - 0.00
    velY = velY - 0.05
    velZ = velZ - 0.00
    move.dLoc = [velX, velY, velZ]

elif pressR.positive:
    cont.activate(move)
    velX = velX + 0.00
    velY = velY + 0.05
    velZ = velZ + 0.00
    move.dLoc = [velX, velY, velZ]
    cont.activate(move)
    
elif pressS.positive:
    velX = 0.0
    velY = 0.0
    velZ = 0.0
    move.dLoc = [velX, velY, velZ]
    obj.worldPosition = [3.0, 4.0, 2.0]
    cont.deactivate(move)
    