# File: IK_SolverPanel.py
# Data: 26 Apr 2015
# Auth: Gary E. Deschaines
# Desc: Blender panel tool for connecting to an IK_Solver server.
#
# Disclaimer:
#
# See IK_Solver/DISCLAIMER

import bpy
import mathutils as mu
import math

from socket import *
from array  import *
from struct import *

### IK Server tcp connection parameters
  
HOST       = '127.0.0.1'
PORT       = 21000
ADDR       = (HOST, PORT)
BUFSIZ     = 512
tcpCliSock = 0
nFrame     = 0
lastTime   = -1.0
targetLoc  = mu.Vector((0.0,0.0,0.0))
endeffLoc  = mu.Vector((0.0,0.0,0.0))

### Time text display handler

scene = bpy.context.scene
ttobj = scene.objects['Text_Time']

def time_text(scene):
    fps = scene.render.fps / scene.render.fps_base  # actual framerate
    ttobj.data.body = 'Time: {0: 6.3f} secs'.format(scene.frame_current/fps)

bpy.app.handlers.frame_change_pre.append(time_text)

### Data pack/unpack functions

def pack_target_pos(buffer,x,y,z):
    ioff = 0
    pack_into('c',buffer,ioff,'P'.encode())
    ioff = ioff + calcsize('c')
    pack_into('fff',buffer,ioff,x,y,z)
    ioff = ioff + calcsize('fff')
    return ioff

def pack_target_vel(buffer,vx,vy,vz):
    ioff = 0
    pack_into('c',buffer,ioff,'V'.encode())
    ioff = ioff + calcsize('c')
    pack_into('fff',buffer,ioff,vx,vy,vz)
    ioff = ioff + calcsize('fff')
    return ioff

def unpack_target_loc(buffer,ioff):
    (n,) = unpack_from('I',buffer,ioff)
    ioff = ioff + calcsize('I')
    Loc = []
    for i in range(0,n) :
       (x, y, z) = unpack_from('fff',buffer,ioff)
       ioff = ioff + calcsize('fff')
       Loc.append(mu.Vector((x, y ,z)))
    return (ioff, Loc)
    
def unpack_endeff_loc(buffer,ioff):
    (n,) = unpack_from('I',buffer,ioff)
    ioff = ioff + calcsize('I')
    Loc = []
    for i in range(0,n) :
       (x, y, z) = unpack_from('fff',buffer,ioff)
       ioff = ioff + calcsize('fff')
       Loc.append(mu.Vector((x, y ,z)))
    return (ioff, Loc)
    
def unpack_joints_rot(buffer,ioff):  
    (n,) = unpack_from('I',buffer,ioff)
    ioff = ioff + calcsize('I')
    Rot = []
    for i in range(0,n) :
       (q,)= unpack_from('f',buffer,ioff)
       ioff = ioff + calcsize('f')
       Rot.append(q)
    return (ioff, Rot)
    
### Functions utilized by the IK_SolverUpdate operator

def sendTgtVel(n,context):
    global BUFSIZ, tcpCliSock
    # Update target velocity
    if n == 0 :
        tVel = mu.Vector((0.0,-0.2, 0.0))
    elif n == 251:
        tVel = mu.Vector((0.0, 0.0, 0.0))
    # Flush buffer.
    buffer = array('B',(0 for i in range(0,16)))
    # Pack target velocity and send.
    ioff = pack_target_vel(buffer,tVel[0],tVel[1],tVel[2])
    tcpCliSock.send(buffer)  
    # Wait for acknowledge.
    data = tcpCliSock.recv(16)
    
def sendTgtPos(n,context):
    global BUFSIZ, tcpCliSock
    # Get empty object.
    empty = bpy.data.objects['Empty']
    eLoc  = empty.location
    # Calculate and send target position.
    if n == 0 :
        tPos = eLoc
        # Flush buffer.
        buffer = array('B',(0 for i in range(0,16)))
        # Pack target position and send.
        ioff = pack_target_pos(buffer,tPos[0],tPos[1],tPos[2])
        tcpCliSock.send(buffer)  
        # Wait for acknowledge.
        data = tcpCliSock.recv(16) 
    eLoc[1] = 2.5 - 0.2*(n*0.08) 
    empty.location = eLoc
    empty.keyframe_insert(data_path="location",frame=n,index=1)

def updateIK(n, armature):
    global BUFSIZ, tcpCliSock, lastTime, targetLoc, endeffLoc
    
    # Send update message with frame # to server and wait to receive data.
    data = 'update' + ' ' + str(n)
    tcpCliSock.send(data.encode())
    buffer = tcpCliSock.recv(BUFSIZ)
    # Unpack data received.
    ioff = 0
    (time,)= unpack_from('f',buffer,ioff)
    ioff = ioff + calcsize('f')
    (ioff,tLoc) = unpack_target_loc(buffer,ioff)
    (ioff,eLoc) = unpack_endeff_loc(buffer,ioff)
    (ioff,jRot) = unpack_joints_rot(buffer,ioff)
    
    # Store target and end-effector locations.
    targetLoc[0] = tLoc[0][0]
    targetLoc[1] = tLoc[0][1]
    targetLoc[2] = tLoc[0][2]
    endeffLoc[0] = eLoc[0][0]
    endeffLoc[1] = eLoc[0][1]
    endeffLoc[2] = eLoc[0][2]
    '''
    print("time=%8.3f tLoc=[%8.4f %8.4f %8.4f] eLoc=[%8.4f %8.4f %8.4f]" % \
          (time, targetLoc.x, targetLoc.y, targetLoc.z, \
                 endeffLoc.x, endeffLoc.y, endeffLoc.z) )
    '''
    # Check for IK Solver iterations completed.
    if abs(time-lastTime) < 0.001 : return True
    lastTime = time
        
    # Update orientations of armature bones.
    k = 0
    for bone in armature.pose.bones:
        euler = bone.rotation_euler
        if bone.name == 'Bone1':
            euler.x = 0
            euler.y = jRot[k]
            euler.z = 0
            bone.rotation_euler = euler
            bone.keyframe_insert(data_path="rotation_euler",frame=n,index=1)
        elif bone.name == 'Bone4':
            k = k + 1
            euler.x = -jRot[k]
            euler.y = 0
            euler.z = 0
            bone.rotation_euler = euler
            bone.keyframe_insert(data_path="rotation_euler",frame=n,index=0)
            k = k + 1
            euler.x = 0
            euler.y = 0
            euler.z = jRot[k]
            bone.rotation_euler = euler
            bone.keyframe_insert(data_path="rotation_euler",frame=n,index=2)    
        else :
            k = k + 1
            euler.x = -jRot[k]
            euler.y = 0
            euler.z = 0
            bone.rotation_euler = euler
            bone.keyframe_insert(data_path="rotation_euler",frame=n,index=0)
    # Return indication IK Solver iterations not completed.
    return False
  
### IK_Solver tool operators

class IK_SolverUpdate(bpy.types.Operator):
    """Tooltip"""
    bl_idname = "armature.ik_solver_update"
    bl_label = "IK Solver Update Operator"
    
    @classmethod
    def poll(cls, context):
        return context.active_object is not None
       
    def execute(self, context):
        global tcpCliSock, nFrame
        if tcpCliSock :
            # Move armature.
            done = False
            while not done :
                nFrame = nFrame + 1
                if nFrame <= 251 : sendTgtPos(nFrame, context)
                if nFrame == 251 : sendTgtVel(nFrame, context)
                done = updateIK(nFrame, context.active_object)
        return {'FINISHED'}
        
class IK_SolverStop(bpy.types.Operator):
    """Tooltip"""
    bl_idname = "armature.ik_solver_stop"
    bl_label = "IK Solver Stop Operator"
    
    @classmethod
    def poll(cls, context):
        return context.active_object is not None
       
    def execute(self, context):
        global tcpCliSock, nFrame
        if tcpCliSock :
            data = 'stop'
            tcpCliSock.send(data.encode())
            tcpCliSock.close()
            del tcpCliSock
            tcpCliSock = 0
        return {'FINISHED'}
              
class IK_SolverStart(bpy.types.Operator):
    """Tooltip"""
    bl_idname = "armature.ik_solver_start"
    bl_label = "IK Solver Start Operator"
    
    @classmethod
    def poll(cls, context):
        return context.active_object is not None
      
    def execute(self, context):
        global tcpCliSock, nFrame, lastTime
        if tcpCliSock == 0 :
            tcpCliSock = socket(family=AF_INET, type=SOCK_STREAM)
            try :
                nFrame   = 0
                lastTime = -1.0
                Plot3D   = 2
                IKmethod = 7
                tcpCliSock.connect(ADDR)
                data = 'start' + ' ' + str(Plot3D) + ' ' + str(IKmethod)
                tcpCliSock.send(data.encode())
                raw_data = tcpCliSock.recv(BUFSIZ)
                data     = raw_data.decode()
                print(data)
                sendTgtPos(nFrame, context)
                sendTgtVel(nFrame, context)
                updateIK(nFrame, context.active_object)
            except error :
                print('* Could not connect to the IK Solver server.')
                print('  The IK Solver server needs to be running.')
                if tcpCliSock :
                    tcpCliSock.close()
                    del tcpCliSock
                    tcpCliSock = 0
        return {'FINISHED'}
        
class IK_SolverQuit(bpy.types.Operator):
    """Tooltip"""
    bl_idname = "armature.ik_solver_quit"
    bl_label = "IK Solver Quit Operator"
    
    @classmethod
    def poll(cls, context):
        return context.active_object is not None
       
    def execute(self, context):
        global tcpCliSock
        if tcpCliSock :
            data = 'quit'
            tcpCliSock.send(data.encode())
            tcpCliSock.close()
            del tcpCliSock
            tcpCliSock = 0
        return {'FINISHED'}

### IK_Solver menu panel

class ARMATURE_PT_IK_SolverPanel(bpy.types.Panel):
    bl_label = "IK Solver Server"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "data"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return (context.object is not None and \
                context.active_object.name in bpy.data.armatures.keys())

    def draw(self, context):
        layout = self.layout
        obj = context.object
        box = layout.box()
        box.label(text="IK Solver Server Tools")
        box.operator("armature.ik_solver_start")
        box.operator("armature.ik_solver_update")
        box.operator("armature.ik_solver_stop")
        box.operator("armature.ik_solver_quit")

### Menu panel and operator register/unregister functions

def register():
    bpy.utils.register_class(IK_SolverStart)
    bpy.utils.register_class(IK_SolverUpdate)
    bpy.utils.register_class(IK_SolverStop)
    bpy.utils.register_class(IK_SolverQuit)
    bpy.utils.register_class(ARMATURE_PT_IK_SolverPanel)

def unregister():
    bpy.utils.register_class(ARMATURE_PT_IK_SolverPanel)
    bpy.utils.unregister_class(IK_SolverQuit)
    bpy.utils.unregister_class(IK_SolverStop)
    bpy.utils.unregister_class(IK_SolverUpdate)
    bpy.utils.unregister_class(IK_SolverStart)

if __name__ == "__main__":
    register()
