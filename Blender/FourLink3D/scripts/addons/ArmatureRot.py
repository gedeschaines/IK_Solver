# File: ArmatureRot.py
# Date: 16 Apr 2015
# Auth: Gary E. Deschaines
# Desc: Blender script addon to set armature bone rotation angles.

bl_info = {
    "name": "Rotate Bones",
    "category": "Object",
}

import bpy
import mathutils as mu
import math

class ArmatureRot(bpy.types.Operator):
    """Armature Rotate Script"""       # Blender will use this as a tooltip for
                                       #   menu items and buttons.
    bl_idname = "armature.rot_bones"   # unique identifier for buttons and menu
                                       #   items to reference.
    bl_label = "Rotate Bones"          # display name in the interface.
    bl_options = {'REGISTER', 'UNDO'}  # enable undo for the operator.

    # New property to store angle value for the rotate bones tool.
    angle = bpy.props.IntProperty(name="RotAng", 
                                  default=0, min=-180, max=180,
                                  subtype='ANGLE')

    # The execute() function is called by Blender when running the operator.
    def execute(self, context):  
        # Rotate armature bones by value of angle.
        armature = context.scene.objects['Armature1']
        for bone in armature.pose.bones:
            euler = bone.rotation_euler
            if bone.name == 'Bone1':
                euler.x = 0
                euler.y = math.radians(self.angle)
                euler.z = 0
            elif bone.name == 'Bone4':
                euler.x = math.radians(self.angle)
                euler.y = 0
                euler.z = math.radians(self.angle)
            else :
                euler.x = math.radians(self.angle)
                euler.y = 0
                euler.z = 0
            bone.rotation_euler = euler
      
        return {'FINISHED'}  # this lets Blender know the operator finished 
                             # successfully.

def menu_func(self, context):
    self.layout.operator(ArmatureRot.bl_idname)
    
# Store keymaps here to access after registration.
addon_keymaps = []
    
def register():
    bpy.utils.register_class(ArmatureRot)
    bpy.types.VIEW3D_MT_object.append(menu_func)
    
    # Handle the keymap.
    wm = bpy.context.window_manager
    km = wm.keyconfigs.addon.keymaps.new(name='Object Mode', space_type='EMPTY')
    
    kmi = km.keymap_items.new(ArmatureRot.bl_idname, 'SPACE', 'PRESS', 
                              ctrl=True, shift=True)
    kmi.properties.angle = 5
    addon_keymaps.append((km, kmi))
    
def unregister():
    bpy.utils.unregister_class(ArmatureRot)
    bpy.types.VIEW3D_MT_object.remove(menu_func)
     
    # Handle the keymap.
    for km, kmi in addon_keymaps:
        km.keymap_items.remove(kmi)
    addon_keymaps.clear()
    
    
# This allows you to run the script directly from Blender's text
# editor to test the addon without having to install it.
if __name__ == "__main__":
    register()
