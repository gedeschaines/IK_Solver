import bpy

# Run this script once to enable animation of frame time
# text in the 3D viewport while in Blender Render mode.
#
# Derived from code presented at following web page:
# 
# https://blender.stackexchange.com/questions/7904/how-can-i-make-dynamic-text-in-an-animation

### Time text display handler

scene = bpy.context.scene
ttobj = scene.objects['Text_Time']

def time_text(scene):
    fps = scene.render.fps / scene.render.fps_base  # actual framerate
    ttobj.data.body = 'Time: {0: 6.3f} secs'.format(scene.frame_current/fps)

bpy.app.handlers.frame_change_pre.append(time_text)