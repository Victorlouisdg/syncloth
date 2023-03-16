import bpy

bpy.ops.object.delete()  # Delete the default cube

camera = bpy.data.objects["Camera"]

# Set camera extrinsics
camera.location = (0, 0, 1)
camera.rotation_euler = (0, 0, 0)  # Look down the z-axis
# You can also set the camera.matrix_world
# pose = np.identity(4)
# from mathutils import Matrix
# camera.matrix_world = Matrix(pose)

# Set camera intrinsics
camera.data.lens = 2.12  # focal length in mm
camera.data.sensor_width = 5.376  # mm
camera.data.sensor_height = 3.04  # mm
# TODO: check the meaning of sensor_fit
# camera.data.sensor_fit = 'HORIZONTAL'

scene = bpy.context.scene
scene.render.resolution_x = 1280
scene.render.resolution_y = 720

# Important: for the below operator to work, activate the "Import Images as Planes" addon in Blender preferences
image_name = "airo.jpg"
bpy.ops.import_image.to_plane(files=[{"name": image_name}], relative=True, height=0.3)

# Set the plane's pose
plane = bpy.context.object
plane.rotation_euler = (0.314, 0, 0)
