import json
import os

import airo_blender as ab
import bpy
import cv2
import numpy as np

# Delete the default cube
bpy.ops.object.delete()

scene = bpy.context.scene

# add plane
bpy.ops.mesh.primitive_plane_add()

# Part 1: load the towel asset
file_directory = os.path.dirname(os.path.realpath(__file__))
asset_snapshot_path = os.path.join(file_directory, "asset_snapshot.json")

with open(asset_snapshot_path, "r") as file:
    assets = json.load(file)["assets"]

towel_mesh_assets = [asset for asset in assets if "towel_mesh" in asset["tags"]]

# Load a random towel mesh
random_towel_info = towel_mesh_assets[5]
towel = ab.load_asset(**random_towel_info)
scene.collection.objects.link(towel)

# Add modifiers
subdivision_modifier = towel.modifiers.new(name="Subdivision", type="SUBSURF")
subdivision_modifier.levels = 2
subdivision_modifier.render_levels = 2

# Add a solifify modifier to the towel to give it some thickness
solidify_modifier = towel.modifiers.new(name="Solidify", type="SOLIDIFY")
solidify_modifier.thickness = np.random.uniform(0, 0.006)  # max 6mm thickness
solidify_modifier.offset = 0

# Add a subsivision surface modifier to the towel
subdivision_modifier2 = towel.modifiers.new(name="Subdivision2", type="SUBSURF")
subdivision_modifier2.levels = 1
subdivision_modifier2.render_levels = 1

camera = bpy.data.objects["Camera"]
camera.location = (0.5, -0.5, 0.5)

# Set the camera focal length to 32 mm
camera.data.lens = 32

scene.render.engine = "CYCLES"
scene.cycles.samples = 64

image_width, image_height = 256, 256
scene.render.resolution_x = image_width
scene.render.resolution_y = image_height

bpy.context.view_layer.use_pass_z = True
bpy.context.view_layer.use_pass_normal = True

scene.use_nodes = True

# Add a file output node to the scene
tree = scene.node_tree
links = tree.links
nodes = tree.nodes

viewer_node = nodes.new("CompositorNodeViewer")
links.new(tree.nodes["Render Layers"].outputs["Normal"], viewer_node.inputs[0])

# render the scene
bpy.ops.render.render(write_still=True)

world_normals_image = bpy.data.images["Viewer Node"]
print(world_normals_image.size[0], world_normals_image.size[1])
w, h = world_normals_image.size
world_normals = np.array(world_normals_image.pixels)
world_normals = np.reshape(world_normals, (h, w, 4))[:, :, 0:3]
world_normals = np.flipud(world_normals)  # flip the image vertically

camera_matrix = bpy.data.objects["Camera"].matrix_world
camera_matrix = np.array(camera_matrix)
camera_orientation = camera_matrix[:3, :3]

camera_normals = np.zeros((h, w, 3))

camera_inverse = np.linalg.inv(camera_orientation)

# TODO figure out how to do this loop vectorized
for i in range(h):
    for j in range(w):
        world_normal = world_normals[i, j, :]
        camera_normal = camera_inverse @ world_normal
        camera_normals[i, j, :] = camera_normal


world_normals = 0.5 * world_normals + 0.5  # [-1, 1] to [0, 1]
camera_normals = 0.5 * camera_normals + 0.5  # [-1, 1] to [0, 1]

# use opencv to save the normal map
cv2.imwrite("normal_map.png", 255 * world_normals[:, :, ::-1])
cv2.imwrite("camera_normal_map.png", 255 * camera_normals[:, :, ::-1])
