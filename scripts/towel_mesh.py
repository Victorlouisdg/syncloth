import argparse
import json
import os
import sys

import airo_blender as ab
import bmesh
import bpy
import numpy as np
from scipy.spatial.transform import Rotation as R

random_seed = 0

if "--" in sys.argv:
    argv = sys.argv[sys.argv.index("--") + 1 :]  # get all args after "--"
    parser = argparse.ArgumentParser()
    parser.add_argument("seed", type=int)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_known_args(argv)[0]
    random_seed = args.seed

np.random.seed(random_seed)
scene = bpy.context.scene

bpy.ops.object.delete()
bpy.ops.mesh.primitive_plane_add()
# Add collision modifier to plane
bpy.ops.object.modifier_add(type="COLLISION")
ground = bpy.context.object
collistion_modifier = ground.modifiers["Collision"]

# Can't be too low or parts of the towel will fall through
ground_thickness = 0.01
ground.collision.thickness_outer = ground_thickness
ground.collision.cloth_friction = 80.0  # set maximum friction to make interesting configurations more stable

# Part 1: Create the towel geometry
width = np.random.uniform(0.2, 0.6)
length = np.random.uniform(width, 2 * width)

vertices = [
    np.array([-width / 2, -length / 2, 0.0]),
    np.array([-width / 2, length / 2, 0.0]),
    np.array([width / 2, length / 2, 0.0]),
    np.array([width / 2, -length / 2, 0.0]),
]
edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
faces = [(0, 1, 2, 3)]

name = "Towel"
mesh = bpy.data.meshes.new(name)
mesh.from_pydata(vertices, edges, faces)
mesh.update()
towel = bpy.data.objects.new(name, mesh)
bpy.context.collection.objects.link(towel)

# quad_target_width = 0.02 # 2 cm
quad_target_width = np.random.uniform(0.02, 0.05)

number_cuts = int(width / quad_target_width)

bpy.context.view_layer.objects.active = towel
bpy.ops.object.mode_set(mode="EDIT")
bpy.ops.mesh.subdivide(number_cuts=number_cuts)
bpy.ops.object.mode_set(mode="OBJECT")


# Increase vertex crease causes the subdivision to round them less
# See here why we need bmesh to a a VertexCreaseLayer:
# https://devtalk.blender.org/t/blender-3-1-vertex-crease-values/23607/7
bm = bmesh.new()
bm.from_mesh(towel.data)
bm.verts.layers.crease.new()
bm.edges.layers.crease.new()
bm.to_mesh(towel.data)
corner_vertices = [0, 1, 2, 3]
vertex_creases = creases = towel.data.vertex_creases[0].data
for i in corner_vertices:
    vertex_creases[i].value = np.random.uniform(0.0, 0.5)

bpy.ops.object.shade_smooth()

# UV unwrap the towel here
bpy.ops.object.select_all(action="DESELECT")
towel.select_set(True)
bpy.context.view_layer.objects.active = towel
bpy.ops.object.mode_set(mode="EDIT")
bpy.ops.mesh.select_all(action="SELECT")
bpy.ops.uv.unwrap()
bpy.ops.object.mode_set(mode="OBJECT")

# raise position the towel above the table
towel.location[2] = width
towel.rotation_euler = R.random().as_euler("xyz")

# Add a cloth modifier to the towel
cloth_modifier = towel.modifiers.new(name="Cloth", type="CLOTH")
cloth_modifier.collision_settings.distance_min = 0.0

# Enable self collision
self_collision_distance = quad_target_width / 4  # magic number
cloth_modifier.collision_settings.use_self_collision = True
cloth_modifier.collision_settings.self_distance_min = self_collision_distance

# Set scene frame end to 50
n_frames = 50
scene.frame_end = n_frames

# Run the simulation forward
for i in range(1, n_frames + 1):
    scene.frame_set(i)

# Apply the cloth modifier
bpy.ops.object.modifier_apply(modifier="Cloth")

# Select the towel
bpy.ops.object.select_all(action="DESELECT")
towel.select_set(True)
bpy.context.view_layer.objects.active = towel

# Remove most of the ground thickness to make the towel sit closer to the table
towel.location[2] -= 0.8 * ground_thickness
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

# Set the camera to look top down at the towel
camera = bpy.data.objects["Camera"]
camera.location = (0, 0, 2.0)
camera.rotation_euler = (0, 0, 0)

# set resolution to 256x256
scene.render.resolution_x = 256
scene.render.resolution_y = 256

scene.render.engine = "CYCLES"
scene.cycles.samples = 64

towel.asset_mark()
towel.asset_data.author = "airo"
towel.asset_data.description = "A rectangular towel, dropped from a small height."
tags = ["towel", "cloth", "mesh", "simulated", "airo"]
for tag in tags:
    towel.asset_data.tags.new(tag)

# Generating preview only work without -b flag
# override = bpy.context.copy()
# override["id"] = towel
# with bpy.context.temp_override(**override):
#     bpy.ops.ed.lib_id_generate_preview()

random_seed_padded = f"{random_seed:08d}"
blendfile_path = os.path.abspath(random_seed_padded + ".blend")

prefs = bpy.context.preferences
use_file_compression = prefs.filepaths.use_file_compression
file_preview_type = prefs.filepaths.file_preview_type

# Temporary change the preferences to save storage space
prefs.filepaths.use_file_compression = True
prefs.filepaths.file_preview_type = "NONE"  # "CAMERA" does not work with -b flag
bpy.ops.wm.save_as_mainfile(filepath=blendfile_path)

# Restore the preferences
prefs.filepaths.use_file_compression = use_file_compression
prefs.filepaths.file_preview_type = file_preview_type


if not args.debug:
    quit()

### Debug stuff below ###
# Set the output path
scene.render.filepath = random_seed_padded

# Render the scene
bpy.ops.render.render(write_still=True)

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

depsgraph = bpy.context.evaluated_depsgraph_get()
evaluated_towel = towel.evaluated_get(depsgraph)
mesh = evaluated_towel.data.copy()

# Visualization stuff
keypoint_ids = [0, 1, 2, 3]
keypoints_3D = [evaluated_towel.matrix_world @ v.co for v in evaluated_towel.data.vertices if v.index in keypoint_ids]

# Add a small sphere at each keypoint
for i, keypoint in enumerate(keypoints_3D):
    bpy.ops.mesh.primitive_uv_sphere_add(location=keypoint, scale=(0.01, 0.01, 0.01))
    sphere = bpy.context.object
    ab.add_material(sphere, (0.8, 0, 0))


ab.add_material(towel, [0.244384, 0.564874, 0.800000, 1.000000])

# load the river_rocks HDRI
file_directory = os.path.dirname(os.path.realpath(__file__))
asset_snapshot_path = os.path.join(file_directory, "asset_snapshot.json")

with open(asset_snapshot_path, "r") as file:
    assets = json.load(file)["assets"]

# Set an HDRI world background
world_info = [asset for asset in assets if asset["name"] == "river_rocks"][0]
world = ab.load_asset(**world_info)
scene.world = world
