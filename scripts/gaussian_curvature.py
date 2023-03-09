import json
import os

import airo_blender as ab
import bpy
import igl
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
subdivision_modifier2.levels = 3
subdivision_modifier2.render_levels = 3

# Add triangulation modifier
triangulation_modifier = towel.modifiers.new(name="Triangulate", type="TRIANGULATE")

# set active object
bpy.context.view_layer.objects.active = towel

# Apply the three modifiers
bpy.ops.object.modifier_apply(modifier="Subdivision")
bpy.ops.object.modifier_apply(modifier="Solidify")
bpy.ops.object.modifier_apply(modifier="Subdivision2")
bpy.ops.object.modifier_apply(modifier="Triangulate")

# update
bpy.context.view_layer.update()

# get vertices as numpy array
n_vertices = len(towel.data.vertices)
V = np.zeros((n_vertices, 3), dtype=np.float32)
towel.data.vertices.foreach_get("co", V.ravel())

# get triangles as numpy array
n_triangles = len(towel.data.polygons)

print(n_triangles)

F = np.zeros((n_triangles, 3), dtype=np.int32)
towel.data.polygons.foreach_get("vertices", F.ravel())

k = igl.gaussian_curvature(V, F)
k = np.abs(k)
k = k / np.max(k)  # normalize
k = 1.0 - k  # invert
k = k**300  # squashes values down towards black to make edges more visible


# print minimum and maximum curvature
print(np.min(k))
print(np.max(k))

# Part 2: add custom attributes to the towel
# Add a custom attribute to the towel
attribute = towel.data.attributes.new(name="myAttribute", type="FLOAT", domain="POINT")
# Set the value of the custom attribute
n_vertices = len(towel.data.vertices)

# test = np.zeros(n_vertices, dtype=np.float32)
# test = np.array([v.co.x for v in towel.data.vertices])

attribute.data.foreach_set("value", k)

# towel material use nodes
material = bpy.data.materials.new(name="Curvature")
material.use_nodes = True

node_tree = material.node_tree
nodes = node_tree.nodes
links = node_tree.links

# add atrribute node
attribute_node = nodes.new(type="ShaderNodeAttribute")
attribute_node.attribute_name = "myAttribute"

# link attribute node to material output
links.new(attribute_node.outputs[0], nodes["Material Output"].inputs["Surface"])

# add material to towel
towel.data.materials.append(material)

camera = bpy.data.objects["Camera"]
camera.location = (0.5, -0.5, 0.5)

# Set the camera focal length to 32 mm
camera.data.lens = 32

scene.render.engine = "CYCLES"
scene.cycles.samples = 64

image_width, image_height = 256, 256
scene.render.resolution_x = image_width
scene.render.resolution_y = image_height


# pale_blue = (0.5, 0.5, 1.0, 1.0)
# pale_yellow = (1.0, 1.0, 0.5, 1.0)
# green = (0.0, 1.0, 0.0, 1.0)
# material = create_gridded_dish_towel_material(5, 5, 0.2, 0.2, pale_blue, pale_yellow, green)
# towel.data.materials.append(material)
