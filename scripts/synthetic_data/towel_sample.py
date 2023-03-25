import argparse
import json
import os
import sys

import airo_blender as ab
import bmesh
import bpy
import cv2
import numpy as np
from airo_blender.coco_parser import CocoImage, CocoKeypointAnnotation
from bpy_extras.object_utils import world_to_camera_view
from pycocotools import mask

random_seed = 0

if "--" in sys.argv:
    argv = sys.argv[sys.argv.index("--") + 1 :]  # get all args after "--"
    parser = argparse.ArgumentParser()
    parser.add_argument("seed", type=int)
    args = parser.parse_known_args(argv)[0]
    random_seed = args.seed

# Set numpy random seed for reproducibility
np.random.seed(random_seed)

# Delete the default cube
bpy.ops.object.delete()

scene = bpy.context.scene

# Part 1: load the towel asset
file_directory = os.path.dirname(os.path.realpath(__file__))
asset_snapshot_path = os.path.join(file_directory, "asset_snapshot.json")

with open(asset_snapshot_path, "r") as file:
    assets = json.load(file)["assets"]

towel_mesh_assets = [asset for asset in assets if "towel_mesh" in asset["tags"]]

# Load a random towel mesh
random_towel_info = np.random.choice(towel_mesh_assets)
towel = ab.load_asset(**random_towel_info)
scene.collection.objects.link(towel)

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

# Add a random color to the towel
random_rgb_color = np.random.uniform(0.0, 1.0, size=3)
ab.add_material(towel, random_rgb_color)


# Part 2: Load a background and a table
# Set an HDRI world background
worlds = [asset for asset in assets if asset["type"] == "worlds"]
random_world_info = np.random.choice(worlds)
world = ab.load_asset(**random_world_info)
scene.world = world


# Load a random table
def table_filter(asset_info: dict) -> bool:
    if asset_info["type"] != "collections":
        return False

    if "table" not in asset_info["tags"]:
        return False

    not_tables = ["desk_lamp_arm_01", "CoffeeCart_01", "wicker_basket_01"]
    if asset_info["name"] in not_tables:
        return False

    return True


tables = [asset for asset in assets if table_filter(asset)]
random_table = np.random.choice(tables)
table_collection = ab.load_asset(**random_table)
bpy.ops.object.collection_instance_add(collection=table_collection.name)
table = bpy.context.object

# Part 3: Place the towel on the table
# Bounding box of the table
_, max_corner = ab.axis_aligned_bounding_box(table_collection.objects)
_, _, z_max = max_corner

# Place the towel on the table with a random rotation
towel.location = (0.0, 0.0, z_max + 0.003)
towel.rotation_euler = (0.0, 0.0, np.random.uniform(0.0, 2 * np.pi))

# Part 4: Setting up the camera
camera = bpy.data.objects["Camera"]


def random_point_on_unit_sphere() -> np.ndarray:
    point_gaussian_3D = np.random.randn(3)
    point_on_unit_sphere = point_gaussian_3D / np.linalg.norm(point_gaussian_3D)
    return point_on_unit_sphere


# Sample a point on the top part of the unit sphere
high_point = random_point_on_unit_sphere()
while high_point[2] < 0.75:
    high_point = random_point_on_unit_sphere()

# Place the camera above the table
high_point[2] += z_max
camera.location = high_point

# Make the camera look at the towel center
camera_direction = towel.location - camera.location  # Note: these are mathutils Vectors
camera.rotation_euler = camera_direction.to_track_quat("-Z", "Y").to_euler()

# Set the camera focal length to 32 mm
camera.data.lens = 32

# Part 5: Render the image

# Telling Blender to render with Cycles, and how many rays we want to cast per pixel
scene.render.engine = "CYCLES"
scene.cycles.samples = 64

image_width, image_height = 256, 256
scene.render.resolution_x = image_width
scene.render.resolution_y = image_height

scene.view_settings.exposure = np.random.uniform(-2, 2)
scene.view_settings.gamma = np.random.uniform(0.9, 1.1)

# Make a directory to organize all the outputs
random_seed_padded = f"{random_seed:08d}"
output_dir = random_seed_padded
os.makedirs(output_dir, exist_ok=True)

# Set image format to PNG
image_name = random_seed_padded

# Semantic segmentation of the towel
towel.pass_index = 1

scene.view_layers["ViewLayer"].use_pass_object_index = True
scene.use_nodes = True

# Add a file output node to the scene
tree = scene.node_tree
links = tree.links
nodes = tree.nodes
node = nodes.new("CompositorNodeOutputFile")
node.location = (500, 200)
node.base_path = output_dir
slot_image = node.file_slots["Image"]
slot_image.path = image_name
slot_image.format.color_mode = "RGB"

# Prevent the 0001 suffix from being added to the file name


segmentation_name = f"{image_name}_segmentation"
node.file_slots.new(segmentation_name)
slot_segmentation = node.file_slots[segmentation_name]

# slot_segmentation.path = f"{random_seed:08d}_segmentation"
slot_segmentation.format.color_mode = "BW"
slot_segmentation.use_node_format = False
slot_segmentation.save_as_render = False

render_layers_node = nodes["Render Layers"]
links.new(render_layers_node.outputs["Image"], node.inputs[0])

# Other method, use the mask ID node
mask_id_node = nodes.new("CompositorNodeIDMask")
mask_id_node.index = 1
mask_id_node.location = (300, 200)
links.new(render_layers_node.outputs["IndexOB"], mask_id_node.inputs[0])
links.new(mask_id_node.outputs[0], node.inputs[slot_segmentation.path])

# Rendering the scene into an image
bpy.ops.render.render(animation=False)

# Annoying fix, because Blender adds a 0001 suffix to the file name which can't be disabled
image_path = os.path.join(output_dir, f"{image_name}0001.png")
image_path_new = os.path.join(output_dir, f"{image_name}.png")
os.rename(image_path, image_path_new)

segmentation_path = os.path.join(output_dir, f"{segmentation_name}0001.png")
segmentation_path_new = os.path.join(output_dir, f"{segmentation_name}.png")
os.rename(segmentation_path, segmentation_path_new)

# TODO get bounding box of the segmentation mask
# TODO load segmentation mask
# np.where(segmentation_mask == True) # get the coordinates

segmentation_mask = cv2.imread(segmentation_path_new, cv2.IMREAD_GRAYSCALE)
mask_coords = np.where(segmentation_mask == 255)
area = mask_coords[0].shape[0]  # number of pixels
x_min = np.min(mask_coords[1])
y_min = np.min(mask_coords[0])
x_max = np.max(mask_coords[1])
y_max = np.max(mask_coords[0])
bbox_width = x_max - x_min
bbox_height = y_max - y_min
coco_bbox = (x_min, y_min, bbox_width, bbox_height)

rle = mask.encode(np.asfortranarray(segmentation_mask))
coco_segmentation = {"counts": rle["counts"].decode("utf-8"), "size": rle["size"]}

# drawing the rectangle
image_bgr = cv2.imread(image_path_new)
cv2.rectangle(image_bgr, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
image_annotated_path = os.path.join(output_dir, f"{image_name}_annotated.png")
# Draw a circle in the top left corner of the bounding box
cv2.circle(image_bgr, (x_min, y_min), 5, (0, 0, 255), -1)
cv2.imwrite(image_annotated_path, image_bgr)


coco_image = CocoImage(file_name=image_path_new, height=image_height, width=image_width, id=random_seed)
print(coco_image)

# We need to use the evaluated version of the towel, which takes into account the modifiers
depsgraph = bpy.context.evaluated_depsgraph_get()
evaluated_towel = towel.evaluated_get(depsgraph)
mesh = evaluated_towel.data.copy()

keypoint_ids = [0, 1, 2, 3]
keypoints_3D = [evaluated_towel.matrix_world @ v.co for v in evaluated_towel.data.vertices if v.index in keypoint_ids]
keypoints_2D = [world_to_camera_view(scene, camera, corner) for corner in keypoints_3D]

coco_keypoints = []
num_labeled_keypoints = 0
for keypoint_2D in keypoints_2D:
    u, v, _ = keypoint_2D
    px = image_width * u
    py = image_height * (1.0 - v)
    visible_flag = 2

    # Currently we set keypoints outside the image to be "not labeled"
    if px < 0 or py < 0 or px > image_width or py > image_height:
        visible_flag = 0
        px = 0.0
        py = 0.0

    if v > 0:
        num_labeled_keypoints += 1

    coco_keypoints += (px, py, visible_flag)

towel_category_id = 0  # TODO import this once airo-datasets exists

annotation = CocoKeypointAnnotation(
    category_id=towel_category_id,
    id=np.random.randint(np.iinfo(np.int32).max),
    image_id=random_seed,
    keypoints=coco_keypoints,
    num_keypoints=num_labeled_keypoints,
    segmentation=coco_segmentation,
    area=area,
    bbox=coco_bbox,
    iscrowd=0,
)

# Save CocoImage to disk as json
coco_image_json = f"{image_name}_coco_image.json"
coco_image_json_path = os.path.join(output_dir, coco_image_json)
with open(coco_image_json_path, "w") as file:
    json.dump(coco_image.dict(exclude_none=True), file, indent=4)


# Save CocoKeypointAnnotation to disk as json
coco_annotation_json = f"{image_name}_coco_annotation_{annotation.id}.json"
coco_annotation_json_path = os.path.join(output_dir, coco_annotation_json)
with open(coco_annotation_json_path, "w") as file:
    json.dump(annotation.dict(exclude_none=True), file, indent=4)
