import airo_blender as ab
import bpy
import numpy as np

from syncloth.curves.bezier import quadratic_bezier
from syncloth.visualization.curves import add_curve_mesh, skin
from syncloth.visualization.points import add_points_as_instances

bpy.ops.object.delete()  # Delete the default cube

point0 = np.array([0.0, 0.0, 0.0])
point1 = np.array([0.5, 1.0, 0.0])
point2 = np.array([1.0, 0.0, 0.0])

points = np.vstack((point0, point1, point2))
add_points_as_instances(points, radius=0.01, color=(0, 0, 1))
points_curve_mesh = add_curve_mesh(points)
skin(points_curve_mesh, radius=0.005)
ab.add_material(points_curve_mesh, (0.5, 0.5, 1))

num_samples = 100
t_range = np.linspace(0, 1, num_samples, endpoint=True)
curve = np.array([quadratic_bezier(t, point0, point1, point2) for t in t_range])

curve_curve_mesh = add_curve_mesh(curve)

# calculate the length of each edge
edges = [(i, i + 1) for i in range(len(curve) - 1)]

edge_lengths = []
for edge in edges:
    edge_length = np.linalg.norm(curve[edge[1]] - curve[edge[0]])
    edge_lengths.append(edge_length)

smallest_edge_length = np.min(edge_lengths)
add_points_as_instances(curve, radius=smallest_edge_length / 4, color=(1, 0, 0))

# add a build modifier
build_modifier = curve_curve_mesh.modifiers.new(name="Build", type="BUILD")
skin(curve_curve_mesh, radius=smallest_edge_length / 2)
ab.add_material(curve_curve_mesh, (1, 0.5, 0))

# set the camera position to look top down at the curve
camera = bpy.data.objects["Camera"]
camera.location = (0.5, 0.5, 1.5)
camera.rotation_euler = (0, 0, 0)

scene = bpy.context.scene
scene.render.engine = "CYCLES"
scene.cycles.samples = 64

image_width, image_height = 1024, 1024
scene.render.resolution_x = image_width
scene.render.resolution_y = image_height
scene.view_settings.view_transform = "Standard"  # White stays white


scene.frame_end = 120

# set the world background color to pure white
bpy.data.worlds["World"].node_tree.nodes["Background"].inputs["Color"].default_value = (1.0, 1.0, 1.0, 1.0)
