import airo_blender as ab
import bpy
import numpy as np


def add_points_as_instances(points: np.ndarray, radius: float = 0.01, color: tuple = None):
    """Add a point cloud to the scene as instances of a sphere."""

    # Save the active collection as we will temporarily change it to add the instances etc.
    view_layer = bpy.context.view_layer
    collection_originally_active = view_layer.active_layer_collection

    template_collection = bpy.data.collections.new("Point template")
    bpy.context.scene.collection.children.link(template_collection)
    view_layer.active_layer_collection = view_layer.layer_collection.children[template_collection.name]
    bpy.context.view_layer.update()
    bpy.ops.mesh.primitive_ico_sphere_add(scale=(radius, radius, radius))
    template = bpy.context.object

    # Make new collection to group the instances
    instances_collection = bpy.data.collections.new("Point instances")
    bpy.context.scene.collection.children.link(instances_collection)
    view_layer.active_layer_collection = view_layer.layer_collection.children[instances_collection.name]

    # Add instances of the sphere
    for point in points:
        bpy.ops.object.collection_instance_add(collection=template_collection.name, location=point)
        instance_empty = bpy.context.object
        instance_empty.empty_display_size = radius * 1.5

    # Add a material to the template if a color was specified
    if color is not None:
        ab.add_material(template, color)

    # Hide the template itself
    view_layer.layer_collection.children.get(template_collection.name).exclude = True

    # Restore the active collection
    view_layer.active_layer_collection = collection_originally_active

    return template


def add_curve_mesh(points):
    """Add a string of edges between the given points."""
    edges = [(i, i + 1) for i in range(len(points) - 1)]
    mesh = bpy.data.meshes.new("Edge string")
    mesh.from_pydata(points, edges, [])
    mesh.update()
    curve_mesh = bpy.data.objects.new("Edge string", mesh)
    bpy.context.collection.objects.link(curve_mesh)
    return curve_mesh


def skin(edge_mesh: bpy.types.Object, radius: float = 0.01):
    edge_mesh.modifiers.new(name="Skin", type="SKIN")
    edge_mesh.modifiers.new(name="Subdivision", type="SUBSURF")

    for vertex in edge_mesh.data.vertices:
        skin_vertex = edge_mesh.data.skin_vertices[""].data[vertex.index]
        skin_vertex.radius = (radius, radius)


def quadratic_bezier(t, p0, p1, p2):
    return (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t**2 * p2


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
