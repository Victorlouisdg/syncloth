import bpy
import numpy as np
import triangle

random_seed = 0
np.random.seed(random_seed)

bpy.ops.object.delete()

width = np.random.uniform(0.2, 0.6)
length = np.random.uniform(width, 2 * width)

vertices_2D = [
    np.array([-width / 2, -length / 2]),
    np.array([-width / 2, length / 2]),
    np.array([width / 2, length / 2]),
    np.array([width / 2, -length / 2]),
]
edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
# faces = [(0, 1, 2, 3)]

triangle_input_mesh = {
    "vertices": vertices_2D,
    "segments": edges,
}
maximum_triangle_area = 0.0001  # 1 cm^2
triangle_options_string = f"qpa{maximum_triangle_area:.32f}"
print(triangle_options_string)
triangle_output_mesh = triangle.triangulate(triangle_input_mesh, triangle_options_string)

vertices_2D = triangle_output_mesh["vertices"]
triangles = triangle_output_mesh["triangles"]
vertices = np.column_stack([vertices_2D, np.zeros(vertices_2D.shape[0])])

name = "Towel"
mesh = bpy.data.meshes.new(name)
mesh.from_pydata(vertices, [], triangles)
mesh.update()
towel = bpy.data.objects.new(name, mesh)
bpy.context.collection.objects.link(towel)
