import bpy


def skin(edge_mesh: bpy.types.Object, radius: float = 0.01):
    edge_mesh.modifiers.new(name="Skin", type="SKIN")
    edge_mesh.modifiers.new(name="Subdivision", type="SUBSURF")

    for vertex in edge_mesh.data.vertices:
        skin_vertex = edge_mesh.data.skin_vertices[""].data[vertex.index]
        skin_vertex.radius = (radius, radius)


def add_curve_mesh(points):
    """Add a string of edges between the given points."""
    edges = [(i, i + 1) for i in range(len(points) - 1)]
    mesh = bpy.data.meshes.new("Curve Mesh")
    mesh.from_pydata(points, edges, [])
    mesh.update()
    curve_mesh = bpy.data.objects.new("Curve Mesh", mesh)
    bpy.context.collection.objects.link(curve_mesh)
    return curve_mesh
