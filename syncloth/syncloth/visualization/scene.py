import bpy
import numpy as np


def add_table(
    width: float = 0.7, length: float = 1.4, height: float = 0.7, rotation_z: float = 0.0
) -> bpy.types.Object:
    """Creates a rectangular table surface.

    Args:
        width: Extend along x-axis.
        length: Extend along y-axis.
        height: Height of the table top.
        rotation_z: Rotation in radians around the z_axis.

    Returns:
        bpy.types.Object: The table Blender object.
    """
    bpy.ops.mesh.primitive_plane_add()
    table = bpy.context.object
    table.scale = (width / 2.0, length / 2.0, 1)
    table.rotation_euler = (0, 0, rotation_z)
    table.location = (0, 0, height)
    return table


def add_towel(width: float = 0.5, length: float = 1.2, height: float = 0.7, rotation_z: float = 0.0):
    """Creates a rectangular towel with known vertex order."""

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

    towel.rotation_euler = (0, 0, rotation_z)
    towel.location = (0, 0, height)
    return towel
