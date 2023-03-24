import airo_blender as ab
import bpy
import mathutils
import numpy as np
from mathutils import Matrix


def add_cylinder_with_origin_in_base(radius_height_ratio=0.05):
    bpy.ops.mesh.primitive_cylinder_add(depth=1, radius=radius_height_ratio)
    cylinder = bpy.context.object
    cylinder.data.transform(mathutils.Matrix.Translation((0, 0, 0.5)))
    return cylinder


def add_frame(pose: np.ndarray = np.identity(4), size: float = 0.1, name: str = "frame"):
    bpy.ops.object.empty_add(type="ARROWS")
    frame = bpy.context.object

    X = add_cylinder_with_origin_in_base()
    Y = add_cylinder_with_origin_in_base()
    Z = add_cylinder_with_origin_in_base()

    X.name = "X"
    Y.name = "Y"
    Z.name = "Z"

    X.rotation_euler = (0, np.pi / 2, 0)
    Y.rotation_euler = (-np.pi / 2, 0, 0)

    blender_red = [0.930111, 0.036889, 0.084376, 1.000000]
    blender_green = [0.205079, 0.527115, 0.006049, 1.000000]
    blender_blue = [0.028426, 0.226966, 0.760525, 1.000000]

    ab.add_material(X, blender_red)
    ab.add_material(Y, blender_green)
    ab.add_material(Z, blender_blue)

    X.parent = frame
    Y.parent = frame
    Z.parent = frame

    frame.matrix_world = Matrix(pose)
    bpy.context.view_layer.update()
    frame.scale = (size, size, size)

    # Apply the scale
    bpy.ops.object.select_all(action="DESELECT")
    frame.select_set(True)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

    frame.name = name

    return frame
