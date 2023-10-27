import os

import bpy
import numpy as np
from mathutils import Matrix


def add_charuco_board():
    # In the future we could generate charuco board with board.generateImage
    # Important: for the below operator to work, activate the "Import Images as Planes" addon in Blender preferences
    home = os.path.expanduser("~")
    image_path = os.path.join(home, "airo-mono/airo-camera-toolkit/test/data/default_charuco_board.png")
    print(image_path, os.path.exists(image_path))
    bpy.ops.import_image.to_plane(files=[{"name": image_path}], relative=False, height=0.22)
    board = bpy.context.object
    # plane.rotation_euler = (0.314, 0, 0)

    # enable the backface culling option for the material of the board
    board.data.materials[0].use_backface_culling = True

    num_board_rows = 5
    num_board_cols = 7

    board.data.transform(Matrix.Rotation(np.pi, 4, "X"))

    x_shift = num_board_cols / 2 * 0.04
    y_shift = num_board_rows / 2 * 0.04

    # board.data.transform(Matrix.Translation((x_shift, 0, 0)))
    board.data.transform(Matrix.Translation((x_shift, y_shift, 0)))
