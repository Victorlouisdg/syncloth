import json
import os

import airo_blender as ab
import bpy
import numpy as np
from airo_dataset_tools.camera_intrinsics import CameraIntrinsics, FocalLengths, PrincipalPoint, Resolution
from mathutils import Matrix

from syncloth.robot_arms import add_ur_with_robotiq

bpy.ops.object.delete()  # Delete the default cube

camera = bpy.data.objects["Camera"]

# Set camera extrinsics
camera.location = (0, 0, 1.3)
camera.rotation_euler = (0, 0, 0)  # Look down the z-axis

# Set camera intrinsics
# All don't work
# Zed2i.RESOLUTION_2K.width,
# Zed2i.RESOLUTION_2K.getWidth()
# Zed2i.RESOLUTION_2K._w
image_resolution_x = 2208
image_resolution_y = 1242


camera.data.lens_unit = "FOV"
camera.data.angle = np.deg2rad(92.32963675846877)

scene = bpy.context.scene

scene.render.resolution_x = image_resolution_x
scene.render.resolution_y = image_resolution_y

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
checker_size = 0.04

board.data.transform(Matrix.Rotation(np.pi, 4, "X"))

x_shift = num_board_cols / 2 * 0.04
y_shift = num_board_rows / 2 * 0.04

# board.data.transform(Matrix.Translation((x_shift, 0, 0)))
board.data.transform(Matrix.Translation((x_shift, y_shift, 0)))

arm_joints, _, _, tool_link, gripper_joint = add_ur_with_robotiq()
gripper_joint.rotation_euler.z = np.deg2rad(42)

board_offset = np.array([0.0, -y_shift, 0.15])
board_transform = np.identity(4)

orientation = Matrix.Rotation(-np.pi / 2, 3, "Y")
board_transform[:3, :3] = orientation  # .to_numpy()
board_transform[:3, 3] = board_offset
board.parent = tool_link
board.matrix_parent_inverse = Matrix(board_transform)


K = ab.intrinsics_matrix_from_blender()

with np.printoptions(precision=3, suppress=True):
    print(K)

camera_intrinsics = CameraIntrinsics(
    image_resolution=Resolution(width=image_resolution_x, height=image_resolution_y),
    focal_lengths_in_pixels=FocalLengths(fx=K[0, 0], fy=K[1, 1]),
    principal_point_in_pixels=PrincipalPoint(cx=K[0, 2], cy=K[1, 2]),
)

print(camera_intrinsics)

with open("camera_intrinsics.json", "w") as file:
    json.dump(camera_intrinsics.dict(exclude_none=True), file, indent=4)


eef_poses = []  # TODO

# Generated with:
# np.printoptions(precision=3, suppress=True)
# np.random.uniform(-np.pi, np.pi, 6)
# But tweaked to make the board visible
joint_configurations = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [np.pi / 2, np.deg2rad(-142), np.deg2rad(77.4), -1.5707963267948966, np.deg2rad(190), -np.pi / 2],
    [np.deg2rad(-142), -1.97601766, np.deg2rad(-71.4), 2.36436238, -0.40880767, np.deg2rad(146)],
    [np.deg2rad(25), -1.26125591, np.deg2rad(-110), -3.10492817, -1.63491052, np.deg2rad(73)],
    [-2.37219922, np.deg2rad(250), -1.79723309, -2.50483946, np.deg2rad(35), 1.69997479],
    [-0.97493267, -3.0887579, 1.03988655, 1.05644123, 3.03990261, np.deg2rad(-316)],
]

for i, joint_configuration in enumerate(joint_configurations):
    for joint, angle in zip(arm_joints.values(), joint_configuration):
        joint.rotation_euler.z = angle

    bpy.context.view_layer.update()
    eef_poses.append(tool_link.matrix_world.copy())

    bpy.context.scene.render.filepath = f"pose{i:04d}.png"
    bpy.ops.render.render(write_still=True)
