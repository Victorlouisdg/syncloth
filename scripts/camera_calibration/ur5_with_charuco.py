import os

import bpy
import numpy as np
from mathutils import Matrix

from syncloth.robot_arms import add_ur_with_robotiq

bpy.ops.object.delete()  # Delete the default cube

camera = bpy.data.objects["Camera"]

# Set camera extrinsics
camera.location = (0, 0, 1)
camera.rotation_euler = (0, 0, 0)  # Look down the z-axis

# Set camera intrinsics
# All don't work
# Zed2i.RESOLUTION_2K.width,
# Zed2i.RESOLUTION_2K.getWidth()
# Zed2i.RESOLUTION_2K._w
image_resolution_x = 2208
image_resolution_y = 1242

sensor_witdh_mm = 5.376
sensor_height_mm = 3.04

camera.data.lens = 2.12  # focal length in mm
camera.data.sensor_width = sensor_witdh_mm  # mm
camera.data.sensor_height = sensor_height_mm  # mm

image_aspect_ratio = image_resolution_x / image_resolution_y
sensor_aspect_ratio = sensor_witdh_mm / sensor_height_mm

if image_aspect_ratio > sensor_aspect_ratio:
    camera.data.sensor_fit = "HORIZONTAL"  # image is wider, so fit to width
else:
    camera.data.sensor_fit = "VERTICAL"

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

# TODO save image to disk and pass to opencv


from airo_camera_toolkit.calibration.fiducial_markers import AIRO_DEFAULT_ARUCO_DICT, AIRO_DEFAULT_CHARUCO_BOARD

# aruco_marker_size = (0.031,)
# charuco_x_count = 7
# charuco_y_count = 5
# charuco_tile_size = (0.04,)

# aruco_dict = AIRO_DEFAULT_ARUCO_DICT
# detect_charuco = charuco_x_count is not None and charuco_y_count is not None and charuco_tile_size is not None
# if detect_charuco:
#     charuco_board = aruco.CharucoBoard(
#         (charuco_x_count, charuco_y_count), charuco_tile_size, aruco_marker_size, aruco_dict
#     )

aruco_dict = AIRO_DEFAULT_ARUCO_DICT
charuco_board = AIRO_DEFAULT_CHARUCO_BOARD

# camera = Zed2i()

# print("press Q to quit")
# while True:
#     aruco_result = None
#     charuco_result = None
#     aruco_poses = None
#     charuco_pose = None
#     # image = camera.get_rgb_image()

#     image = ImageConverter.from_numpy_format(image).image_in_opencv_format

#     intrinsics = camera.intrinsics_matrix()

#     aruco_result = detect_aruco_markers(image, aruco_dict)

#     if aruco_result:
#         aruco_poses = get_poses_of_aruco_markers(aruco_result, 0.04, intrinsics)

#     if aruco_result:
#         charuco_result = detect_charuco_corners(image, aruco_result, charuco_board)

#         if charuco_result:
#             charuco_pose = get_pose_of_charuco_board(charuco_result, charuco_board, intrinsics)
#     if aruco_result:
#         image = visualize_aruco_detections(image, aruco_result)
#         if aruco_poses is not None:
#             image = draw_frame_on_image(image, aruco_poses[0], intrinsics)

#     if charuco_result:
#         image = visualize_charuco_detection(image, charuco_result)
#         if charuco_pose is not None:
#             image = draw_frame_on_image(image, charuco_pose, intrinsics)

#     image = cv2.resize(image, (1024, 768))
#     cv2.imshow("image", image)
#     key = cv2.waitKey(1)
#     if key == ord("q"):
#         break
