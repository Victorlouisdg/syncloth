import airo_blender as ab
import bpy
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from ur_analytic_ik import ur3e, ur5e, ur10e

from syncloth.curves.bezier import quadratic_bezier
from syncloth.robot_arms import add_ur_with_robotiq, set_joint_angles
from syncloth.visualization.curves import add_curve_mesh, skin
from syncloth.visualization.inverse_kinematics import keyframe_joints
from syncloth.visualization.points import add_points_as_instances
from syncloth.visualization.scene import add_table, add_towel

# Main parameters

# towel_length = 0.42
# ur_type = "ur3e"
# distance_between_robots = 0.5
# height_of_robots_above_table = 0.2
# arm_roll = np.deg2rad(90)

towel_length = 0.8
ur_type = "ur5e"
distance_between_robots = 0.6
height_of_robots_above_table = 0.3
arm_roll = np.deg2rad(90)

# towel_length = 1.2
# ur_type = "ur10e"

# distance_between_robots = 0.6
# height_of_robots_above_table = 0.3
# arm_roll = np.deg2rad(90)

# distance_between_robots = 0.8
# height_of_robots_above_table = 0.05
# arm_roll = np.deg2rad(0)


if ur_type == "ur3e":
    ur = ur3e
elif ur_type == "ur5e":
    ur = ur5e
else:
    ur = ur10e


bpy.ops.object.delete()

table_height = 0.7
table = add_table(height=table_height, rotation_z=np.deg2rad(90))
towel = add_towel(length=towel_length, height=table_height + 0.003, rotation_z=np.deg2rad(90))

table.location.x += 0.7
towel.location.x += towel_length / 2 + 0.1

ab.add_material(towel, (0.5, 0.5, 0.8))

bpy.context.view_layer.update()
keypoints_3D = [towel.matrix_world @ v.co for v in towel.data.vertices]

add_points_as_instances(keypoints_3D, radius=0.01, color=(0.8, 0, 0))

left_arm_joints, _, left_arm_base, _, _ = add_ur_with_robotiq(ur_type)
right_arm_joints, _, right_arm_base, _, _ = add_ur_with_robotiq(ur_type)

y = distance_between_robots / 2
left_arm_base.location = (-0.05, y, table_height + height_of_robots_above_table)
left_arm_base.rotation_euler = (-arm_roll, 0, 0)

right_arm_base.location = (-0.05, -y, table_height + height_of_robots_above_table)
right_arm_base.rotation_euler = (arm_roll, 0, 0)

bpy.context.view_layer.update()

# Positions the camera
camera = bpy.data.objects["Camera"]
camera.location = (0, 0, 1.4)
camera.rotation_euler = (np.deg2rad(30), 0, np.deg2rad(-90))

# set lens and sensor size
camera.data.lens = 2.12
camera.data.sensor_width = 5.376
camera.data.sensor_height = 3.04

camera.data.display_size = 0.2

# set resolution to 720p
scene = bpy.context.scene
scene.render.resolution_x = 1280
scene.render.resolution_y = 720


# Get the two keypoints with the greatest distance to the robot
keypoints_by_distance = sorted(keypoints_3D, key=lambda x: np.linalg.norm(x))
close_keypoints = keypoints_by_distance[:2]
far_keypoints = keypoints_by_distance[-2:]  # sort is ascending, so last two are the farthest

close_keypoints_right_to_left = sorted(close_keypoints, key=lambda x: x[1])
close_right = close_keypoints_right_to_left[0]
close_left = close_keypoints_right_to_left[1]

far_keypoints_right_to_left = sorted(far_keypoints, key=lambda x: x[1])
far_right = far_keypoints_right_to_left[0]
far_left = far_keypoints_right_to_left[1]

home_joints_left = np.deg2rad([-180, -30, 75, -135, -45, 0])
home_joints_right = np.deg2rad([0, -150, -75, -45, 45, 0])

set_joint_angles(home_joints_left, left_arm_joints)
set_joint_angles(home_joints_right, right_arm_joints)

keyframe_joints(left_arm_joints, home_joints_left, 0)
keyframe_joints(right_arm_joints, home_joints_right, 0)

tcp_offset = np.array([0.0, 0.0, 0.172])
tcp_transform = np.identity(4)
tcp_transform[:3, 3] = tcp_offset


def animate_arm(far_self, close_self, far_other, arm_in_world, arm_joints, home_joints, left=True):

    far_to_close = close_self - far_self
    far_to_close_direction = far_to_close / np.linalg.norm(far_to_close)
    start = far_self + far_to_close_direction * 0.04
    end = close_self - far_to_close_direction * 0.04

    middle = (start + end) / 2
    middle[2] += 0.3
    points = np.vstack((start, middle, end))

    num_samples = 100
    t_range = np.linspace(0, 1, num_samples, endpoint=True)
    curve = np.array([quadratic_bezier(t, *points) for t in t_range])

    curve_mesh = add_curve_mesh(curve)
    ab.add_material(curve_mesh, color=(1, 0.5, 0))
    skin(curve_mesh, radius=0.005)

    Z = far_other - far_self
    Z = Z / np.linalg.norm(Z)
    X = np.array([0, 0, 1])
    Y = np.cross(Z, X)
    orientation = np.column_stack((X, Y, Z))

    # rotate 45 degrees around local Y
    rotation_Y = Rotation.from_rotvec(-np.pi / 4 * Y).as_matrix()
    orientation_start = rotation_Y @ orientation
    local_Z = orientation_start[:, 2]

    z_sign = 1 if left else -1
    rotation_Z90 = Rotation.from_rotvec(z_sign * np.pi / 2 * local_Z).as_matrix()
    orientation_middle = rotation_Z90 @ orientation_start

    rotation_Z180 = Rotation.from_rotvec(z_sign * np.pi * local_Z).as_matrix()
    orientation_end = rotation_Z180 @ orientation_start

    # use scipy SLERP to interpolate between the orientations
    orientations = np.array([orientation_start, orientation_middle, orientation_end])
    orientations = Rotation.from_matrix(orientations)
    slerp = Slerp([0.0, 0.5, 1.0], orientations)

    prev_joints = home_joints

    for i in range(num_samples):
        translation = curve[i]
        interpolated_orientation = slerp(t_range[i]).as_matrix()
        grasp = np.identity(4)
        grasp[:3, :3] = interpolated_orientation
        grasp[:3, 3] = translation
        grasp_in_arm = np.linalg.inv(arm_in_world) @ grasp

        solutions = ur.inverse_kinematics_closest_with_tcp(grasp_in_arm, tcp_transform, *prev_joints)

        if len(solutions) == 0:
            print(f"No solution found for pose {i}: \n", grasp_in_arm)
            break

        keyframe_joints(arm_joints, *solutions[0], i + 1)
        prev_joints = solutions[0][0]


animate_arm(
    far_left, close_left, far_right, np.array(left_arm_base.matrix_world), left_arm_joints, home_joints_left, left=True
)
animate_arm(
    far_right,
    close_right,
    far_left,
    np.array(right_arm_base.matrix_world),
    right_arm_joints,
    home_joints_right,
    left=False,
)
