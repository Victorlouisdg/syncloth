import airo_blender as ab
import bpy
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from ur_analytic_ik import ur5e

from syncloth.curves.bezier import quadratic_bezier
from syncloth.geometry import get_ordered_keypoints
from syncloth.robot_arms import add_ur_with_robotiq, set_joint_angles
from syncloth.visualization.curves import add_curve_mesh, skin
from syncloth.visualization.inverse_kinematics import keyframe_joints
from syncloth.visualization.points import add_points_as_instances
from syncloth.visualization.scene import add_table, add_towel

bpy.ops.object.delete()

table_height = 0.7
table = add_table(height=table_height, rotation_z=np.deg2rad(90))
towel = add_towel(height=table_height + 0.003, rotation_z=np.deg2rad(90))

ab.add_material(towel, (0.5, 0.5, 0.8))

bpy.context.view_layer.update()

keypoints_3D = [towel.matrix_world @ v.co for v in towel.data.vertices]

add_points_as_instances(keypoints_3D, radius=0.01, color=(0.8, 0, 0))

left_arm_joints, _, left_arm_base, _, _ = add_ur_with_robotiq("ur5e")
right_arm_joints, _, right_arm_base, _, _ = add_ur_with_robotiq("ur5e")

y = 0.5
left_arm_base.location = (0, y, table_height)

right_arm_base.location = (0, -y, table_height)
right_arm_base.rotation_euler = (0, 0, np.pi)

bpy.context.view_layer.update()

home_joints_left = np.deg2rad([0, -45, -90, 0, 0, 0])
home_joints_right = np.deg2rad([0, -45, -90, 0, 0, 0])

set_joint_angles(home_joints_left, left_arm_joints)
set_joint_angles(home_joints_right, right_arm_joints)

tcp_offset = np.array([0.0, 0.0, 0.172])
tcp_transform = np.identity(4)
tcp_transform[:3, 3] = tcp_offset

ordered_keypoints = get_ordered_keypoints(keypoints_3D)


def animate_arm(start_self, end_self, start_other, arm_in_world, arm_joints, home_joints):
    middle = (start_self + end_self) / 2
    middle[2] += np.linalg.norm(end_self - start_self)
    points = np.vstack((start_self, middle, end_self))

    num_samples = 100
    t_range = np.linspace(0, 1, num_samples, endpoint=True)
    curve = np.array([quadratic_bezier(t, *points) for t in t_range])

    curve_mesh = add_curve_mesh(curve)
    ab.add_material(curve_mesh, color=(1, 0.5, 0))
    skin(curve_mesh, radius=0.005)

    Z = end_self - start_self
    Z = Z / np.linalg.norm(Z)
    X = np.array([0, 0, 1])
    Y = np.cross(Z, X)
    orientation_flat = np.column_stack((X, Y, Z))

    # rotate 45 degrees around local Y
    rotation_Y45 = Rotation.from_rotvec(-np.pi / 4 * Y).as_matrix()
    orientation_start = rotation_Y45 @ orientation_flat

    # rotate 90 degrees around local Y
    rotation_Y90 = Rotation.from_rotvec(-np.pi / 2 * Y).as_matrix()
    orientation_middle = rotation_Y90 @ orientation_flat

    # rotate 135 degrees around local Y
    rotation_Y135 = Rotation.from_rotvec(-3 * np.pi / 4 * Y).as_matrix()
    orientation_end = rotation_Y135 @ orientation_flat

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

        solutions = ur5e.inverse_kinematics_closest_with_tcp(grasp_in_arm, tcp_transform, *prev_joints)
        keyframe_joints(arm_joints, *solutions[0], i + 1)
        prev_joints = solutions[0][0]


animate_arm(
    ordered_keypoints[0],
    ordered_keypoints[1],
    ordered_keypoints[3],
    np.array(left_arm_base.matrix_world),
    left_arm_joints,
    home_joints_left,
)
animate_arm(
    ordered_keypoints[3],
    ordered_keypoints[2],
    ordered_keypoints[0],
    np.array(right_arm_base.matrix_world),
    right_arm_joints,
    home_joints_right,
)
