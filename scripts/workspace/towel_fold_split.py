import airo_blender as ab
import bpy
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from ur_analytic_ik import ur3e, ur5e, ur10e

from syncloth.curves.bezier import quadratic_bezier
from syncloth.geometry import get_ordered_keypoints
from syncloth.robot_arms import add_ur_with_robotiq, set_joint_angles
from syncloth.visualization.curves import add_curve_mesh, skin
from syncloth.visualization.inverse_kinematics import keyframe_joints
from syncloth.visualization.points import add_points_as_instances
from syncloth.visualization.scene import add_table, add_towel

# Main parameters
towel_length = 0.68  # max for tilt angle 60
ur_type = "ur3e"
distance_between_robots = 0.8

# towel_length = 1.2
# ur_type = "ur5e"
# distance_between_robots = 1.0


# towel_length = 2.0
# ur_type = "ur10e"
# distance_between_robots = 1.5


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

ab.add_material(towel, (0.5, 0.5, 0.8))

bpy.context.view_layer.update()

keypoints_3D = [towel.matrix_world @ v.co for v in towel.data.vertices]

add_points_as_instances(keypoints_3D, radius=0.01, color=(0.8, 0, 0))

left_arm_joints, _, left_arm_base, _, _ = add_ur_with_robotiq(ur_type)
right_arm_joints, _, right_arm_base, _, _ = add_ur_with_robotiq(ur_type)

y = distance_between_robots / 2
left_arm_base.location = (0, y, table_height)

right_arm_base.location = (0, -y, table_height)
right_arm_base.rotation_euler = (0, 0, np.pi)

bpy.context.view_layer.update()

home_joints_left = np.deg2rad([0, -45, -90, 0, 0, 0])
home_joints_right = np.deg2rad([0, -45, -90, 0, 0, 0])

set_joint_angles(home_joints_left, left_arm_joints)
set_joint_angles(home_joints_right, right_arm_joints)

keyframe_joints(left_arm_joints, home_joints_left, 0)
keyframe_joints(right_arm_joints, home_joints_right, 0)

tcp_offset = np.array([0.0, 0.0, 0.172])
tcp_transform = np.identity(4)
tcp_transform[:3, 3] = tcp_offset

ordered_keypoints = get_ordered_keypoints(keypoints_3D)


def animate_arm(start_self, end_self, start_other, arm_in_world, arm_joints, home_joints):
    print("start", start_self, end_self)
    middle = (start_self + end_self) / 2
    middle[2] += np.linalg.norm(end_self - start_self) * 0.8
    print("middle", middle)
    points = np.vstack((start_self, middle, end_self))

    add_points_as_instances(points, radius=0.01, color=(0, 0.8, 0))

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
    tilt = -np.deg2rad(60)
    rotation_Y45 = Rotation.from_rotvec(tilt * Y).as_matrix()
    orientation_start = rotation_Y45 @ orientation_flat

    # rotate 90 degrees around local Y
    rotation_Y90 = Rotation.from_rotvec(-np.pi / 2 * Y).as_matrix()
    orientation_middle = rotation_Y90 @ orientation_flat

    # rotate 135 degrees around local Y
    final_tilt = -np.pi - tilt
    rotation_Y135 = Rotation.from_rotvec(final_tilt * Y).as_matrix()
    orientation_end = rotation_Y135 @ orientation_flat

    # use scipy SLERP to interpolate between the orientations
    orientations = np.array([orientation_start, orientation_middle, orientation_end])
    orientations = Rotation.from_matrix(orientations)
    slerp = Slerp([0.0, 0.5, 1.0], orientations)

    frame_poses = []  # for each frame a pose

    grasp_pose = np.identity(4)
    grasp_pose[:3, :3] = slerp(0).as_matrix()
    grasp_pose[:3, 3] = curve[0]

    pregrasp_pose = grasp_pose.copy()
    # shift the pregrasp pose 5 cm backwards along the local Z-axis of the untilted grasp pose
    pregrasp_pose[:3, 3] -= 0.03 * orientation_flat[:3, 2]

    hover_pose = pregrasp_pose.copy()
    # shift the hover pose 5 cm upwards
    hover_pose[2, 3] += 0.03

    fold_end_pose = np.identity(4)
    fold_end_pose[:3, :3] = slerp(1).as_matrix()
    fold_end_pose[:3, 3] = curve[-1]

    retreat_pose = fold_end_pose.copy()
    retreat_pose[2, 3] += 0.1
    # shift the retreat pose 5 cm upwards, but also change the orientation to top-down
    top_down = orientation_middle
    orientations2 = np.array([slerp(1).as_matrix(), top_down])
    orientations2 = Rotation.from_matrix(orientations2)
    slerp2 = Slerp([0.0, 1.0], orientations2)

    for i in range(11):
        pose = hover_pose + i / 10 * (pregrasp_pose - hover_pose)
        pose_in_arm = np.linalg.inv(arm_in_world) @ pose
        frame_poses.append(pose_in_arm)

    for i in range(11):
        pose = pregrasp_pose + i / 10 * (grasp_pose - pregrasp_pose)
        pose_in_arm = np.linalg.inv(arm_in_world) @ pose
        frame_poses.append(pose_in_arm)

    for i in range(num_samples):
        translation = curve[i]
        interpolated_orientation = slerp(t_range[i]).as_matrix()
        grasp = np.identity(4)
        grasp[:3, :3] = interpolated_orientation
        grasp[:3, 3] = translation
        grasp_in_arm = np.linalg.inv(arm_in_world) @ grasp
        frame_poses.append(grasp_in_arm)

    for i in range(21):
        translation = fold_end_pose[:3, 3] + i / 20 * (retreat_pose[:3, 3] - fold_end_pose[:3, 3])
        interpolated_orientation = slerp2(i / 20).as_matrix()
        pose = np.identity(4)
        pose[:3, :3] = interpolated_orientation
        pose[:3, 3] = translation
        pose_in_arm = np.linalg.inv(arm_in_world) @ pose
        frame_poses.append(pose_in_arm)

    prev_joints = home_joints
    for i, pose in enumerate(frame_poses):
        solutions = ur.inverse_kinematics_closest_with_tcp(pose, tcp_transform, *prev_joints)
        if len(solutions) == 0:
            print(f"No solution found for pose {i}: \n", pose)
            break
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
