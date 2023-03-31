import bpy
import numpy as np
from scipy.spatial.transform import Rotation
from synthetic_cloth_data.geometric_templates import TshirtMeshConfig, create_tshirt_vertices

from syncloth.folding.fold_arcs.pose_trajectories.circular_slerp import fold_arc_circular_slerp_trajectory
from syncloth.geometry import (
    flat_orientation,
    pitch_gripper_orientation,
    project_point_on_line,
    reflect_point_over_line,
)
from syncloth.grasping.slide_grasp import slide_grasp_orthogonal_approach_direction
from syncloth.robot_arms import add_animated_robotiq
from syncloth.visualization.curves import add_curve_mesh, visualize_line_segment
from syncloth.visualization.paths import visualize_trajectory, visualize_trajectory_frames
from syncloth.visualization.points import add_points_as_instances

# Trajectory parameters
grasp_depth = 0.02
approach_margin = 0.05
approach_distance = grasp_depth + approach_margin
approach_angle = np.pi / 4
fold_end_height = 0.05


_, keypoints = create_tshirt_vertices(TshirtMeshConfig(sleeve_length=0.7))

for key, value in keypoints.items():
    keypoints[key] = value / 2.0

keypoints_3D = keypoints.values()


# bottom_to_top_direction

shoulder_left = keypoints["shoulder_left"]
shoulder_right = keypoints["shoulder_right"]
waist_left = keypoints["waist_left"]
waist_right = keypoints["waist_right"]
neck_left = keypoints["neck_left"]
sleeve_left_top = keypoints["sleeve_left_top"]
sleeve_left_bottom = keypoints["sleeve_left_bottom"]
armpit_left = keypoints["armpit_left"]

top_middle = (shoulder_left + shoulder_right) / 2
bottom_middle = (waist_left + waist_right) / 2
bottom_to_top = top_middle - bottom_middle

left_middle = (shoulder_left + waist_left) / 2
right_middle = (shoulder_right + waist_right) / 2
left_to_right = right_middle - left_middle

fold_line_point = armpit_left
armpit_to_neck = neck_left - armpit_left
fold_line_direction = armpit_to_neck / np.linalg.norm(armpit_to_neck)
fold_line = (fold_line_point, fold_line_direction)

approach_direction = slide_grasp_orthogonal_approach_direction(sleeve_left_bottom, sleeve_left_top)
grasp_location = (sleeve_left_top + sleeve_left_bottom) / 2
grasp_location += grasp_depth * approach_direction


grasp_point_projection = project_point_on_line(grasp_location, fold_line)
grasp_point_reflected = reflect_point_over_line(grasp_location, fold_line)

grasp_orientation_flat = flat_orientation(approach_direction)
fold_arc_middle_orientation = (
    Rotation.from_rotvec(np.pi / 2 * fold_line_direction).as_matrix() @ grasp_orientation_flat
)
fold_arc_end_orientation_flat = Rotation.from_rotvec(np.pi * fold_line_direction).as_matrix() @ grasp_orientation_flat

grasp_orientation_pitched = pitch_gripper_orientation(grasp_orientation_flat, -approach_angle)
fold_arc_end_orientation_pitched = pitch_gripper_orientation(fold_arc_end_orientation_flat, approach_angle)

# grasp_orientation = slide_grasp_orientation(approach_direction, approach_angle)
# grasp_orientation = flat_orientation(approach_direction)
orientations = [grasp_orientation_pitched, fold_arc_middle_orientation, fold_arc_end_orientation_pitched]

fold_arc_trajectory = fold_arc_circular_slerp_trajectory(fold_line, grasp_location, orientations, fold_end_height)


# Visualization
bpy.ops.object.delete()
add_curve_mesh(keypoints_3D, closed=True)

start = fold_line_point - fold_line_direction * 0.5
end = fold_line_point + fold_line_direction * 0.3
add_points_as_instances([start, end], color=(1, 1, 0))
visualize_line_segment(start, end, radius=0.002)

add_points_as_instances([grasp_location, grasp_point_projection, grasp_point_reflected], color=(1, 0, 0))

# add_frame(fold_arc_trajectory(0.5 * fold_arc_trajectory.duration))

visualize_trajectory(fold_arc_trajectory)
visualize_trajectory_frames(fold_arc_trajectory)
add_animated_robotiq(fold_arc_trajectory)

# add_points_as_instances(rotated_points, color=(0, 1, 0), radius=0.002)

# for angle in np.linspace(0, np.pi, 20):
#     translation = rotate_point(grasp_location, fold_line_point, fold_line_direction, angle)

#     rotation = Rotation.from_rotvec(angle * fold_line_direction).as_matrix()

#     orientation = flat_orientation(approach_direction)
#     orientation = rotation @ orientation
#     # orientation = rotation @ grasp_orientation

#     pose  = np.identity(4)
#     pose[:3, :3] = orientation
#     pose[:3, 3] = translation

#     add_frame(pose, size =0.025)
