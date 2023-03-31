import bpy
import numpy as np
from synthetic_cloth_data.geometric_templates import TshirtMeshConfig, create_tshirt_vertices

from syncloth.folding.fold_arcs.pose_trajectories.circular_slerp import fold_arc_circular_slerp_trajectory
from syncloth.grasping.slide_grasp import (
    slide_grasp_constant_orientation_pose_trajectory,
    slide_grasp_orthogonal_approach_direction,
)
from syncloth.paths.orientation import linear_position_constant_orientation_trajectory
from syncloth.paths.retiming import scale_speed
from syncloth.visualization.curves import add_curve_mesh
from syncloth.visualization.dual_arm_fold import visualize_dual_arm_fold

# Trajectory parameters
grasp_depth = 0.05
approach_margin = 0.05
approach_distance = grasp_depth + approach_margin
approach_angle = -np.pi / 4
fold_end_height = 0.05


bpy.ops.object.delete()

vertices, keypoints = create_tshirt_vertices(TshirtMeshConfig())
keypoints_3D = keypoints.values()

add_curve_mesh(vertices, closed=True)

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

fold_line_point = neck_left - 0.05 * left_to_right / np.linalg.norm(left_to_right)
fold_line_direction = bottom_to_top / np.linalg.norm(bottom_to_top)
fold_line = (fold_line_point, fold_line_direction)

approach_direction_sleeve = slide_grasp_orthogonal_approach_direction(sleeve_left_bottom, sleeve_left_top)
grasp_location_sleeve = (sleeve_left_top + sleeve_left_bottom) / 2
grasp_location_sleeve += grasp_depth * approach_direction_sleeve

approach_direction_waist = slide_grasp_orthogonal_approach_direction(waist_left, armpit_left)
waist_to_armpit = armpit_left - waist_left
waist_to_armpit_direction = waist_to_armpit / np.linalg.norm(waist_to_armpit)
grasp_location_waist = waist_left + 0.05 * waist_to_armpit_direction
grasp_location_waist += grasp_depth * approach_direction_waist


# Grasp trajectories
def fold_trajectories(
    fold_line,
    grasp_location,
    grasp_approach_direction,
    approach_distance,
    approach_angle,
    fold_end_height,
    retreat_height=0.1,
):
    grasp_trajectory = slide_grasp_constant_orientation_pose_trajectory(
        grasp_location, grasp_approach_direction, approach_distance, approach_angle
    )

    # TODO fix orientations
    fold_arc_trajectory = fold_arc_circular_slerp_trajectory(
        fold_line, grasp_location, grasp_trajectory(0)[:3, :3], fold_end_height, approach_angle
    )

    # Alternatively, use the Bezier trajectory
    # fold_arc_trajectory = fold_arc_bezier_slerp_trajectory(
    #     fold_line, grasp_location, grasp_trajectory(0)[:3, :3], fold_end_height
    # )

    fold_end = fold_arc_trajectory.end_value[:3, 3]
    retreat_vector = np.array([0, 0, retreat_height])
    retreat_trajectory = linear_position_constant_orientation_trajectory(
        fold_end, fold_end + retreat_vector, fold_arc_trajectory.end_value[:3, :3], speed=0.1
    )
    return [grasp_trajectory, fold_arc_trajectory, retreat_trajectory]


trajectories_sleeve = fold_trajectories(
    fold_line, grasp_location_sleeve, approach_direction_sleeve, approach_distance, approach_angle, fold_end_height
)
trajectories_waist = fold_trajectories(
    fold_line, grasp_location_waist, approach_direction_waist, approach_distance, approach_angle, fold_end_height
)

sleeve_fold_arc_trajectory_left = trajectories_sleeve[1]
waist_fold_arc_trajectory_right = trajectories_waist[1]

# slow down the waist trajectory to match the duration of the sleeve trajectory
speed_factor = waist_fold_arc_trajectory_right.duration / sleeve_fold_arc_trajectory_left.duration
trajectories_waist[1] = scale_speed(waist_fold_arc_trajectory_right, speed_factor)

# Visualization
visualize_dual_arm_fold(
    keypoints_3D,
    fold_line,
    trajectories_sleeve,
    trajectories_waist,
)


# add_points_as_instances([grasp_location_sleeve, grasp_location_waist], color=(0.0, 0.0, 1.0))

# yellow = [1.0, 1.0, 0.0]
# add_points_as_instances(keypoints_3D, color=yellow)


# # fold_line_point, fold_line_direction = fold_line
# scale = 0.5
# fold_line_points = [
#     fold_line_point + 0.1 * fold_line_direction,
#     fold_line_point - (np.linalg.norm(bottom_to_top) + 0.1) * fold_line_direction,
# ]
# fold_line_mesh = add_curve_mesh(fold_line_points)
# ab.add_material(fold_line_mesh, yellow)
# skin(fold_line_mesh)
