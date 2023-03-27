import bpy
import numpy as np

from syncloth.folding.fold_arcs.bezier import fold_arc_position_trajectory_bezier
from syncloth.folding.fold_arcs.orientation_trajectories import fold_arc_orientation_trajectory
from syncloth.folding.fold_lines.towel import towel_fold_line
from syncloth.geometry import get_counterclockwise_ordered_keypoints
from syncloth.grasping.slide_grasp import grasp_locations_from_edge, slide_grasp_constant_orientation_pose_trajectory
from syncloth.paths.constant import constant_trajectory
from syncloth.paths.linear import linear_trajectory
from syncloth.paths.orientation import combine_orientation_and_position_trajectory
from syncloth.visualization.frame import add_frame
from syncloth.visualization.paths import add_pose_path_as_points
from syncloth.visualization.points import add_points_as_instances
from syncloth.visualization.scene import add_towel

"""
Tow-down view of the table with the towel on it.

     +--------+
     |        |
     |  1--0  |
left |  |  |  | right
     |  2--3  |
     |        |
     +--------+

       camera
y
|
+--x
"""

# Main parameters
towel_length = 1.2

bpy.ops.object.delete()

towel = add_towel(length=towel_length, height=0.0)

bpy.context.view_layer.update()
keypoints_3D = [towel.matrix_world @ v.co for v in towel.data.vertices]
ordered_keypoints = get_counterclockwise_ordered_keypoints(keypoints_3D)

# 1) determine whether to start the fold, or reposition the towel
# determine whether to grasp top edge or side edges
# 2) determine the fold line
# 3) determine the grasp points / grasped edge

# Determine the grasp points for this robot setup (split)
top_right_corner, top_left_corner, bottom_left_corner, bottom_right_corner = ordered_keypoints

top_edge_left_to_right = top_right_corner - top_left_corner
top_edge_left_to_right_direction = top_edge_left_to_right / np.linalg.norm(top_edge_left_to_right)

left_edge_top_to_bottom = bottom_left_corner - top_left_corner
right_edge_top_to_bottom = bottom_right_corner - top_right_corner

grasp_approach_direction = (left_edge_top_to_bottom + right_edge_top_to_bottom) / 2

# Grasp locations on the border of the towel
grasp_depth = 0.05
grasp_inset = 0.05
grasp_location_left, grasp_location_right = grasp_locations_from_edge(
    top_left_corner, top_right_corner, grasp_approach_direction, grasp_depth, grasp_inset
)

# Adding a bit of compliance
compliance_depth = 0.01
grasp_location_left -= np.array([0.0, 0.0, compliance_depth])
grasp_location_right -= np.array([0.0, 0.0, compliance_depth])


# Grasp trajectories
approach_margin = 0.05
approach_distance = grasp_depth + approach_margin
approach_angle = -np.pi / 4

grasp_trajectory_left = slide_grasp_constant_orientation_pose_trajectory(
    grasp_location_left, grasp_approach_direction, approach_distance, approach_angle
)

grasp_trajectory_right = slide_grasp_constant_orientation_pose_trajectory(
    grasp_location_right, grasp_approach_direction, approach_distance, approach_angle
)

# Fold arc trajectories
fold_line = towel_fold_line(ordered_keypoints)


fold_end_height = 0.05 + compliance_depth
fold_arc_position_trajectory_left = fold_arc_position_trajectory_bezier(
    fold_line, grasp_location_left, fold_end_height
)

fold_arc_orienation_trajectory_left = fold_arc_orientation_trajectory(
    fold_arc_position_trajectory_left, grasp_trajectory_left.function(0)[:3, :3]
)

fold_arc_trajectory_left = combine_orientation_and_position_trajectory(
    fold_arc_orienation_trajectory_left, fold_arc_position_trajectory_left
)

fold_arc_position_trajectory_right = fold_arc_position_trajectory_bezier(
    fold_line, grasp_location_right, fold_end_height
)

fold_arc_orienation_trajectory_right = fold_arc_orientation_trajectory(
    fold_arc_position_trajectory_right, grasp_trajectory_right.function(0)[:3, :3]
)

fold_arc_trajectory_right = combine_orientation_and_position_trajectory(
    fold_arc_orienation_trajectory_right, fold_arc_position_trajectory_right
)

# Retreat trajectories
fold_end_left = fold_arc_position_trajectory_left.end_value
fold_end_right = fold_arc_position_trajectory_right.end_value
retreat_height = 0.1
retreat_vector = np.array([0, 0, retreat_height])
retreat_position_trajectory_left = linear_trajectory(fold_end_left, fold_end_left + retreat_vector, speed=0.1)
retreat_position_trajectory_right = linear_trajectory(fold_end_right, fold_end_right + retreat_vector, speed=0.1)

fold_end_orientation_left = fold_arc_orienation_trajectory_left.end_value
retreat_orienation_trajectory_left = constant_trajectory(
    fold_end_orientation_left, retreat_position_trajectory_left.duration
)
retreat_trajectory_left = combine_orientation_and_position_trajectory(
    retreat_orienation_trajectory_left, retreat_position_trajectory_left
)

fold_end_orientation_right = fold_arc_orienation_trajectory_right.end_value
retreat_orientation_trajectory_right = constant_trajectory(
    fold_end_orientation_right, retreat_position_trajectory_right.duration
)
retreat_trajectory_right = combine_orientation_and_position_trajectory(
    retreat_orientation_trajectory_right, retreat_position_trajectory_right
)


# Visualization
blender_red = [0.930111, 0.036889, 0.084376, 1.000000]
blender_green = [0.205079, 0.527115, 0.006049, 1.000000]
blender_blue = [0.028426, 0.226966, 0.760525, 1.000000]

add_points_as_instances(ordered_keypoints, radius=0.01, color=(0.8, 0, 0))
add_points_as_instances([grasp_location_left, grasp_location_right], radius=0.01, color=(0, 0.8, 0))
add_points_as_instances([fold_end_left, fold_end_right], radius=0.01, color=(0, 0, 0.8))

add_pose_path_as_points(grasp_trajectory_left, num_points=100, color=blender_red)
add_pose_path_as_points(grasp_trajectory_right, num_points=100, color=blender_red)
add_pose_path_as_points(fold_arc_trajectory_left, num_points=100, color=blender_green)
add_pose_path_as_points(fold_arc_trajectory_right, num_points=100, color=blender_green)
add_pose_path_as_points(retreat_trajectory_left, num_points=100, color=blender_blue)
add_pose_path_as_points(retreat_trajectory_right, num_points=100, color=blender_blue)

grasp_slide_duration = max(grasp_trajectory_left.duration, grasp_trajectory_right.duration)
fold_arc_duration = max(fold_arc_trajectory_left.duration, fold_arc_trajectory_right.duration)

# Grasp start and end frames
add_frame(grasp_trajectory_left.function(0))
add_frame(grasp_trajectory_right.function(0))
add_frame(grasp_trajectory_left.function(grasp_slide_duration))
add_frame(grasp_trajectory_right.function(grasp_slide_duration))

# Fold arc start and end frames
add_frame(fold_arc_trajectory_left.function(0))
add_frame(fold_arc_trajectory_right.function(0))
add_frame(fold_arc_trajectory_left.function(0.5 * fold_arc_duration))
add_frame(fold_arc_trajectory_right.function(0.5 * fold_arc_duration))
add_frame(fold_arc_trajectory_left.function(fold_arc_duration))
add_frame(fold_arc_trajectory_right.function(fold_arc_duration))

# Retreat start and end frames
add_frame(retreat_trajectory_left.function(0))
add_frame(retreat_trajectory_right.function(0))
add_frame(retreat_trajectory_left.function(retreat_trajectory_left.duration))
add_frame(retreat_trajectory_right.function(retreat_trajectory_right.duration))
