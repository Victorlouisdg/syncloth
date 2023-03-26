import bpy
import numpy as np

from syncloth.folding.fold_arcs.bezier import fold_arc_position_trajectory_bezier
from syncloth.folding.fold_lines.towel import towel_fold_line
from syncloth.geometry import get_ordered_keypoints
from syncloth.grasping.sliding_grasp import sliding_grasp_position_trajectory
from syncloth.paths.linear import linear_trajectory
from syncloth.visualization.paths import add_path_as_points
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
ordered_keypoints = get_ordered_keypoints(keypoints_3D)
add_points_as_instances(ordered_keypoints, radius=0.01, color=(0.8, 0, 0))


# 1) determine whether to start the fold, or reposition the towel
# determine whether to grasp top edge or side edges
# 2) determine the fold line
# 3) determine the grasp points / grasped edge


def release_trajectory():
    pass


fold_line = towel_fold_line(ordered_keypoints)

top_right_corner, top_left_corner, bottom_left_corner, bottom_right_corner = ordered_keypoints

top_edge_left_to_right = top_right_corner - top_left_corner
top_edge_left_to_right_direction = top_edge_left_to_right / np.linalg.norm(top_edge_left_to_right)

left_edge_top_to_bottom = bottom_left_corner - top_left_corner
right_edge_top_to_bottom = bottom_right_corner - top_right_corner

grasp_approach_direction = (left_edge_top_to_bottom + right_edge_top_to_bottom) / 2

grasp_depth = 0.05
grasp_inset = 0.05
compliance_depth = 0.01

# Grasp locations on the border of the towel
grasp_location_left = top_left_corner.copy()
grasp_location_left += grasp_inset * top_edge_left_to_right_direction
grasp_location_left += grasp_depth * grasp_approach_direction
grasp_location_left -= np.array([0.0, 0.0, compliance_depth])

grasp_location_right = top_right_corner.copy()
grasp_location_right -= grasp_inset * top_edge_left_to_right_direction
grasp_location_right += grasp_depth * grasp_approach_direction
grasp_location_right -= np.array([0.0, 0.0, compliance_depth])

add_points_as_instances([grasp_location_left, grasp_location_right], radius=0.01, color=(0, 0.8, 0))

# TODO add grasp depth here etc
approach_margin = 0.05
approach_distance = grasp_depth + approach_margin
grasp_trajectory_left = sliding_grasp_position_trajectory(
    grasp_location_left, grasp_approach_direction, approach_distance
)
grasp_trajectory_right = sliding_grasp_position_trajectory(
    grasp_location_right, grasp_approach_direction, approach_distance
)

fold_end_height = 0.05 + compliance_depth
fold_arc_position_trajectory_left = fold_arc_position_trajectory_bezier(
    fold_line, grasp_location_left, fold_end_height
)
fold_arc_position_trajectory_right = fold_arc_position_trajectory_bezier(
    fold_line, grasp_location_right, fold_end_height
)

fold_end_left = fold_arc_position_trajectory_left.end_value
fold_end_right = fold_arc_position_trajectory_right.end_value

add_points_as_instances([fold_end_left, fold_end_right], radius=0.01, color=(0, 0, 0.8))

retreat_height = 0.1
retreat_vector = np.array([0, 0, retreat_height])
retreat_position_trajectory_left = linear_trajectory(fold_end_left, fold_end_left + retreat_vector, speed=0.1)
retreat_position_trajectory_right = linear_trajectory(fold_end_right, fold_end_right + retreat_vector, speed=0.1)


add_path_as_points(fold_arc_position_trajectory_left, num_points=100)
add_path_as_points(fold_arc_position_trajectory_right, num_points=100)

add_path_as_points(grasp_trajectory_left, num_points=100)
add_path_as_points(grasp_trajectory_right, num_points=100)

add_path_as_points(retreat_position_trajectory_left, num_points=100)
add_path_as_points(retreat_position_trajectory_right, num_points=100)
