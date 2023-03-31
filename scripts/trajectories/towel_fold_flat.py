import bpy
import numpy as np

from syncloth.folding.fold_arcs.pose_trajectories.circular_slerp import fold_arc_circular_slerp_trajectory
from syncloth.folding.fold_lines.towel import towel_fold_line
from syncloth.geometry import get_counterclockwise_ordered_keypoints
from syncloth.grasping.slide_grasp import (
    slide_grasp_constant_orientation_pose_trajectory,
    slide_grasp_locations_from_edge,
    slide_grasp_orthogonal_approach_direction,
)
from syncloth.paths.orientation import linear_position_constant_orientation_trajectory
from syncloth.visualization.dual_arm_fold import visualize_dual_arm_fold
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

# Scene parameters
towel_length = 1.2

# Trajectory parameters
grasp_depth = 0.05
grasp_inset = 0.05
approach_margin = 0.05
approach_distance = grasp_depth + approach_margin
approach_angle = -np.pi / 4
compliance_depth = 0.01
fold_end_height = 0.05 + compliance_depth

# Add the towel to the scene
bpy.ops.object.delete()
towel = add_towel(length=towel_length, height=0.0)

# Get its keypoints
bpy.context.view_layer.update()
keypoints_3D = [towel.matrix_world @ v.co for v in towel.data.vertices]
ordered_keypoints = get_counterclockwise_ordered_keypoints(keypoints_3D)

# Determine the fold line
fold_line = towel_fold_line(ordered_keypoints)

### Grasp point selection
# Determine the grasp points and apporach direction for this robot setup (split setup and grasp at the top)
top_right_corner, top_left_corner, bottom_left_corner, bottom_right_corner = ordered_keypoints
grasp_approach_direction = slide_grasp_orthogonal_approach_direction(top_left_corner, top_right_corner)

# Grasp locations on the border of the towel
grasp_location_left, grasp_location_right = slide_grasp_locations_from_edge(
    top_left_corner, top_right_corner, grasp_approach_direction, grasp_depth, grasp_inset
)

# Adding a bit of compliance
grasp_location_left -= np.array([0.0, 0.0, compliance_depth])
grasp_location_right -= np.array([0.0, 0.0, compliance_depth])


### Everything below here can be used with any grasp location and approach direction
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
        fold_line, grasp_location, grasp_trajectory(0)[:3, :3], fold_end_height
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


trajectories_left = fold_trajectories(
    fold_line, grasp_location_left, grasp_approach_direction, approach_distance, approach_angle, fold_end_height
)
trajectories_right = fold_trajectories(
    fold_line, grasp_location_right, grasp_approach_direction, approach_distance, approach_angle, fold_end_height
)

# Visualization
visualize_dual_arm_fold(
    ordered_keypoints,
    fold_line,
    trajectories_left,
    trajectories_right,
)
