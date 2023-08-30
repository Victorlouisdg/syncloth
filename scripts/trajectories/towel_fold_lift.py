import bpy
import numpy as np

from syncloth.folding.fold_lines.towel import towel_fold_line
from syncloth.geometry import get_counterclockwise_ordered_keypoints
from syncloth.grasping.slide_grasp import slide_grasp_trajectory
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
approach_angle = np.pi / 4
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
fold_line_center, fold_line_direction = fold_line

### Grasp point selection
# Determine the grasp points and apporach direction for this robot setup (split setup and grasp at the top)
top_right_corner, top_left_corner, bottom_left_corner, bottom_right_corner = ordered_keypoints

# TODO change these to where the fold line intersects the towel
left_middle = (top_left_corner + bottom_left_corner) / 2
right_middle = (top_right_corner + bottom_right_corner) / 2

left_approach_direction = fold_line_direction
right_approach_direction = -fold_line_direction

grasp_location_left = left_middle + 0.05 * left_approach_direction
grasp_location_right = right_middle + 0.05 * right_approach_direction

# Adding a bit of compliance
# grasp_location_left -= np.array([0.0, 0.0, compliance_depth])
# grasp_location_right -= np.array([0.0, 0.0, compliance_depth])

left_edge_length = np.linalg.norm(top_left_corner - bottom_left_corner)
right_edge_length = np.linalg.norm(top_right_corner - bottom_right_corner)
towel_length_estimate = (left_edge_length + right_edge_length) / 2


### Everything below here can be used with any grasp location and approach direction
# Grasp trajectories
def fold_lift_trajectories(
    fold_line,
    grasp_location,
    grasp_approach_direction,
    approach_distance,
    approach_angle,
    fold_end_height,
    towel_length_estimate,
):
    grasp_trajectory = slide_grasp_trajectory(
        grasp_location, grasp_approach_direction, approach_distance, approach_angle
    )

    grasp_orienation = grasp_trajectory.end[:3, :3]

    lift_start = grasp_trajectory.end[:3, 3]
    lift_vector = np.array([0, 0, 1.1 * towel_length_estimate / 2])
    lift_end = lift_start + lift_vector

    lift_trajectory = linear_position_constant_orientation_trajectory(
        lift_start, lift_end, grasp_orienation, speed=0.25
    )

    swing_start = lift_end
    _, fold_direction = fold_line
    up = np.array([0, 0, 1])
    swing_vector = towel_length_estimate / 2 * np.cross(fold_direction, up)
    swing_end = swing_start + swing_vector
    swing_trajectory = linear_position_constant_orientation_trajectory(
        swing_start, swing_end, grasp_orienation, speed=1.0
    )

    laydown_start = swing_end
    laydown_end = grasp_location - swing_vector
    laydown_trajectory = linear_position_constant_orientation_trajectory(
        laydown_start, laydown_end, grasp_orienation, speed=1.0
    )

    return [grasp_trajectory, lift_trajectory, swing_trajectory, laydown_trajectory]


trajectories_left = fold_lift_trajectories(
    fold_line,
    grasp_location_left,
    left_approach_direction,
    approach_distance,
    approach_angle,
    fold_end_height,
    towel_length_estimate,
)
trajectories_right = fold_lift_trajectories(
    fold_line,
    grasp_location_right,
    right_approach_direction,
    approach_distance,
    approach_angle,
    fold_end_height,
    towel_length_estimate,
)

# Visualization
visualize_dual_arm_fold(
    ordered_keypoints,
    fold_line,
    trajectories_left,
    trajectories_right,
)
