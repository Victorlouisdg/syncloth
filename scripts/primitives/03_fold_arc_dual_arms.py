import bpy
import numpy as np

from syncloth.folding.fold_arcs.pose_trajectories.circular_slerp import fold_arc_circular_slerp_trajectory
from syncloth.folding.fold_lines.towel import towel_fold_line
from syncloth.geometry import flat_orientation, get_counterclockwise_ordered_keypoints, pitch_gripper_orientation
from syncloth.grasping.slide_grasp import slide_grasp_locations_from_edge, slide_grasp_orthogonal_approach_direction
from syncloth.paths.circular_arc import circular_arc_orientation_path
from syncloth.robot_arms import add_animated_robotiq
from syncloth.visualization.curves import visualize_line_segment
from syncloth.visualization.paths import visualize_trajectory, visualize_trajectory_frames
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

# Scene parameters
towel_length = 1.2

# Trajectory parameters
grasp_depth = 0.02
grasp_inset = 0.05
approach_margin = 0.05
approach_distance = grasp_depth + approach_margin
approach_angle = np.pi / 4
fold_end_height = 0.05
fold_end_angle = np.pi / 8

# Add the towel to the scene
bpy.ops.object.delete()
towel = add_towel(length=towel_length, height=0.0)

# Get its keypoints
bpy.context.view_layer.update()
keypoints_3D = [towel.matrix_world @ v.co for v in towel.data.vertices]
ordered_keypoints = get_counterclockwise_ordered_keypoints(keypoints_3D)

# Determine the fold line
fold_line = towel_fold_line(ordered_keypoints)
fold_line_point, fold_line_direction = fold_line

### Grasp point selection
# Determine the grasp points and apporach direction for this robot setup (split setup and grasp at the top)
top_right_corner, top_left_corner, bottom_left_corner, bottom_right_corner = ordered_keypoints
approach_direction = slide_grasp_orthogonal_approach_direction(top_left_corner, top_right_corner)

# Grasp locations on the border of the towel
grasp_location_left, grasp_location_right = slide_grasp_locations_from_edge(
    top_left_corner, top_right_corner, approach_direction, grasp_depth, grasp_inset
)

grasp_orientation_flat = flat_orientation(approach_direction)
circular_orientations = circular_arc_orientation_path(grasp_orientation_flat, fold_line_direction, np.pi)
end_orientation_flat = circular_orientations.end
middle_orientation = circular_orientations(np.pi / 2)

# Pitch the orientation of the start and end
grasp_orientation_pitched = pitch_gripper_orientation(grasp_orientation_flat, -approach_angle)
end_orientation_pitched = pitch_gripper_orientation(end_orientation_flat, fold_end_angle)

orientations = [grasp_orientation_pitched, middle_orientation, end_orientation_pitched]
fold_arc_trajectory_left = fold_arc_circular_slerp_trajectory(
    fold_line, grasp_location_left, orientations, fold_end_height
)
fold_arc_trajectory_right = fold_arc_circular_slerp_trajectory(
    fold_line, grasp_location_right, orientations, fold_end_height
)

# Visualization
bpy.ops.object.delete()

start = fold_line_point - fold_line_direction * 0.5
end = fold_line_point + fold_line_direction * 0.3
add_points_as_instances([start, end], color=(1, 1, 0))
visualize_line_segment(start, end, radius=0.002)


visualize_trajectory(fold_arc_trajectory_left)
visualize_trajectory_frames(fold_arc_trajectory_left, frames_per_second=10)
add_animated_robotiq(fold_arc_trajectory_left)
visualize_trajectory(fold_arc_trajectory_right)
visualize_trajectory_frames(fold_arc_trajectory_right, frames_per_second=10)
add_animated_robotiq(fold_arc_trajectory_right)
