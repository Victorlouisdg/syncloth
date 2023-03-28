import bpy
import numpy as np

from syncloth.grasping.slide_grasp import (
    slide_grasp_constant_orientation_pose_trajectory,
    slide_grasp_orthogonal_approach_direction,
)
from syncloth.robot_arms import add_animated_robotiq
from syncloth.visualization.curves import visualize_line_segment
from syncloth.visualization.paths import visualize_trajectory
from syncloth.visualization.points import add_points_as_instances

edge_start = np.array([0.05, 0.0, 0.0])
edge_end = -edge_start
edge_middle = (edge_start + edge_end) / 2

grasp_approach_direction = slide_grasp_orthogonal_approach_direction(edge_start, edge_end)

speed = 0.05
hover_height = 0.04
grasp_depth = 0.05
approach_margin = 0.02
approach_distance = grasp_depth + approach_margin
grasp_location = edge_middle + grasp_depth * grasp_approach_direction
approach_angle = np.pi / 4

grasp_trajectory = slide_grasp_constant_orientation_pose_trajectory(
    grasp_location, grasp_approach_direction, approach_distance, approach_angle, hover_height, speed
)

# Visualization
bpy.ops.object.delete()
add_points_as_instances([edge_start, edge_end], color=(1, 1, 0))
visualize_line_segment(edge_start, edge_end, radius=0.002)
visualize_trajectory(grasp_trajectory)
add_animated_robotiq(grasp_trajectory)
