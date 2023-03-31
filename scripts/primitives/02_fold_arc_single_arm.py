import bpy
import numpy as np
from synthetic_cloth_data.geometric_templates import TshirtMeshConfig, create_tshirt_vertices

from syncloth.folding.fold_arcs.pose_trajectories.circular_slerp import fold_arc_circular_slerp_trajectory
from syncloth.geometry import flat_orientation, pitch_gripper_orientation
from syncloth.grasping.slide_grasp import slide_grasp_orthogonal_approach_direction
from syncloth.paths.circular_arc import circular_arc_orientation_path
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
fold_end_angle = np.pi / 8


_, keypoints = create_tshirt_vertices(TshirtMeshConfig(sleeve_length=0.7))

for key, value in keypoints.items():
    keypoints[key] = value / 2.0

keypoints_3D = keypoints.values()

neck_left = keypoints["neck_left"]
sleeve_left_top = keypoints["sleeve_left_top"]
sleeve_left_bottom = keypoints["sleeve_left_bottom"]
armpit_left = keypoints["armpit_left"]

fold_line_point = armpit_left
armpit_to_neck = neck_left - armpit_left
fold_line_direction = armpit_to_neck / np.linalg.norm(armpit_to_neck)
fold_line = (fold_line_point, fold_line_direction)

# Get grasp location and the gripper orientation for a perfectly flat grasp
approach_direction = slide_grasp_orthogonal_approach_direction(sleeve_left_bottom, sleeve_left_top)
grasp_location = (sleeve_left_top + sleeve_left_bottom) / 2
grasp_location += grasp_depth * approach_direction
grasp_orientation_flat = flat_orientation(approach_direction)
circular_orientations = circular_arc_orientation_path(grasp_orientation_flat, fold_line_direction, np.pi)
end_orientation_flat = circular_orientations.end
middle_orientation = circular_orientations(np.pi / 2)

# Pitch the orientation of the start and end
grasp_orientation_pitched = pitch_gripper_orientation(grasp_orientation_flat, -approach_angle)
end_orientation_pitched = pitch_gripper_orientation(end_orientation_flat, fold_end_angle)

orientations = [grasp_orientation_pitched, middle_orientation, end_orientation_pitched]
fold_arc_trajectory = fold_arc_circular_slerp_trajectory(fold_line, grasp_location, orientations, fold_end_height)


# Visualization
bpy.ops.object.delete()
add_curve_mesh(keypoints_3D, closed=True)

start = fold_line_point - fold_line_direction * 0.5
end = fold_line_point + fold_line_direction * 0.3
add_points_as_instances([start, end], color=(1, 1, 0))
visualize_line_segment(start, end, radius=0.002)


visualize_trajectory(fold_arc_trajectory)
visualize_trajectory_frames(fold_arc_trajectory)
add_animated_robotiq(fold_arc_trajectory)
