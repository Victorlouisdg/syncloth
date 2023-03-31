import bpy
import numpy as np
from synthetic_cloth_data.geometric_templates import TshirtMeshConfig, create_tshirt_vertices

from syncloth.geometry import flat_orientation
from syncloth.grasping.slide_grasp import slide_grasp_orthogonal_approach_direction
from syncloth.paths.circular_arc import circular_arc_pose_path
from syncloth.visualization.curves import add_curve_mesh, visualize_line_segment
from syncloth.visualization.frame import add_frame
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
waist_left = keypoints["waist_left"]
neck_left = keypoints["neck_left"]
sleeve_left_top = keypoints["sleeve_left_top"]
sleeve_left_bottom = keypoints["sleeve_left_bottom"]
armpit_left = keypoints["armpit_left"]

fold_line_point = armpit_left
armpit_to_neck = neck_left - armpit_left
fold_line_direction = armpit_to_neck / np.linalg.norm(armpit_to_neck)
fold_line = (fold_line_point, fold_line_direction)

approach_direction = slide_grasp_orthogonal_approach_direction(sleeve_left_bottom, sleeve_left_top)
grasp_location = (sleeve_left_top + sleeve_left_bottom) / 2
grasp_location += grasp_depth * approach_direction

grasp_orientation_flat = flat_orientation(approach_direction)

start_pose = np.identity(4)
start_pose[:3, :3] = grasp_orientation_flat
start_pose[:3, 3] = grasp_location
poses = circular_arc_pose_path(start_pose, *fold_line, np.pi)

# You can also get the position and orientation separately
# positions = circular_arc_position_path(grasp_location, *fold_line, np.pi)
# orientations = circular_arc_orientation_path(grasp_orientation_flat, fold_line_direction, np.pi)

# Visualization
bpy.ops.object.delete()
add_curve_mesh(keypoints_3D, closed=True)

start = fold_line_point - fold_line_direction * 0.5
end = fold_line_point + fold_line_direction * 0.3
add_points_as_instances([start, end], color=(1, 1, 0))
visualize_line_segment(start, end, radius=0.002)

for t in np.linspace(0, np.pi, 50):
    add_frame(poses(t))
