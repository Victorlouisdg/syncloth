import bpy
import numpy as np
from scipy.spatial.transform import Rotation

from syncloth.curves.bezier import quadratic_bezier
from syncloth.geometry import top_down_orientation
from syncloth.robot_arms import add_robotiq
from syncloth.trajectories import (
    combine_orientation_and_position_trajectory,
    concatenate_trajectories,
    linear_trajectory,
    minimum_jerk_trajectory,
    slerp_orientation_trajectory,
)
from syncloth.visualization.frame import add_frame
from syncloth.visualization.points import add_points_as_instances
from syncloth.visualization.trajectories import (
    animate_object_along_trajectory,
    visualize_position_trajectory_as_dots,
    visualize_position_trajectory_as_growing_curve,
)

bpy.ops.object.delete()  # Delete default cube

# Defining the key locations of the fold
grasp_location = np.array([0.0, 0.0, 0.0])
fold_center = np.array([0.35, 0.0, 0.0])
fold_axis = np.array([0.0, 1.0, 0.0])


pregrasp_location = grasp_location + np.array([-0.05, 0.0, 0.0])
hover_location = pregrasp_location + np.array([0.0, 0.0, 0.05])
fold_end = grasp_location + 2 * (fold_center - grasp_location)
fold_height = 0.35
fold_middle = fold_center + np.array([0.0, 0.0, fold_height])
retreat_location = fold_end + np.array([0.0, 0.0, 0.05])


# Building up the trajectory
descend_trajectory, descend_duration = linear_trajectory(hover_location, pregrasp_location, speed=0.1)
approach_trajectory, approach_duration = linear_trajectory(pregrasp_location, grasp_location, speed=0.1)

# Curved Trajectories
# Curve 1: Quadratic Bezier
middle_control_point = fold_center + np.array([0.0, 0.0, 2 * fold_height])
fold_arc_path = quadratic_bezier(grasp_location, middle_control_point, fold_end)  # domain [0, 1]
fold_arc_trajectory, fold_arc_duration = minimum_jerk_trajectory(fold_arc_path, peak_speed=0.5)

# Curve 2: (Squashed) Circular Arc
# circular_arc_path = circular_arc(grasp_location, fold_center, fold_axis, np.pi)  # domain [0, 1]
# vertical_squash = 0.5
# def fold_arc_path(t):
#     x, y, z = circular_arc_path(t)
#     return np.array([x, y, z * vertical_squash])

# fold_arc_trajectory, fold_arc_duration = minimum_jerk_trajectory(fold_arc_path, peak_speed=0.5)

# Triangular Trajectory
# first_half_trajectory, first_half_duration = linear_trajectory(grasp_location, fold_middle, speed=0.5)
# second_half_trajectory, second_half_duration = linear_trajectory(fold_middle, fold_end, speed=0.5)
# fold_arc_trajectory, fold_arc_duration = concatenate_trajectories(
#     [first_half_trajectory, second_half_trajectory], [first_half_duration, second_half_duration]
# )


retreat_trajectory, retreat_duration = linear_trajectory(fold_end, retreat_location, speed=0.1)

position_trajectories = [
    descend_trajectory,
    approach_trajectory,
    fold_arc_trajectory,
    retreat_trajectory,
]

position_durations = [
    descend_duration,
    approach_duration,
    fold_arc_duration,
    retreat_duration,
]


fold_position_trajectory, total_duration = concatenate_trajectories(position_trajectories, position_durations)

# Defining the key orientations of the fold
# Middle orienation
fold_middle_orientation = top_down_orientation(np.array([1, 0, 0]))
middle_location = fold_arc_trajectory(0.5 * fold_arc_duration)

local_Y = fold_middle_orientation[:, 1]

# Start orienation
tilt_angle = np.deg2rad(45)
rotation_local_Y = Rotation.from_rotvec(tilt_angle * local_Y).as_matrix()
grasp_orientation = rotation_local_Y @ fold_middle_orientation

# Hover orientation
hover_orientation = grasp_orientation.copy()

# End orientation
rotation_local_Y_negative = Rotation.from_rotvec(-tilt_angle * local_Y).as_matrix()
fold_end_orientation = rotation_local_Y_negative @ fold_middle_orientation

# Retreat orientation
retreat_orientation = fold_middle_orientation.copy()


orientations = [
    hover_orientation,
    grasp_orientation,
    fold_middle_orientation,
    fold_end_orientation,
    retreat_orientation,
]

orientation_durations = [
    descend_duration + approach_duration,
    0.5 * fold_arc_duration,
    0.5 * fold_arc_duration,
    retreat_duration,
]

times = [0.0]
for duration in orientation_durations:
    times.append(times[-1] + duration)

# fold_orientation_trajectory = Slerp(times, orientations)
fold_orientation_trajectory = slerp_orientation_trajectory(times, orientations)
fold_trajectory = combine_orientation_and_position_trajectory(fold_orientation_trajectory, fold_position_trajectory)


# Visualization
key_locations = [
    hover_location,
    pregrasp_location,
    grasp_location,
    fold_center,
    fold_middle,
    fold_end,
    retreat_location,
]
add_points_as_instances(key_locations, radius=0.0075, color=(0, 0, 1))


frames_per_second = 30
frame_interval = 1.0 / frames_per_second

num_frames = int(np.ceil(total_duration / frame_interval))

for time in times:
    add_frame(fold_trajectory(time), size=0.05)

visualize_position_trajectory_as_dots(fold_position_trajectory, total_duration)
visualize_position_trajectory_as_growing_curve(
    fold_position_trajectory, total_duration, num_frames, color=(0.8, 0.5, 0.2)
)

# Animating a gripper that follows the trajectory
bpy.context.scene.frame_end = num_frames
bpy.context.scene.render.fps = frames_per_second


def gripper_base_trajectory(t):
    tcp_offset = np.array([0.0, 0.0, 0.172])
    tcp_transform = np.identity(4)
    tcp_transform[:3, 3] = tcp_offset
    tcp_inverse_transform = np.linalg.inv(tcp_transform)
    return fold_trajectory(t) @ tcp_inverse_transform


gripper_base = add_robotiq()
animate_object_along_trajectory(gripper_base, gripper_base_trajectory, total_duration)


# Setting up the camera
camera = bpy.data.objects["Camera"]
camera.location = fold_center + np.array([0.0, -1.0, 0.16])
camera.rotation_euler = np.array([np.deg2rad(90.0), 0.0, 0.0])
camera.data.type = "ORTHO"
camera.data.ortho_scale = 1.2
