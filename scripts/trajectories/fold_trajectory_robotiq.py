import bpy
import numpy as np
from scipy.spatial.transform import Rotation

from syncloth.geometry import top_down_orientation
from syncloth.paths.bezier import quadratic_bezier_path
from syncloth.paths.concatenate import concatenate_trajectories
from syncloth.paths.linear import linear_trajectory
from syncloth.paths.orientation import combine_orientation_and_position_trajectory, slerp_trajectory
from syncloth.paths.path import Path
from syncloth.paths.retiming import minimum_jerk_trajectory
from syncloth.robot_arms import add_robotiq
from syncloth.visualization.frame import add_frame
from syncloth.visualization.paths import add_path_as_growing_curve, add_path_as_points, animate_object_along_path

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

descent_trajectory = linear_trajectory(hover_location, pregrasp_location, speed=0.1)
approach_trajectory = linear_trajectory(pregrasp_location, grasp_location, speed=0.1)

middle_control_point = fold_center + np.array([0.0, 0.0, 2 * fold_height])
fold_arc_path = quadratic_bezier_path(grasp_location, middle_control_point, fold_end)
fold_arc_trajectory = minimum_jerk_trajectory(fold_arc_path)

retreat_trajectory = linear_trajectory(fold_end, retreat_location, speed=0.1)

fold_position_trajectory = concatenate_trajectories(
    [descent_trajectory, approach_trajectory, fold_arc_trajectory, retreat_trajectory]
)


# Orientations
# Defining the key orientations of the fold
# Middle orienation
fold_middle_orientation = top_down_orientation(np.array([1, 0, 0]))
middle_location = fold_arc_trajectory(0.5 * fold_arc_trajectory.duration)

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
    descent_trajectory.duration + approach_trajectory.duration,
    0.5 * fold_arc_trajectory.duration,
    0.5 * fold_arc_trajectory.duration,
    retreat_trajectory.duration,
]


times = [0.0]
for duration in orientation_durations:
    times.append(times[-1] + duration)


orientation_trajectory = slerp_trajectory(times, orientations)
fold_trajectory = combine_orientation_and_position_trajectory(orientation_trajectory, fold_position_trajectory)

# Visualization below
# Animating a gripper that follows the trajectory
frames_per_second = 30
frame_interval = 1.0 / frames_per_second
num_frames = int(np.ceil(fold_trajectory.duration / frame_interval))
bpy.context.scene.frame_end = num_frames
bpy.context.scene.render.fps = frames_per_second

add_path_as_points(fold_position_trajectory, num_points=100)
add_path_as_growing_curve(fold_position_trajectory, color=(0.8, 0.5, 0.2))

for time in times:
    add_frame(fold_trajectory(time), size=0.05)


gripper_base = add_robotiq()

tcp_trajectory = fold_trajectory


def gripper_base_trajectory_function(t):
    tcp_offset = np.array([0.0, 0.0, 0.172])
    tcp_transform = np.identity(4)
    tcp_transform[:3, 3] = tcp_offset
    tcp_inverse_transform = np.linalg.inv(tcp_transform)
    return tcp_trajectory(t) @ tcp_inverse_transform


gripper_base_trajectory = Path(
    gripper_base_trajectory_function,
    start_time=tcp_trajectory.start_time,
    end_time=tcp_trajectory.end_time,
)

animate_object_along_path(gripper_base, gripper_base_trajectory)

# Setting up the camera
camera = bpy.data.objects["Camera"]
camera.location = fold_center + np.array([0.0, -1.0, 0.22])
camera.rotation_euler = np.array([np.deg2rad(90.0), 0.0, 0.0])
camera.data.type = "ORTHO"
camera.data.ortho_scale = 1.2
