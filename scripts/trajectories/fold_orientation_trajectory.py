import airo_blender as ab
import bpy
import numpy as np
from mathutils import Matrix
from scipy.spatial.transform import Rotation, Slerp

from syncloth.curves.arc_length import arc_length_parametrize
from syncloth.curves.bezier import quadratic_bezier
from syncloth.curves.linear import linear_interpolation
from syncloth.visualization.curves import add_curve_mesh, skin
from syncloth.visualization.frame import add_frame
from syncloth.visualization.points import add_points_as_instances


def minimum_jerk(t: float):
    """Minimum jerk function for t in [0, 1]"""
    return 10 * (t**3) - 15 * (t**4) + 6 * (t**5)


def linear_trajectory(start, end, speed: float = 0.1):
    linear_path = linear_interpolation(start, end)  # domain [0, 1]
    length = np.linalg.norm(start - end)
    duration = length / speed

    def trajectory(t):
        return linear_path(t / duration)  # domain [0, duration]

    return trajectory, duration


def create_fold_arch_trajectory(grasp_location, fold_middle, fold_end, peak_speed: float = 0.5):
    path = quadratic_bezier(grasp_location, fold_middle, fold_end)  # domain [0, 1]
    path_arc_length_parametrized, arc_length = arc_length_parametrize(path, 0.0, 1.0)  # domain [0, arc_length]

    # An arc length parametrized curve has constant speed of 1 m/s
    def path_minimum_jerk(t):
        return path_arc_length_parametrized(minimum_jerk(t / arc_length) * arc_length)  # domain [0, arc_length]

    # Minimum jerk has max speed of 1.875 m/s at t=0.5 (TODO verify the math for this)
    # We want to scale that to peak_speed e.g. 0.5 m/s
    # We do this by scaling the domain of the function by the fraction 1.875 / peak_speed
    scaling_factor = 1.875 / peak_speed
    duration = arc_length * scaling_factor

    def trajectory(t):
        return path_minimum_jerk(t / scaling_factor)

    return trajectory, duration


bpy.ops.object.delete()  # Delete default cube

# Defining the key locations of the fold
grasp_location = np.array([0.0, 0.0, 0.0])
pregrasp_location = grasp_location + np.array([-0.05, 0.0, 0.0])
hover_location = pregrasp_location + np.array([0.0, 0.0, 0.05])
fold_center = np.array([0.35, 0.0, 0.0])
fold_end = grasp_location + 2 * (fold_center - grasp_location)
fold_middle = fold_center + np.array([0.0, 0.0, 0.4])
retreat_location = fold_end + np.array([0.0, 0.0, 0.05])

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


# Building up the trajectory
descend_trajectory, descend_duration = linear_trajectory(hover_location, pregrasp_location, speed=0.1)
approach_trajectory, approach_duration = linear_trajectory(pregrasp_location, grasp_location, speed=0.1)
fold_arch_trajectory, fold_arch_duration = create_fold_arch_trajectory(grasp_location, fold_middle, fold_end)
retreat_trajectory, retreat_duration = linear_trajectory(fold_end, retreat_location, speed=0.1)

position_trajectories = [
    descend_trajectory,
    approach_trajectory,
    fold_arch_trajectory,
    retreat_trajectory,
]

durations = [
    descend_duration,
    approach_duration,
    fold_arch_duration,
    retreat_duration,
]


def concatenate_trajectories(trajectories, durations):
    def trajectory(t):
        for trajectory, duration in zip(trajectories, durations):
            if t <= duration:
                return trajectory(t)
            t -= duration  # go to next trajectory
        return trajectory(duration)  # if we get here, return last point

    return trajectory, sum(durations)


fold_position_trajectory, total_duration = concatenate_trajectories(position_trajectories, durations)


def pose_matrix(orientation, position):
    pose = np.identity(4)
    pose[:3, :3] = orientation
    pose[:3, 3] = position
    return pose


# Defining the key orientations of the fold
def top_down_orientation(gripper_open_direction) -> np.ndarray:
    X = gripper_open_direction / np.linalg.norm(gripper_open_direction)  # np.array([-1, 0, 0])
    Z = np.array([0, 0, -1])
    Y = np.cross(Z, X)
    return np.column_stack([X, Y, Z])


# Middle orienation
fold_middle_orientation = top_down_orientation(np.array([1, 0, 0]))
middle_location = fold_arch_trajectory(0.5 * fold_arch_duration)

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


orientations = np.array(
    [hover_orientation, grasp_orientation, fold_middle_orientation, fold_end_orientation, retreat_orientation]
)
orientations = Rotation.from_matrix(orientations)

durations = [
    descend_duration + approach_duration,
    0.5 * fold_arch_duration,
    0.5 * fold_arch_duration,
    retreat_duration,
]

times = [0.0]
for duration in durations:
    times.append(times[-1] + duration)

fold_orientation_trajectory = Slerp(times, orientations)


def fold_trajectory(t):
    pose = np.identity(4)
    pose[:3, :3] = fold_orientation_trajectory(t).as_matrix()
    pose[:3, 3] = fold_position_trajectory(t)
    return pose


for time in times:
    add_frame(fold_trajectory(time), size=0.05)


frames_per_second = 30
frame_interval = 1.0 / frames_per_second

total_duration = sum(durations)
num_frames = int(np.ceil(total_duration / frame_interval))

# Visualizing the trajectory
dots = [fold_position_trajectory(t) for t in np.linspace(0, total_duration, 100)]
add_points_as_instances(dots, radius=0.0025, color=(1, 0, 0))

curve_points = [fold_position_trajectory(t) for t in np.linspace(0, total_duration, 200)]
curve_mesh = add_curve_mesh(curve_points)
ab.add_material(curve_mesh, color=(0.8, 0.5, 0.2))

build_modifier = curve_mesh.modifiers.new(name="Build", type="BUILD")
build_modifier.frame_start = 1
build_modifier.frame_duration = num_frames
skin(curve_mesh, radius=0.0025)


bpy.context.scene.frame_end = num_frames
bpy.context.scene.render.fps = frames_per_second

# Animating a frame that follows the trajectory
gripper_frame = add_frame(np.identity(4), size=0.1)

for i in range(num_frames):
    bpy.context.scene.frame_set(i + 1)
    t = i * frame_interval
    pose = fold_trajectory(t)
    gripper_frame.matrix_world = Matrix(pose)
    bpy.context.view_layer.update()
    print(pose[:3, 3])
    gripper_frame.keyframe_insert(data_path="location")
    gripper_frame.keyframe_insert(data_path="rotation_euler")

bpy.context.scene.frame_set(1)

# Setting up the camera
camera = bpy.data.objects["Camera"]
camera.location = fold_center + np.array([0.0, -1.0, 0.16])
camera.rotation_euler = np.array([np.deg2rad(90.0), 0.0, 0.0])
camera.data.type = "ORTHO"
camera.data.ortho_scale = 1.0
