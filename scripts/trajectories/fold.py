import airo_blender as ab
import bpy
import numpy as np

from syncloth.curves.arc_length import arc_length_parametrize
from syncloth.curves.bezier import quadratic_bezier
from syncloth.curves.linear import linear_interpolation
from syncloth.visualization.curves import add_curve_mesh, skin
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

trajectories = [
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


fold_trajectory, total_duration = concatenate_trajectories(trajectories, durations)


frames_per_second = 30
frame_interval = 1.0 / frames_per_second

total_duration = sum(durations)
num_frames = int(np.ceil(total_duration / frame_interval))

# Visualizing the trajectory
dots = [fold_trajectory(t) for t in np.linspace(0, total_duration, 50)]
add_points_as_instances(dots, radius=0.005, color=(1, 0, 0))

curve_points = [fold_trajectory(t) for t in np.linspace(0, total_duration, 200)]
curve_mesh = add_curve_mesh(curve_points)
build_modifier = curve_mesh.modifiers.new(name="Build", type="BUILD")
build_modifier.frame_start = 1
build_modifier.frame_duration = num_frames
skin(curve_mesh, radius=0.0025)


bpy.context.scene.frame_end = num_frames
bpy.context.scene.render.fps = frames_per_second

bpy.ops.mesh.primitive_uv_sphere_add(scale=(0.01, 0.01, 0.01))
sphere = bpy.context.object
ab.add_material(sphere, color=(1, 1, 0))

for i in range(num_frames):
    bpy.context.scene.frame_set(i + 1)
    t = i * frame_interval
    location = fold_trajectory(t)
    sphere.location = location
    sphere.keyframe_insert(data_path="location")

# Setting up the camera
camera = bpy.data.objects["Camera"]
camera.location = fold_center + np.array([0.0, -1.0, 0.2])
camera.rotation_euler = np.array([np.deg2rad(90.0), 0.0, 0.0])
camera.data.type = "ORTHO"
camera.data.ortho_scale = 1.0
