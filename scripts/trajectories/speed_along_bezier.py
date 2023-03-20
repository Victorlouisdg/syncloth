import airo_blender as ab
import bpy
import numpy as np

from syncloth.curves.arc_length import arc_length_parametrize, integrate_arc_length
from syncloth.curves.bezier import quadratic_bezier
from syncloth.visualization.points import add_points_as_instances

position0 = np.array([0.0, 0.0, 0.0])
position1 = np.array([0.5, 0.0, 2.0])
position2 = np.array([1.0, 0.0, 0.0])

# Plot the path in Blender
bpy.ops.object.delete()  # Delete default cube

n_samples = 40

# Parametrization:
# 1) bezier parametrization (0 -> 1) (or more generally any a -> b, e.g. 0 -> 2pi for a circle)
# 2) arc length parametrization (0 -> total arc length)
# 3) time parametrization (0 -> t)

bezier = quadratic_bezier(position0, position1, position2)
samples0 = [bezier(s) for s in np.linspace(0, 1, n_samples)]
arc_length0 = integrate_arc_length(bezier)
print("arc_length0 =", arc_length0)

# arc_length1 = integrate_arc_length(bezier_reparametrized, end=arc_length0, num=1000)
bezier_reparametrized, arc_length1 = arc_length_parametrize(bezier, 0.0, 1.0)
print("arc_length1 =", arc_length1)


def bezier_shifted(s):
    return bezier_reparametrized(s) + np.array([1.05, 0, 0])


samples1_shifted = [bezier_shifted(s) for s in np.linspace(0, arc_length1, n_samples)]


def minimum_jerk(t):
    return 10 * (t**3) - 15 * (t**4) + 6 * (t**5)


def bezier_retimed(s):
    return bezier_reparametrized(arc_length1 * minimum_jerk(s / arc_length1))


def bezier_retimed_shifted(s):
    return bezier_retimed(s) + np.array([2.1, 0, 0])


samples2_shifted = [bezier_retimed_shifted(s) for s in np.linspace(0, arc_length1, n_samples)]


# Plot the path in Blender
bpy.ops.object.delete()  # Delete default cube


add_points_as_instances(samples0, radius=0.02, color=(1, 0, 0))
add_points_as_instances(samples1_shifted, radius=0.02, color=(0, 1, 0))
add_points_as_instances(samples2_shifted, radius=0.02, color=(0, 0, 1))


# Animated a sphere to follow the path
def animate_sphere_along_path(curve, start=0.0, end=1.0):
    bpy.ops.mesh.primitive_uv_sphere_add(location=position0, scale=(0.05, 0.05, 0.05))
    sphere = bpy.context.object

    total_time = 2.0  # seconds
    fps = bpy.context.scene.render.fps
    frames = int(total_time * fps)

    bpy.context.scene.frame_end = frames

    s_range = np.linspace(start, end, frames)

    for i, s in enumerate(s_range):
        location = curve(s)
        bpy.context.scene.frame_set(i + 1)
        sphere.location = location
        sphere.keyframe_insert(data_path="location")

    return sphere


sphere0 = animate_sphere_along_path(bezier)
sphere1 = animate_sphere_along_path(bezier_shifted, end=arc_length1)
sphere2 = animate_sphere_along_path(bezier_retimed_shifted, end=arc_length1)
bpy.context.scene.frame_set(1)

# add material to sphere
ab.add_material(sphere0, color=(1, 0, 0))
ab.add_material(sphere1, color=(0, 1, 0))
ab.add_material(sphere2, color=(0, 0, 1))

camera = bpy.data.objects["Camera"]
camera.data.type = "ORTHO"
camera.location = (1.5, -2, 0.5)
camera.rotation_euler = (np.deg2rad(90), 0, 0)
camera.data.ortho_scale = 3.5

scene = bpy.context.scene
scene.render.resolution_x = 3000
scene.render.resolution_y = 1000

scene.render.engine = "CYCLES"
