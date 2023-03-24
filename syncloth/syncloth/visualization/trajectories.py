import airo_blender as ab
import bpy
import numpy as np
from mathutils import Matrix

from syncloth.visualization.curves import add_curve_mesh, skin
from syncloth.visualization.points import add_points_as_instances


def visualize_position_trajectory_as_dots(position_trajectory, duration, num_dots=100, radius=0.0025, color=(1, 0, 0)):
    dots = [position_trajectory(t) for t in np.linspace(0, duration, num_dots)]
    add_points_as_instances(dots, radius=0.0025, color=(1, 0, 0))


def visualize_position_trajectory_as_growing_curve(
    position_trajectory, duration, num_frames=100, radius=0.0025, color=(1, 0, 0)
):
    curve_points = [position_trajectory(t) for t in np.linspace(0, duration, num_frames)]
    curve_mesh = add_curve_mesh(curve_points)
    ab.add_material(curve_mesh, color=color)

    build_modifier = curve_mesh.modifiers.new(name="Build", type="BUILD")
    build_modifier.frame_start = 1
    build_modifier.frame_duration = num_frames
    skin(curve_mesh, radius=radius)


def animate_object_along_trajectory(object, pose_trajectory, total_duration):
    fps = bpy.context.scene.render.fps
    frame_interval = 1 / fps
    num_frames = int(np.ceil(total_duration / frame_interval))

    bpy.context.scene.frame_set(1)

    for i in range(num_frames):
        bpy.context.scene.frame_set(i + 1)
        t = i * frame_interval
        pose = pose_trajectory(t)
        object.matrix_world = Matrix(pose)
        bpy.context.view_layer.update()
        object.keyframe_insert(data_path="location")
        object.keyframe_insert(data_path="rotation_euler")

    bpy.context.scene.frame_set(1)
