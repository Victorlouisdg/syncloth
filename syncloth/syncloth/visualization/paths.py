import airo_blender as ab
import bpy
import numpy as np
from mathutils import Matrix

from syncloth.paths.path import Path
from syncloth.visualization.curves import add_curve_mesh, skin
from syncloth.visualization.points import add_points_as_instances


def add_path_as_points(path: Path, num_points=100) -> bpy.types.Object:
    dots = [path.function(t) for t in np.linspace(path.start, path.end, num_points)]
    add_points_as_instances(dots, radius=0.0025, color=(1, 0, 0))


def animate_object_along_path(object: bpy.types.Object, pose_path: Path) -> None:
    fps = bpy.context.scene.render.fps
    frame_interval = 1 / fps
    num_frames = int(np.ceil(pose_path.duration / frame_interval))

    bpy.context.scene.frame_set(1)

    for i in range(num_frames):
        bpy.context.scene.frame_set(i + 1)
        t = i * frame_interval
        pose = pose_path.function(pose_path.start + t)
        object.matrix_world = Matrix(pose)
        bpy.context.view_layer.update()
        object.keyframe_insert(data_path="location")
        object.keyframe_insert(data_path="rotation_euler")

    bpy.context.scene.frame_set(1)


def add_path_as_growing_curve(path, radius=0.0025, color=(1, 0, 0)):
    fps = bpy.context.scene.render.fps
    frame_interval = 1 / fps
    num_frames = int(np.ceil(path.duration / frame_interval))

    curve_points = [path.function(t) for t in np.linspace(path.start, path.end, num_frames)]
    curve_mesh = add_curve_mesh(curve_points)
    ab.add_material(curve_mesh, color=color)

    build_modifier = curve_mesh.modifiers.new(name="Build", type="BUILD")
    build_modifier.frame_start = 1
    build_modifier.frame_duration = num_frames
    skin(curve_mesh, radius=radius)
