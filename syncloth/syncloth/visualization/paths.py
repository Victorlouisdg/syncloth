import airo_blender as ab
import bpy
import numpy as np
from mathutils import Matrix

from syncloth.paths.path import Path
from syncloth.visualization.curves import add_curve_mesh, skin
from syncloth.visualization.frame import add_frame
from syncloth.visualization.points import add_points_as_instances


def add_path_as_points(path: Path, num_points=100, color=(1, 0, 0)) -> bpy.types.Object:
    dots = [path(t) for t in np.linspace(path.start_time, path.end_time, num_points)]
    add_points_as_instances(dots, radius=0.0025, color=color)


def add_pose_path_as_points(path: Path, num_points=100, color=(1, 0, 0)) -> bpy.types.Object:
    dots = [path(t)[:3, 3] for t in np.linspace(path.start_time, path.end_time, num_points)]
    add_points_as_instances(dots, radius=0.0025, color=color)


def animate_object_along_path(object: bpy.types.Object, pose_path: Path) -> None:
    fps = bpy.context.scene.render.fps
    frame_interval = 1 / fps
    num_frames = int(np.ceil(pose_path.duration / frame_interval))

    bpy.context.scene.frame_set(1)

    for i in range(num_frames):
        bpy.context.scene.frame_set(i + 1)
        t = i * frame_interval
        pose = pose_path(pose_path.start_time + t)
        object.matrix_world = Matrix(pose)
        bpy.context.view_layer.update()
        object.keyframe_insert(data_path="location")
        object.keyframe_insert(data_path="rotation_euler")

    bpy.context.scene.frame_set(1)


def add_path_as_growing_curve(path, radius=0.0025, color=(1, 0, 0)):
    fps = bpy.context.scene.render.fps
    frame_interval = 1 / fps
    num_frames = int(np.ceil(path.duration / frame_interval))

    curve_points = [path(t) for t in np.linspace(path.start_time, path.end_time, num_frames)]
    curve_mesh = add_curve_mesh(curve_points)
    ab.add_material(curve_mesh, color=color)

    build_modifier = curve_mesh.modifiers.new(name="Build", type="BUILD")
    build_modifier.frame_start = 1
    build_modifier.frame_duration = num_frames
    skin(curve_mesh, radius=radius)


def visualize_trajectory(trajectory: Path, points_per_second: float = 20.0, color=(1, 0, 0)):
    add_pose_path_as_points(trajectory, num_points=int(points_per_second * trajectory.duration), color=color)
    add_frame(trajectory.start, size=0.05)
    add_frame(trajectory.end, size=0.05)


def visualize_trajectory_frames(trajectory: Path, num_frames: int = 100):
    for t in np.linspace(0.0, trajectory.duration, num_frames):
        add_frame(trajectory(t), size=0.05)
