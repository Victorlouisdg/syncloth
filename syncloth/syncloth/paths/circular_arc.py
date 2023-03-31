import numpy as np
from scipy.spatial.transform import Rotation

from syncloth.paths.path import Path


def circular_arc_position_path(start, center, axis, max_angle: float) -> Path:
    def circular_arc_function(angle):
        rotation = Rotation.from_rotvec(angle * axis)
        return center + rotation.apply(start - center)

    return Path(circular_arc_function, start_time=0.0, end_time=max_angle)


def circular_arc_orientation_path(start_orientation, axis, max_angle: float) -> Path:
    def circular_arc_function(angle):
        rotation = Rotation.from_rotvec(angle * axis)
        return rotation.as_matrix() @ start_orientation

    return Path(circular_arc_function, start_time=0.0, end_time=max_angle)


def circular_arc_pose_path(start_pose, center, axis, max_angle: float) -> Path:
    center_pose = np.identity(4)
    center_pose[:3, 3] = center

    def circular_arc_function(angle):
        rotation = Rotation.from_rotvec(angle * axis)
        homogeneous_rotation = np.identity(4)
        homogeneous_rotation[:3, :3] = rotation.as_matrix()
        return center_pose @ homogeneous_rotation @ np.linalg.inv(center_pose) @ start_pose

    return Path(circular_arc_function, start_time=0.0, end_time=max_angle)
