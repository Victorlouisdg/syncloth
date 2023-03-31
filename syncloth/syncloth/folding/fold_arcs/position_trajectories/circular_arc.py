from typing import Tuple

import numpy as np
from airo_typing import Vector3DType

from syncloth.geometry import project_point_on_line
from syncloth.paths.circular_arc import circular_arc_position_path
from syncloth.paths.path import Path


def fold_arc_circular_trajectory(
    fold_line: Tuple[Vector3DType, Vector3DType], grasp_location: Vector3DType, end_height_offset: float
) -> Path:

    grasp_projected = project_point_on_line(grasp_location, fold_line)
    radius = np.linalg.norm(grasp_projected - grasp_location)
    angle_delta = np.arcsin(end_height_offset / radius)
    angle = np.pi - angle_delta
    path = circular_arc_position_path(grasp_location, *fold_line, angle)

    # Arc length parametrize the circular path
    trajectory = Path(
        lambda t: path(t / radius),
        0,
        radius * angle,
    )
    return trajectory
