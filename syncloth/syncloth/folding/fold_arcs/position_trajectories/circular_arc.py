from typing import Tuple

import numpy as np
from airo_typing import Vector3DType

from syncloth.paths.circular_arc import circular_arc_path
from syncloth.paths.path import Path
from syncloth.paths.retiming import minimum_jerk_trajectory


def fold_arc_circular_trajectory(
    fold_line: Tuple[Vector3DType, Vector3DType], grasp_location: Vector3DType, end_height_offset: float
) -> Path:
    fold_line_point, fold_line_direction = fold_line

    path = circular_arc_path(grasp_location, *fold_line, np.deg2rad(180))
    trajectory = minimum_jerk_trajectory(path)
    return trajectory

    # fold_plane_normal = np.cross(fold_line_direction, [0, 0, 1])
    # fold_plane = (fold_line_point, fold_plane_normal)

    # end_point = reflect_point_over_plane(grasp_location, fold_plane)
    # distance_start_end = np.linalg.norm(end_point - grasp_location)
    # fold_center = start_point + (end_point - start_point) / 2

    # # Note that the peak of the will be half als high (property of the quadratic bezier curve)
    # height_offset = np.array([0, 0, distance_start_end])
    # middle_control_point = fold_center + height_offset

    # end_point_raised = end_point + np.array([0, 0, end_height_offset])

    # # path = quadratic_bezier_path(start_point, middle_control_point, end_point_raised)
    # trajectory = minimum_jerk_trajectory(path)
    # return trajectory
