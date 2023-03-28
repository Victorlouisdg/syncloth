import numpy as np
from scipy.spatial.transform import Rotation

from syncloth.geometry import top_down_orientation
from syncloth.paths.orientation import slerp_trajectory


def fold_arc_slerp_trajectory(position_trajectory, start_orientation, fold_line_direction, approach_angle):
    start_to_end = position_trajectory.end_value - position_trajectory.start_value
    start_to_end[2] = 0  # Project onto the x-y plane
    start_to_end_direction = start_to_end / np.linalg.norm(start_to_end)

    middle_orientation = top_down_orientation(start_to_end_direction)
    # end_orientation = top_down_orientation(start_to_end_direction)

    angle = np.pi + 2 * approach_angle
    rotation_matrix_pi = Rotation.from_rotvec(angle * fold_line_direction).as_matrix()

    end_orientation = rotation_matrix_pi @ start_orientation

    orientations = [start_orientation, middle_orientation, end_orientation]

    duration = position_trajectory.duration
    times = [0, 0.5 * duration, duration]

    return slerp_trajectory(times, orientations)
