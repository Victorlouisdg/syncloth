from typing import List

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from syncloth.curves.arc_length import arc_length_parametrize
from syncloth.curves.linear import linear_interpolation


def minimum_jerk(t: float):
    """Minimum jerk function for t in [0, 1]"""
    return 10 * (t**3) - 15 * (t**4) + 6 * (t**5)


def minimum_jerk_trajectory(path, peak_speed: float = 0.5):
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


def linear_trajectory(start, end, speed: float = 0.1):
    linear_path = linear_interpolation(start, end)  # domain [0, 1]
    length = np.linalg.norm(start - end)
    duration = length / speed

    def trajectory(t):
        return linear_path(t / duration)  # domain [0, duration]

    return trajectory, duration


def slerp_orientation_trajectory(times: List[float], orientations: List[np.ndarray]):
    orientations_scipy = Rotation.from_matrix(np.array(orientations))
    slerp = Slerp(times, orientations_scipy)

    def orientation_trajectory(t):
        return slerp(t).as_matrix()

    return orientation_trajectory


def combine_orientation_and_position_trajectory(orientation_trajectory, position_trajectory):
    def pose_trajectory(t):
        pose = np.identity(4)
        pose[:3, :3] = orientation_trajectory(t)
        pose[:3, 3] = position_trajectory(t)
        return pose

    return pose_trajectory


def concatenate_trajectories(trajectories, durations):
    def trajectory(t):
        for trajectory, duration in zip(trajectories, durations):
            if t <= duration:
                return trajectory(t)
            t -= duration  # go to next trajectory
        return trajectory(duration)  # if we get here, return last point

    return trajectory, sum(durations)
