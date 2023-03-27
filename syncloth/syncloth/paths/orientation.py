from typing import List

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from syncloth.paths.constant import constant_trajectory
from syncloth.paths.linear import linear_trajectory
from syncloth.paths.path import Path


def slerp_trajectory(times: List[float], orientations: List[np.ndarray]) -> Path:
    orientations_scipy = Rotation.from_matrix(np.array(orientations))
    slerp = Slerp(times, orientations_scipy)
    return Path(lambda t: slerp(t).as_matrix(), times[0], times[-1])


def combine_orientation_and_position_trajectory(orientation_trajectory: Path, position_trajectory: Path) -> Path:
    def pose_trajectory(t):
        pose = np.identity(4)
        pose[:3, :3] = orientation_trajectory.function(t)
        pose[:3, 3] = position_trajectory.function(t)
        return pose

    # TODO maybe enforce/assert that the trajectories are start and end together?
    # Combining two incorrect trajectories has resulted in a bug for me.
    start = min(orientation_trajectory.start, position_trajectory.start)
    end = max(orientation_trajectory.end, position_trajectory.end)

    return Path(pose_trajectory, start, end)


def linear_position_constant_orientation_trajectory(
    start_position: np.ndarray, end_position: np.ndarray, orientation: np.ndarray, speed: float
) -> Path:
    position_trajectory = linear_trajectory(start_position, end_position, speed)
    orientation_trajectory = constant_trajectory(orientation, duration=position_trajectory.duration)
    return combine_orientation_and_position_trajectory(orientation_trajectory, position_trajectory)
