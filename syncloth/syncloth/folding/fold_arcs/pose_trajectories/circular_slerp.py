import numpy as np

from syncloth.folding.fold_arcs.position_trajectories.circular_arc import fold_arc_circular_trajectory
from syncloth.paths.orientation import combine_orientation_and_position_trajectory, slerp_trajectory
from syncloth.paths.path import Path
from syncloth.paths.retiming import minimum_jerk_trajectory


def fold_arc_circular_slerp_trajectory(
    fold_line,
    grasp_location,
    orientations,
    fold_end_height,
    peak_speed=0.5,
) -> Path:
    position_trajectory = fold_arc_circular_trajectory(fold_line, grasp_location, fold_end_height)

    duration = position_trajectory.duration
    times = np.linspace(0, duration, len(orientations))
    orientation_trajectory = slerp_trajectory(times, orientations)

    trajectory = combine_orientation_and_position_trajectory(orientation_trajectory, position_trajectory)
    trajectory_smooth = minimum_jerk_trajectory(trajectory, peak_speed=peak_speed)
    return trajectory_smooth
