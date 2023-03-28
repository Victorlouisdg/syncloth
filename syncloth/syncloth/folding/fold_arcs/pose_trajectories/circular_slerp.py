from syncloth.folding.fold_arcs.orientation_trajectories.slerp import fold_arc_slerp_trajectory
from syncloth.folding.fold_arcs.position_trajectories.circular_arc import fold_arc_circular_trajectory
from syncloth.paths.orientation import combine_orientation_and_position_trajectory
from syncloth.paths.path import Path


def fold_arc_circular_slerp_trajectory(
    fold_line,
    grasp_location,
    orientations,
    fold_end_height,
) -> Path:
    fold_arc_position_trajectory = fold_arc_circular_trajectory(fold_line, grasp_location, fold_end_height)
    fold_arc_orienation_trajectory = fold_arc_slerp_trajectory(fold_arc_position_trajectory, orientations)

    fold_arc_trajectory = combine_orientation_and_position_trajectory(
        fold_arc_orienation_trajectory, fold_arc_position_trajectory
    )
    return fold_arc_trajectory
