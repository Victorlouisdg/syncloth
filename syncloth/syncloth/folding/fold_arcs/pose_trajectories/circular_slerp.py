from syncloth.folding.fold_arcs.orientation_trajectories.slerp import fold_arc_slerp_trajectory
from syncloth.folding.fold_arcs.position_trajectories.circular_arc import fold_arc_circular_trajectory
from syncloth.paths.orientation import combine_orientation_and_position_trajectory
from syncloth.paths.path import Path


def fold_arc_circular_slerp_trajectory(
    fold_line, grasp_location, grasp_orientation, fold_end_height, approach_angle
) -> Path:
    fold_arc_position_trajectory = fold_arc_circular_trajectory(fold_line, grasp_location, fold_end_height)

    fold_line_direction = fold_line[1]
    fold_arc_orienation_trajectory = fold_arc_slerp_trajectory(
        fold_arc_position_trajectory, grasp_orientation, fold_line_direction, approach_angle
    )

    fold_arc_trajectory = combine_orientation_and_position_trajectory(
        fold_arc_orienation_trajectory, fold_arc_position_trajectory
    )
    return fold_arc_trajectory
