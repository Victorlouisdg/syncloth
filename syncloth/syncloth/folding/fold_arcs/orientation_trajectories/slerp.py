from syncloth.paths.orientation import slerp_trajectory


def fold_arc_slerp_trajectory(position_trajectory, orientations):
    # start_to_end = position_trajectory.end_value - position_trajectory.start_value
    # start_to_end[2] = 0  # Project onto the x-y plane
    # start_to_end_direction = start_to_end / np.linalg.norm(start_to_end)

    # rotation_quarter = Rotation.from_rotvec(np.pi / 2 * fold_line_direction).as_matrix()
    # rotation_half = Rotation.from_rotvec(np.pi * fold_line_direction).as_matrix()
    # grasp_flat = flat_orientation(approach_direction)
    # middle_orientation = rotation_quarter @ grasp_flat
    # end_orientation = rotation_half @ grasp_flat

    # orientations = [start_orientation, middle_orientation, end_orientation]

    duration = position_trajectory.duration
    times = [0, 0.5 * duration, duration]

    return slerp_trajectory(times, orientations)
