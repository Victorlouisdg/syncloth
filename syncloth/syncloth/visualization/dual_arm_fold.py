import airo_blender as ab

from syncloth.paths.concatenate import concatenate_trajectories
from syncloth.robot_arms import add_animated_robotiq
from syncloth.visualization.curves import add_curve_mesh, skin
from syncloth.visualization.frame import add_frame
from syncloth.visualization.paths import add_pose_path_as_points
from syncloth.visualization.points import add_points_as_instances


def visualize_dual_arm_fold(keypoints, fold_line, trajectories_left, trajectories_right):
    blender_red = [0.930111, 0.036889, 0.084376, 1.000000]
    # blender_green = [0.205079, 0.527115, 0.006049, 1.000000]
    # blender_blue = [0.028426, 0.226966, 0.760525, 1.000000]

    trajectories = trajectories_left + trajectories_right
    for trajectory in trajectories:
        add_pose_path_as_points(trajectory, num_points=int(10 * trajectory.duration), color=blender_red)
        add_frame(trajectory.start)
        add_frame(trajectory(0.5 * trajectory.duration), size=0.05)
        add_frame(trajectory.end)

    # Keypoints
    yellow = [1.0, 1.0, 0.0]
    add_points_as_instances(keypoints, color=yellow)

    # Fold line
    fold_line_point, fold_line_direction = fold_line
    scale = 0.5
    fold_line_points = [
        fold_line_point - scale * fold_line_direction,
        fold_line_point + scale * fold_line_direction,
    ]
    fold_line_mesh = add_curve_mesh(fold_line_points)
    ab.add_material(fold_line_mesh, yellow)
    skin(fold_line_mesh)

    # Robotiqs
    concatenated_trajectory_left = concatenate_trajectories(trajectories_left)
    concatenated_trajectory_right = concatenate_trajectories(trajectories_right)

    print(concatenated_trajectory_left.duration)
    add_animated_robotiq(concatenated_trajectory_left)
    add_animated_robotiq(concatenated_trajectory_right)
