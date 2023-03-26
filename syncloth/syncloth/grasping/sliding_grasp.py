import numpy as np
from airo_typing import Vector3DType
from scipy.spatial.transform import Rotation

from syncloth.geometry import flat_orientation
from syncloth.paths.concatenate import concatenate_trajectories
from syncloth.paths.linear import linear_trajectory
from syncloth.paths.path import Path


def sliding_grasp_orientation(approach_direction: Vector3DType, angle: float = np.pi / 6):
    """
    :param approach_direction: The direction along which the gripper will slide towards the grasp location.
    :param angle: The angle in radians that the gripper makes with the surface its sliding on.
    :return: The orientation of the gripper such that it makes an angle with the surface will slide on.
    """
    orientation = flat_orientation(approach_direction)
    local_Y = orientation[:, 1]
    rotation_local_Y = Rotation.from_rotvec(angle * local_Y).as_matrix()
    orientation = rotation_local_Y @ orientation
    return orientation


def sliding_grasp_position_trajectory(
    grasp_location: Vector3DType,
    approach_direction: Vector3DType,
    approach_distance: float = 0.05,
    compliance_depth: float = 0.01,
    hover_height: float = 0.05,
    speed: float = 0.1,
) -> Path:
    """
    :param contact_location: The point where the gripper will first make contact with the object to be grasped.
    :param approach_direction: The direction along which the gripper will slide towards the grasp location.
    :param grasp_depth: How far the gripper will slide further than the grasp location along the approach direction.
    :param angle: The angle in radians that the gripper makes with the surface its sliding on.
    :return: A trajectory of TCP poses.
    """

    approach_direction /= np.linalg.norm(approach_direction)

    pregrasp_location = grasp_location - approach_distance * approach_direction
    hover_location = pregrasp_location + np.array([0.0, 0.0, hover_height])

    descent_trajectory = linear_trajectory(hover_location, pregrasp_location, speed=speed)
    approach_trajectory = linear_trajectory(pregrasp_location, grasp_location, speed=speed)

    trajectory = concatenate_trajectories([descent_trajectory, approach_trajectory])
    return trajectory

    # orientation = sliding_grasp_orientation(approach_direction, angle)
