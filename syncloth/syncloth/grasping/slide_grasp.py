import numpy as np
from airo_typing import Vector3DType
from scipy.spatial.transform import Rotation

from syncloth.geometry import flat_orientation
from syncloth.paths.concatenate import concatenate_trajectories
from syncloth.paths.constant import constant_trajectory
from syncloth.paths.linear import linear_trajectory
from syncloth.paths.orientation import combine_orientation_and_position_trajectory
from syncloth.paths.path import Path


def slide_grasp_orthogonal_approach_direction(edge_start, edge_end):
    """
    Assumes the edge is parallel to the x-y plane

                       |  approach_direction
                       v
    edge_start +-------+------>+ edge_end
               |               |

    TODO: mention the orientation convention that follows from the cross product.
    """
    edge = edge_end - edge_start
    edge_direction = edge / np.linalg.norm(edge)
    up = np.array([0, 0, 1])
    approach_direction = np.cross(edge_direction, up)

    return approach_direction


def slide_grasp_locations_from_edge(
    edge_start: Vector3DType,
    edge_end: Vector3DType,
    grasp_approach_direction: Vector3DType,
    grasp_depth: float = 0.05,
    inset: float = 0.05,
):
    """Diagram:

                inset
    edge_start +---->+-------------------------+<----+ edge_end
               |     |  grasp_depth            |     |
               |     v                         v     |
               |-----+                         +-----|
               | grasp_location0     grasp_location1 |
               |                                     |

    Args:
        edge_start: The first point of the edge.
        edge_end: The second point of the edge.
        grasp_approach_direction: The direction of the edge will be approached.
        grasp_depth: The depth of the grasp.
        inset: The distance along the edge to inset the grasp locations.
    """
    edge = edge_end - edge_start
    edge_direction = edge / np.linalg.norm(edge)

    grasp_location0 = edge_start.copy()
    grasp_location0 += inset * edge_direction
    grasp_location0 += grasp_depth * grasp_approach_direction

    grasp_location1 = edge_end.copy()
    grasp_location1 -= inset * edge_direction
    grasp_location1 += grasp_depth * grasp_approach_direction

    return grasp_location0, grasp_location1


def slide_grasp_orientation(approach_direction: Vector3DType, angle: float = np.pi / 6):
    """
    :param approach_direction: The direction along which the gripper will slide towards the grasp location.
    :param angle: The angle in radians that the gripper makes with the surface its sliding on.
    :return: The orientation of the gripper such that it makes an angle with the surface will slide on.
    """
    orientation = flat_orientation(approach_direction)
    local_Y = orientation[:, 1]
    rotation_local_Y = Rotation.from_rotvec(-angle * local_Y).as_matrix()
    orientation = rotation_local_Y @ orientation
    return orientation


def slide_grasp_position_trajectory(
    grasp_location: Vector3DType,
    approach_direction: Vector3DType,
    approach_distance: float = 0.05,
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

    # orientation = slide_grasp_orientation(approach_direction, angle)


def slide_grasp_trajectory(
    grasp_location, approach_direction, approach_distance, approach_angle, hover_height=0.05, speed=0.1
):
    grasp_position_trajectory = slide_grasp_position_trajectory(
        grasp_location, approach_direction, approach_distance, hover_height, speed
    )
    grasp_slide_duration = grasp_position_trajectory.duration

    grasp_orientation = slide_grasp_orientation(approach_direction, angle=approach_angle)
    grasp_orientation_trajectory = constant_trajectory(grasp_orientation, grasp_slide_duration)

    grasp_pose_trajectory = combine_orientation_and_position_trajectory(
        grasp_orientation_trajectory, grasp_position_trajectory
    )
    return grasp_pose_trajectory
