import numpy as np


def top_down_orientation(gripper_open_direction) -> np.ndarray:
    X = gripper_open_direction / np.linalg.norm(gripper_open_direction)  # np.array([-1, 0, 0])
    Z = np.array([0, 0, -1])
    Y = np.cross(Z, X)
    return np.column_stack([X, Y, Z])


def flat_orientation(gripper_forward_direction) -> np.ndarray:
    Z = gripper_forward_direction / np.linalg.norm(gripper_forward_direction)  # np.array([-1, 0, 0])
    X = np.array([0, 0, 1])
    Y = np.cross(Z, X)
    return np.column_stack([X, Y, Z])


def project_point_on_line(point, line):
    """
    Projects a point on a line.
    :param point: The point to project.
    :param line: The line w.r.t. which the point is projected. The line is defined by a point and a direction
    :return: The projected point.
    """
    point = np.array(point)
    line = np.array(line)
    point_on_line, line_direction = line  # TODO maybe add line as airo-typing type?
    line_direction = line_direction / np.linalg.norm(line_direction)
    dot = np.dot(point - point_on_line, line_direction)
    return point_on_line + dot * line_direction


def reflect_point_over_line(point, line):
    """
    Reflects a point over a line.
    :param point: The point to reflect.
    :param line: The line over which the point is reflected. The line is defined by a point and a direction
    :return: The reflected point.
    """
    point = np.array(point)
    line = np.array(line)

    projection = project_point_on_line(point, line)
    delta = projection - point
    return point + 2.0 * delta


def project_point_on_plane(point, plane):
    """
    Projects a point on a plane.
    :param point: The point to project.
    :param plane: The plane that the point is projected onto. The plane is defined by a point and a normal.
    :return: The projected point.
    """
    point = np.array(point)
    plane = np.array(plane)
    point_on_plane, plane_normal = plane  # TODO maybe add plane as airo-typing type?
    plane_normal = plane_normal / np.linalg.norm(plane_normal)
    dot = np.dot(point - point_on_plane, plane_normal)
    return point - dot * plane_normal


def reflect_point_over_plane(point, plane):
    """
    Reflects a point over a plane.
    :param point: The point to reflect.
    :param plane: The plane that the point is reflected over. The plane is defined by a point and a normal.
    :return: The reflected point.
    """
    point = np.array(point)
    plane = np.array(plane)

    projection = project_point_on_plane(point, plane)
    delta = projection - point
    return point + 2.0 * delta


# Order the keypoints
def angle_2D(v0, v1):
    # TODO: document.
    x1, y1, *_ = v0
    x2, y2, *_ = v1
    dot = x1 * x2 + y1 * y2  # dot product between [x1, y1] and [x2, y2]
    det = x1 * y2 - y1 * x2  # determinant
    angle = np.arctan2(det, dot)  # atan2(y, x) or atan2(sin, cos)
    return angle


def get_ordered_keypoints(keypoints):
    """
    Orders keypoints according to their angle w.r.t. a frame that is created by translating the world frame to the center of the cloth.
    the first keypoints is the one with the smallest, positive angle and they are sorted counter-clockwise.

    1---0
    |  /|
    | . |
    |   |
    2---3

    y
    |
    +--x
    """
    keypoints = np.array(keypoints)
    center = np.mean(keypoints, axis=0)
    x_axis = np.array([1, 0])
    angles = [angle_2D(x_axis, keypoint - center) for keypoint in keypoints]
    angles = [angle % (2 * np.pi) for angle in angles]  # make angles positive from 0 to 2*pi
    keypoints_sorted = keypoints[np.argsort(angles)]
    return list(keypoints_sorted)
