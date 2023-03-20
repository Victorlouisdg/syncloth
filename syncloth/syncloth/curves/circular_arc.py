import numpy as np
from scipy.spatial.transform import Rotation


def circular_arc(start, center, axis, central_angle) -> callable:
    unit_axis = axis / np.linalg.norm(axis)

    def circular_arc_func(s):
        rotation = Rotation.from_rotvec(s * central_angle * unit_axis)
        return center + rotation.apply(start - center)

    return circular_arc_func
