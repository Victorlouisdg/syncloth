from scipy.spatial.transform import Rotation

from syncloth.paths.path import Path


def circular_arc_path(start, center, axis, max_angle: float) -> Path:
    def circular_arc_function(angle):
        rotation = Rotation.from_rotvec(angle * axis)
        return center + rotation.apply(start - center)

    return Path(circular_arc_function, start=0.0, end=max_angle)
