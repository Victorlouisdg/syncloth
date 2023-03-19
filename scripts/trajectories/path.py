import bpy
import numpy as np
from scipy.spatial.transform import Rotation

from syncloth.visualization.points import add_points_as_instances

position0 = np.array([0.0, 0.0, 1.0])
position1 = np.array([1.0, 0.0, 0.0])  # Point that joint the linear and bezier segment
position2 = np.array([2.0, 0.0, 0.5])
position3 = np.array([3.0, 0.0, 0.0])
position4 = np.array([3.5, 0.0, 0.0])  # Center of the circular arc


def linear_interpolation(a, b) -> callable:
    return lambda t: a + t * (b - a)


def quadratic_bezier(a, b, c) -> callable:
    return lambda t: (1 - t) ** 2 * a + 2 * (1 - t) * t * b + t**2 * c


def circular_arc(start, center, axis, central_angle) -> callable:
    unit_axis = axis / np.linalg.norm(axis)

    def circular_arc_func(t):
        rotation = Rotation.from_rotvec(t * central_angle * unit_axis)
        return center + rotation.apply(start - center)

    return circular_arc_func


path = [
    linear_interpolation(position0, position1),
    quadratic_bezier(position1, position2, position3),
    circular_arc(position3, position4, np.array([0, 1, 0]), 3 * np.pi / 4),
]

samples = []
for segment in path:
    samples.extend([segment(t) for t in np.linspace(0, 1, 50)])

# Plot the path in Blender
bpy.ops.object.delete()  # Delete default cube
add_points_as_instances(samples, radius=0.02, color=(0, 1, 0))
