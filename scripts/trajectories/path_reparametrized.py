import bpy
import numpy as np

from syncloth.curves.arc_length import arc_length_parametrize, integrate_arc_length
from syncloth.curves.bezier import quadratic_bezier
from syncloth.curves.circular_arc import circular_arc
from syncloth.curves.linear import linear_interpolation
from syncloth.visualization.points import add_points_as_instances

position0 = np.array([0.0, 0.0, 1.0])
position1 = np.array([1.0, 0.0, 0.0])  # Point that joint the linear and bezier curve
position2 = np.array([2.0, 0.0, 0.5])
position3 = np.array([3.0, 0.0, 0.0])
position4 = np.array([3.5, 0.0, 0.0])  # Center of the circular arc


path = [
    linear_interpolation(position0, position1),  # range [0, 1]
    quadratic_bezier(position1, position2, position3),  # range [0, 1]
    circular_arc(position3, position4, np.array([0, 1, 0]), 3 * np.pi / 4),  # range [0, 1]
]

for curve in path:
    print(integrate_arc_length(curve))


path_reparametrized = []
ends = []
for curve in path:
    curve_reparametrized, arc_length = arc_length_parametrize(curve)
    path_reparametrized.append(curve_reparametrized)
    ends.append(arc_length)

samples = []
arc_length_total = sum(ends)

parameter_range = np.linspace(0, arc_length_total, 50, endpoint=True)

for s in parameter_range:
    curve_index = 0
    while s > ends[curve_index]:
        s -= ends[curve_index]
        curve_index += 1

    samples.append(path_reparametrized[curve_index](s))


# Plot the path in Blender
bpy.ops.object.delete()  # Delete default cube
add_points_as_instances(samples, radius=0.02, color=(0, 1, 0))
