import bpy
import numpy as np

from syncloth.curves.bezier import quadratic_bezier
from syncloth.curves.circular_arc import circular_arc
from syncloth.curves.linear import linear_interpolation
from syncloth.visualization.points import add_points_as_instances

position0 = np.array([0.0, 0.0, 1.0])
position1 = np.array([1.0, 0.0, 0.0])  # Point that joint the linear and bezier curve
position2 = np.array([2.0, 0.0, 0.5])
position3 = np.array([3.0, 0.0, 0.0])
position4 = np.array([3.5, 0.0, 0.0])  # Center of the circular arc

# A curve is a function and a 1D domain
path = [
    linear_interpolation(position0, position1),
    quadratic_bezier(position1, position2, position3),
    circular_arc(position3, position4, np.array([0, 1, 0]), 3 * np.pi / 4),
]

samples = []
for curve in path:
    samples.extend([curve(s) for s in np.linspace(0, 1, 20)])

# Plot the path in Blender
bpy.ops.object.delete()  # Delete default cube
add_points_as_instances(samples, radius=0.02, color=(0, 1, 0))
