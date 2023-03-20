import bpy
import numpy as np
from pydrake.all import PiecewisePolynomial

from syncloth.visualization.points import add_points_as_instances

position0 = np.array([0.0, 0.0, 1.0])
position1 = np.array([1.0, 0.0, 0.0])  # Point that joint the linear and bezier segment
position2 = np.array([2.0, 0.0, 0.5])
position3 = np.array([3.0, 0.0, 0.0])

breaks = [0, 0.25, 0.5, 1]
control_points = np.column_stack([position0, position1, position2, position3])

path = PiecewisePolynomial.CubicShapePreserving(breaks, control_points)

samples = [path.value(t) for t in np.linspace(0, 1, 50)]

bpy.ops.object.delete()  # Delete default cube
add_points_as_instances(samples, radius=0.02, color=(0, 1, 0))
add_points_as_instances(control_points.T, radius=0.04, color=(1, 0, 0))
