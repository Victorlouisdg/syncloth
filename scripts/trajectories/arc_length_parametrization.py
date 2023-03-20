import bpy
import numpy as np

from syncloth.curves.arc_length import arc_length_parametrize, integrate_arc_length
from syncloth.curves.bezier import quadratic_bezier
from syncloth.visualization.points import add_points_as_instances

position0 = np.array([0.0, 0.0, 0.0])
position1 = np.array([0.5, 0.0, 2.0])
position2 = np.array([1.0, 0.0, 0.0])

# Plot the path in Blender
bpy.ops.object.delete()  # Delete default cube

bezier = quadratic_bezier(position0, position1, position2)
samples0 = [bezier(s) for s in np.linspace(0, 1, 40)]
arc_length0 = integrate_arc_length(bezier)
print("arc_length0 =", arc_length0)

# arc_length1 = integrate_arc_length(bezier_reparametrized, end=arc_length0, num=1000)
bezier_reparametrized, arc_length1 = arc_length_parametrize(bezier, 0.0, 1.0)
samples1 = [bezier_reparametrized(s) for s in np.linspace(0, arc_length1, 40)]
print("arc_length1 =", arc_length1)

# Plot the path in Blender
bpy.ops.object.delete()  # Delete default cube
add_points_as_instances(samples0, radius=0.02, color=(0, 1, 0))

samples1_shifted = [p + np.array([1.05, 0, 0]) for p in samples1]
add_points_as_instances(samples1_shifted, radius=0.02, color=(0, 0, 1))
