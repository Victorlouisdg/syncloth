import bpy
import numpy as np

from syncloth.visualization.points import add_points_as_instances

position0 = np.array([0.0, 0.0, 0.0])
position1 = np.array([0.5, 0.0, 2.0])
position2 = np.array([1.0, 0.0, 0.0])


def quadratic_bezier(a, b, c) -> callable:
    return lambda s: (1 - s) ** 2 * a + 2 * (1 - s) * s * b + s**2 * c


def integrate_arc_length(curve, start=0.0, end=1.0, num=50):
    # TODO implement smarter method e.g. Gauss-Kronrod or Gaussian quadrature
    arc_length_sum = 0
    s_range = np.linspace(start, end, num)
    for i in range(len(s_range) - 1):
        s = s_range[i]
        s_next = s_range[i + 1]
        arc_length_sum += np.linalg.norm(curve(s_next) - curve(s))

    return arc_length_sum


def create_arc_length_to_parameter_map(curve, start=0.0, end=1.0, num=100):
    arc_length_to_s: dict[float, float] = {0.0: 0.0}

    arc_length_sum = 0
    s_range = np.linspace(start, end, num)
    for i in range(len(s_range) - 1):
        s = s_range[i]
        s_next = s_range[i + 1]
        arc_length_sum += np.linalg.norm(curve(s_next) - curve(s))
        arc_length_to_s[arc_length_sum] = s_next

    return arc_length_to_s, arc_length_sum


def arc_length_parametrize(curve, start=0.0, end=1.0):
    arc_length_to_s, arc_length = create_arc_length_to_parameter_map(curve, start, end)
    arc_lengths = np.array(list(arc_length_to_s.keys()))

    def arc_length_parametrized(t):
        if np.isclose(t, 0.0):
            return curve(start)
        if np.isclose(t, arc_length):
            return curve(end)

        # TODO verify correctness of the 3 lines below
        i = np.searchsorted(arc_lengths, t, side="right")
        arc_length_next = arc_lengths[min(i, len(arc_lengths) - 1)]
        arc_length_prev = arc_lengths[max(i - 1, 0)]

        s_prev = arc_length_to_s[arc_length_prev]
        s_next = arc_length_to_s[arc_length_next]

        weight_next = (t - arc_length_prev) / (arc_length_next - arc_length_prev)
        weight_prev = 1 - weight_next

        s_interpolated = s_next * weight_next + s_prev * weight_prev
        return curve(s_interpolated)

    return arc_length_parametrized, arc_length


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
