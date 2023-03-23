import numpy as np


def integrate_arc_length(curve, start=0.0, end=1.0, num=50):
    # TODO implement smarter method e.g. Gauss-Kronrod or Gaussian quadrature
    arc_length_sum = 0
    s_range = np.linspace(start, end, num)
    for i in range(len(s_range) - 1):
        s = s_range[i]
        s_next = s_range[i + 1]
        arc_length_sum += np.linalg.norm(curve(s_next) - curve(s))

    return arc_length_sum


def create_arc_length_to_parameter_map(curve, start=0.0, end=1.0, num=1000):
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
