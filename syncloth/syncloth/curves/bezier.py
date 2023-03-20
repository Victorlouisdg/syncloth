def quadratic_bezier(p0, p1, p2):
    return lambda t: (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t**2 * p2


def cubic_bezier(p0, p1, p2, p3):
    return lambda t: (1 - t) ** 3 * p0 + 3 * (1 - t) ** 2 * t * p1 + 3 * (1 - t) * t**2 * p2 + t**3 * p3
