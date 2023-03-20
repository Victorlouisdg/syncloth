def linear_interpolation(a, b) -> callable:
    return lambda t: a + t * (b - a)
