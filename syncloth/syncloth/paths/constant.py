from syncloth.paths.path import Path


def constant_trajectory(value, duration):
    def constant_function(t):
        return value

    return Path(constant_function, 0.0, duration)
