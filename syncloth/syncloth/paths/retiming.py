from syncloth.paths.arc_length import arc_length_parametrize
from syncloth.paths.path import Path


def scale_speed(trajectory: Path, factor: float) -> Path:
    return Path(lambda t: trajectory.function(t * factor), start=0.0, end=trajectory.end / factor)


def minimum_jerk(t: float):
    """Minimum jerk function for t in [0, 1]"""
    return 10 * (t**3) - 15 * (t**4) + 6 * (t**5)


def minimum_jerk_trajectory(path: Path, peak_speed: float = 0.5):
    path_arc_length_parametrized = arc_length_parametrize(path)
    arc_length = path_arc_length_parametrized.end

    # An arc length parametrized curve has constant speed of 1 m/s
    # The minimum jerk functions "wraps" the t parameter, but does not affect the domain.
    path_minimum_jerk = Path(
        lambda t: path_arc_length_parametrized.function(minimum_jerk(t / arc_length) * arc_length),
        start=0.0,
        end=arc_length,
    )

    # Minimum jerk has max speed of 1.875 m/s at t=0.5 (TODO verify the math for this)
    # We do this by scaling the domain of the function by the fraction 1.875 / peak_speed
    # E.g. for a peak_speed 1.0 m/s, scale speed by 0.53333
    scaling_factor = peak_speed / 1.875
    path_rescaled_speed = scale_speed(path_minimum_jerk, scaling_factor)

    return path_rescaled_speed
