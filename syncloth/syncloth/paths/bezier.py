from typing import Callable

import numpy as np

from syncloth.paths.path import Path


def quadratic_bezier_function(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray) -> Callable[[float], np.ndarray]:
    return lambda t: (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t**2 * p2


def quadratic_bezier_path(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray) -> Path:
    function = quadratic_bezier_function(p0, p1, p2)
    return Path(function, start=0.0, end=1.0)
