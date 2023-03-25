from typing import List

from syncloth.paths.path import Path


def concatenate_trajectories(trajectories: List[Path]):
    durations = [trajectory.duration for trajectory in trajectories]
    functions = [trajectory.function for trajectory in trajectories]

    def concatenated_functions(t):
        for function, duration in zip(functions, durations):
            if t <= duration:
                return function(t)
            t -= duration  # go to next trajectory
        return function(duration)  # if we get here, return last point

    return Path(concatenated_functions, 0.0, sum(durations))
