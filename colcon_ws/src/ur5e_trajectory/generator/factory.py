from typing import Union
from .spline import SplineConfig, SplineTrajectory
from .fourier import FourierConfig, FourierTrajectory

TrajectoryConfigUnion = Union[SplineConfig, FourierConfig]

def create_trajectory(config: TrajectoryConfigUnion):
    """Factory method to create a trajectory generator from a configuration object."""
    if isinstance(config, SplineConfig):
        return SplineTrajectory(config)
    elif isinstance(config, FourierConfig):
        return FourierTrajectory(config)
    else:
        raise ValueError(f"Unknown config type: {type(config)}")
