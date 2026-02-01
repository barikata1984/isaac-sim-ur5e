"""UR Robot package for Isaac Sim.

This package provides:
- Robot parameters for UR3, UR5e, UR10, UR10e
- Robot spawning functions for Isaac Sim (requires isaacsim)
"""

# Robot parameters base class (always available)
from ur.robot_param_base import RobotParametersBase

# Robot-specific parameters (always available)
from ur.ur3 import UR3Parameters, create_ur3_parameters
from ur.ur5e import UR5eParameters, create_ur5e_parameters
from ur.ur10 import UR10Parameters, create_ur10_parameters
from ur.ur10e import UR10eParameters, create_ur10e_parameters

__all__ = [
    # Parameters base
    "RobotParametersBase",
    # Robot parameters
    "UR3Parameters",
    "UR5eParameters",
    "UR10Parameters",
    "UR10eParameters",
    # Factory functions
    "create_ur3_parameters",
    "create_ur5e_parameters",
    "create_ur10_parameters",
    "create_ur10e_parameters",
]

# Isaac Sim dependent imports (lazy loading)
# These are only imported when explicitly accessed
def __getattr__(name):
    """Lazy loading for Isaac Sim dependent modules."""
    if name in ("spawn_ur_robot", "spawn_ur5e"):
        from ur.spawning import spawn_ur_robot, spawn_ur5e
        return spawn_ur_robot if name == "spawn_ur_robot" else spawn_ur5e
    elif name in ("get_robot_asset_path", "ROBOT_ASSET_MAPPING"):
        from ur.isaac_utils import get_robot_asset_path, ROBOT_ASSET_MAPPING
        return get_robot_asset_path if name == "get_robot_asset_path" else ROBOT_ASSET_MAPPING
    raise AttributeError(f"module 'ur' has no attribute '{name}'")
