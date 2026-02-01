"""UR10 robot kinematic and dynamic parameters.

Based on Universal Robots UR10 specifications.
DH parameters use Standard DH convention: [d, a, alpha].
"""

from dataclasses import dataclass, field

import numpy as np

from ur.robot_param_base import RobotParametersBase


@dataclass
class UR10Parameters(RobotParametersBase):
    """UR10 robot parameters.

    DH parameters are in Standard DH convention: [d, a, alpha].
    Inertial properties are approximate values from URDF specifications.
    """

    n_joints: int = 6
    dh_convention: str = "standard"

    # Standard DH parameters [d, a, alpha] for each joint
    # From Universal Robots official documentation
    dh_params: np.ndarray = field(default_factory=lambda: np.array([
        [0.1273,    0,        np.pi / 2],   # Joint 1
        [0,        -0.612,    0],            # Joint 2
        [0,        -0.5723,   0],            # Joint 3
        [0.163941,  0,        np.pi / 2],   # Joint 4
        [0.1157,    0,       -np.pi / 2],   # Joint 5
        [0.0922,    0,        0],            # Joint 6
    ], dtype=np.float64))

    # Link masses [kg] - approximate values
    link_masses: np.ndarray = field(default_factory=lambda: np.array([
        7.1,     # Link 1 (shoulder)
        12.7,    # Link 2 (upper arm)
        4.27,    # Link 3 (forearm)
        2.0,     # Link 4 (wrist 1)
        2.0,     # Link 5 (wrist 2)
        0.365,   # Link 6 (wrist 3)
    ], dtype=np.float64))

    # Center of mass positions relative to joint frame [m]
    link_com_positions: np.ndarray = field(default_factory=lambda: np.array([
        [0.0, -0.03, 0.0],
        [-0.306, 0.0, 0.17],
        [-0.286, 0.0, 0.04],
        [0.0, -0.003, 0.025],
        [0.0, 0.003, 0.025],
        [0.0, 0.0, -0.002],
    ], dtype=np.float64))

    # Link inertia tensors [kg*m^2] at center of mass
    link_inertias: np.ndarray = field(default_factory=lambda: np.array([
        # Link 1
        [[0.03, 0.0, 0.0],
         [0.0, 0.03, 0.0],
         [0.0, 0.0, 0.02]],
        # Link 2
        [[0.5, 0.0, 0.0],
         [0.0, 0.5, 0.0],
         [0.0, 0.0, 0.04]],
        # Link 3
        [[0.15, 0.0, 0.0],
         [0.0, 0.15, 0.0],
         [0.0, 0.0, 0.01]],
        # Link 4
        [[0.01, 0.0, 0.0],
         [0.0, 0.01, 0.0],
         [0.0, 0.0, 0.01]],
        # Link 5
        [[0.01, 0.0, 0.0],
         [0.0, 0.01, 0.0],
         [0.0, 0.0, 0.01]],
        # Link 6
        [[0.002, 0.0, 0.0],
         [0.0, 0.002, 0.0],
         [0.0, 0.0, 0.002]],
    ], dtype=np.float64))

    @property
    def robot_name(self) -> str:
        return "UR10"


def create_ur10_parameters() -> UR10Parameters:
    """Factory function to create UR10 parameters.

    Returns:
        UR10Parameters instance.
    """
    return UR10Parameters()
