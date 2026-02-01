"""UR10e robot kinematic and dynamic parameters.

Based on Universal Robots UR10e specifications.
DH parameters use Standard DH convention: [d, a, alpha].
"""

from dataclasses import dataclass, field

import numpy as np

from ur.robot_param_base import RobotParametersBase


@dataclass
class UR10eParameters(RobotParametersBase):
    """UR10e robot parameters.

    DH parameters are in Standard DH convention: [d, a, alpha].
    Inertial properties are approximate values from URDF specifications.
    """

    n_joints: int = 6
    dh_convention: str = "standard"

    # Standard DH parameters [d, a, alpha] for each joint
    # From Universal Robots official documentation
    dh_params: np.ndarray = field(default_factory=lambda: np.array([
        [0.1807,    0,        np.pi / 2],   # Joint 1
        [0,        -0.6127,   0],            # Joint 2
        [0,        -0.57155,  0],            # Joint 3
        [0.17415,   0,        np.pi / 2],   # Joint 4
        [0.11985,   0,       -np.pi / 2],   # Joint 5
        [0.11655,   0,        0],            # Joint 6
    ], dtype=np.float64))

    # Link masses [kg] - approximate values
    link_masses: np.ndarray = field(default_factory=lambda: np.array([
        7.369,   # Link 1 (shoulder)
        10.45,   # Link 2 (upper arm)
        4.321,   # Link 3 (forearm)
        2.18,    # Link 4 (wrist 1)
        2.033,   # Link 5 (wrist 2)
        0.907,   # Link 6 (wrist 3)
    ], dtype=np.float64))

    # Center of mass positions relative to joint frame [m]
    link_com_positions: np.ndarray = field(default_factory=lambda: np.array([
        [0.0, -0.03, 0.0],
        [-0.306, 0.0, 0.15],
        [-0.286, 0.0, 0.035],
        [0.0, -0.003, 0.02],
        [0.0, 0.003, 0.02],
        [0.0, 0.0, -0.002],
    ], dtype=np.float64))

    # Link inertia tensors [kg*m^2] at center of mass
    link_inertias: np.ndarray = field(default_factory=lambda: np.array([
        # Link 1
        [[0.035, 0.0, 0.0],
         [0.0, 0.035, 0.0],
         [0.0, 0.0, 0.022]],
        # Link 2
        [[0.45, 0.0, 0.0],
         [0.0, 0.45, 0.0],
         [0.0, 0.0, 0.035]],
        # Link 3
        [[0.13, 0.0, 0.0],
         [0.0, 0.13, 0.0],
         [0.0, 0.0, 0.009]],
        # Link 4
        [[0.012, 0.0, 0.0],
         [0.0, 0.012, 0.0],
         [0.0, 0.0, 0.009]],
        # Link 5
        [[0.011, 0.0, 0.0],
         [0.0, 0.011, 0.0],
         [0.0, 0.0, 0.008]],
        # Link 6
        [[0.005, 0.0, 0.0],
         [0.0, 0.005, 0.0],
         [0.0, 0.0, 0.004]],
    ], dtype=np.float64))

    @property
    def robot_name(self) -> str:
        return "UR10e"


def create_ur10e_parameters() -> UR10eParameters:
    """Factory function to create UR10e parameters.

    Returns:
        UR10eParameters instance.
    """
    return UR10eParameters()
