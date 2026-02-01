"""UR3 robot kinematic and dynamic parameters.

Based on Universal Robots UR3 specifications.
DH parameters use Standard DH convention: [d, a, alpha].
"""

from dataclasses import dataclass, field

import numpy as np

from ur.robot_param_base import RobotParametersBase


@dataclass
class UR3Parameters(RobotParametersBase):
    """UR3 robot parameters.

    DH parameters are in Standard DH convention: [d, a, alpha].
    Inertial properties are approximate values from URDF specifications.
    """

    n_joints: int = 6
    dh_convention: str = "standard"

    # Standard DH parameters [d, a, alpha] for each joint
    # From Universal Robots official documentation
    dh_params: np.ndarray = field(default_factory=lambda: np.array([
        [0.15185,   0,        np.pi / 2],   # Joint 1
        [0,        -0.24355,  0],            # Joint 2
        [0,        -0.2132,   0],            # Joint 3
        [0.13105,   0,        np.pi / 2],   # Joint 4
        [0.08535,   0,       -np.pi / 2],   # Joint 5
        [0.0921,    0,        0],            # Joint 6
    ], dtype=np.float64))

    # Link masses [kg] - approximate values
    link_masses: np.ndarray = field(default_factory=lambda: np.array([
        2.0,     # Link 1 (shoulder)
        3.42,    # Link 2 (upper arm)
        1.26,    # Link 3 (forearm)
        0.8,     # Link 4 (wrist 1)
        0.8,     # Link 5 (wrist 2)
        0.35,    # Link 6 (wrist 3)
    ], dtype=np.float64))

    # Center of mass positions relative to joint frame [m]
    link_com_positions: np.ndarray = field(default_factory=lambda: np.array([
        [0.0, -0.02, 0.0],
        [-0.12, 0.0, 0.08],
        [-0.10, 0.0, 0.02],
        [0.0, -0.002, 0.015],
        [0.0, 0.002, 0.015],
        [0.0, 0.0, -0.001],
    ], dtype=np.float64))

    # Link inertia tensors [kg*m^2] at center of mass
    link_inertias: np.ndarray = field(default_factory=lambda: np.array([
        # Link 1
        [[0.004, 0.0, 0.0],
         [0.0, 0.004, 0.0],
         [0.0, 0.0, 0.003]],
        # Link 2
        [[0.04, 0.0, 0.0],
         [0.0, 0.04, 0.0],
         [0.0, 0.0, 0.003]],
        # Link 3
        [[0.015, 0.0, 0.0],
         [0.0, 0.015, 0.0],
         [0.0, 0.0, 0.002]],
        # Link 4
        [[0.002, 0.0, 0.0],
         [0.0, 0.002, 0.0],
         [0.0, 0.0, 0.002]],
        # Link 5
        [[0.002, 0.0, 0.0],
         [0.0, 0.002, 0.0],
         [0.0, 0.0, 0.002]],
        # Link 6
        [[0.001, 0.0, 0.0],
         [0.0, 0.001, 0.0],
         [0.0, 0.0, 0.001]],
    ], dtype=np.float64))

    @property
    def robot_name(self) -> str:
        return "UR3"


def create_ur3_parameters() -> UR3Parameters:
    """Factory function to create UR3 parameters.

    Returns:
        UR3Parameters instance.
    """
    return UR3Parameters()
