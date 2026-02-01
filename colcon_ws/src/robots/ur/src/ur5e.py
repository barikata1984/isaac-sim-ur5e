"""UR5e robot kinematic and dynamic parameters.

Based on Universal Robots UR5e specifications.
DH parameters use Modified DH convention (Craig): [a, d, alpha].
"""

from dataclasses import dataclass, field

import numpy as np

from ur.robot_param_base import RobotParametersBase


@dataclass
class UR5eParameters(RobotParametersBase):
    """UR5e robot parameters.

    DH parameters are in Modified DH convention (Craig): [a, d, alpha].
    Inertial properties are approximate values from URDF specifications.
    """

    n_joints: int = 6
    dh_convention: str = "modified"

    # Modified DH parameters [a, d, alpha] for each joint
    # From Universal Robots official documentation (Craig convention)
    dh_params: np.ndarray = field(default_factory=lambda: np.array([
        [0.0,       0.089159,   np.pi / 2],   # Joint 1
        [-0.425,    0.0,        0.0],          # Joint 2
        [-0.392,    0.0,        0.0],          # Joint 3
        [0.0,       0.10915,    np.pi / 2],   # Joint 4
        [0.0,       0.09465,   -np.pi / 2],   # Joint 5
        [0.0,       0.0823,     0.0],          # Joint 6
    ], dtype=np.float64))

    # Link masses [kg] - from URDF specifications
    link_masses: np.ndarray = field(default_factory=lambda: np.array([
        3.7,     # Link 1 (shoulder)
        8.393,   # Link 2 (upper arm)
        2.275,   # Link 3 (forearm)
        1.219,   # Link 4 (wrist 1)
        1.219,   # Link 5 (wrist 2)
        0.1879,  # Link 6 (wrist 3)
    ], dtype=np.float64))

    # Center of mass positions relative to link frame [m]
    # [x, y, z] in link frame when theta = 0
    link_com_positions: np.ndarray = field(default_factory=lambda: np.array([
        [0.0, -0.02561, 0.00193],       # Link 1
        [-0.2125, 0.0, 0.11336],        # Link 2
        [-0.15, 0.0, 0.0265],           # Link 3
        [0.0, -0.0018, 0.01634],        # Link 4
        [0.0, 0.0018, 0.01634],         # Link 5
        [0.0, 0.0, -0.001159],          # Link 6
    ], dtype=np.float64))

    # Link inertia tensors [kg*m^2] at center of mass
    # [[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]]
    link_inertias: np.ndarray = field(default_factory=lambda: np.array([
        # Link 1
        [[0.010267, 0.0, 0.0],
         [0.0, 0.010267, 0.0],
         [0.0, 0.0, 0.00666]],
        # Link 2
        [[0.22689, 0.0, 0.0],
         [0.0, 0.22689, 0.0],
         [0.0, 0.0, 0.0151074]],
        # Link 3
        [[0.049443, 0.0, 0.0],
         [0.0, 0.049443, 0.0],
         [0.0, 0.0, 0.004095]],
        # Link 4
        [[0.111172, 0.0, 0.0],
         [0.0, 0.111172, 0.0],
         [0.0, 0.0, 0.21942]],
        # Link 5
        [[0.111172, 0.0, 0.0],
         [0.0, 0.111172, 0.0],
         [0.0, 0.0, 0.21942]],
        # Link 6
        [[0.0171364, 0.0, 0.0],
         [0.0, 0.0171364, 0.0],
         [0.0, 0.0, 0.033822]],
    ], dtype=np.float64))

    @property
    def robot_name(self) -> str:
        return "UR5e"


def create_ur5e_parameters() -> UR5eParameters:
    """Factory function to create UR5e parameters.

    Returns:
        UR5eParameters instance.
    """
    return UR5eParameters()
