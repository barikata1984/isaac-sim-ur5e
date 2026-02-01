"""Base class for robot kinematic and dynamic parameters.

This module defines the abstract base class for robot parameters that can be
used with the dynamics package for Newton-Euler inverse dynamics computation.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Literal

import numpy as np


@dataclass
class RobotParametersBase(ABC):
    """Abstract base class for robot kinematic and dynamic parameters.

    This class defines the interface for robot parameters required by
    the Newton-Euler dynamics algorithm. Subclasses should provide
    specific values for each robot model.

    Attributes:
        n_joints: Number of joints in the robot.
        dh_params: DH parameters array of shape (n_joints, 3).
                   For standard DH: each row is [d, a, alpha].
                   For modified DH: each row is [a, d, alpha].
        dh_convention: DH convention used ("standard" or "modified").
        link_masses: Array of link masses [kg] of shape (n_joints,).
        link_com_positions: CoM positions relative to joint frame [m],
                            shape (n_joints, 3).
        link_inertias: Inertia tensors at CoM [kg*m^2],
                       shape (n_joints, 3, 3).
    """

    n_joints: int
    dh_params: np.ndarray
    dh_convention: Literal["standard", "modified"]
    link_masses: np.ndarray
    link_com_positions: np.ndarray
    link_inertias: np.ndarray

    @property
    @abstractmethod
    def robot_name(self) -> str:
        """Return the robot model name."""
        pass

    def get_spatial_inertia_at_com(self, link_index: int) -> np.ndarray:
        """Get 6x6 spatial inertia matrix at link CoM.

        The spatial inertia matrix G at CoM is:
            G = [[I,   0  ],
                 [0, m*I_3]]

        where I is the 3x3 rotational inertia and m is the mass.

        Args:
            link_index: 0-indexed link number.

        Returns:
            (6, 6) spatial inertia matrix at CoM.
        """
        if not 0 <= link_index < self.n_joints:
            raise ValueError(f"link_index must be 0-{self.n_joints-1}, got {link_index}")

        mass = self.link_masses[link_index]
        inertia = self.link_inertias[link_index]

        G = np.zeros((6, 6))
        G[:3, :3] = inertia
        G[3:, 3:] = mass * np.eye(3)

        return G

    def get_all_spatial_inertias_at_com(self) -> list:
        """Get all spatial inertia matrices at CoM.

        Returns:
            List of (6, 6) spatial inertia matrices for each link.
        """
        return [self.get_spatial_inertia_at_com(i) for i in range(self.n_joints)]
