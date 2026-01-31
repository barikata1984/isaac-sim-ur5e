"""Spatial inertia matrix implementation.

Based on Lynch and Park (2017), Section 8.2.2, Equation 8.32.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np
from pymlg import SE3


@dataclass
class SpatialInertia:
    """Spatial inertia matrix G_b in R^{6x6}.

    The spatial inertia matrix expressed in a body frame {b} at the center
    of mass is defined as:

        G_b = [[I_b,  0  ],
               [0,   m*I ]]

    where:
        - I_b: 3x3 rotational inertia matrix about the center of mass
        - m: mass of the body
        - I: 3x3 identity matrix

    Attributes:
        I_b: Rotational inertia matrix (3x3) in body frame.
        mass: Mass of the rigid body.
    """

    I_b: np.ndarray  # (3, 3) rotational inertia matrix
    mass: float

    def __post_init__(self) -> None:
        """Validate inputs after initialization."""
        self.I_b = np.asarray(self.I_b, dtype=np.float64)
        if self.I_b.shape != (3, 3):
            raise ValueError(f"I_b must be (3, 3), got {self.I_b.shape}")
        if self.mass <= 0:
            raise ValueError(f"mass must be positive, got {self.mass}")

    def to_matrix(self) -> np.ndarray:
        """Return the 6x6 spatial inertia matrix G_b.

        Returns:
            G_b: (6, 6) spatial inertia matrix.
        """
        G = np.zeros((6, 6), dtype=np.float64)
        G[:3, :3] = self.I_b
        G[3:, 3:] = self.mass * np.eye(3)
        return G

    @classmethod
    def from_matrix(cls, G: np.ndarray) -> "SpatialInertia":
        """Create SpatialInertia from a 6x6 matrix.

        Args:
            G: (6, 6) spatial inertia matrix.

        Returns:
            SpatialInertia instance.
        """
        G = np.asarray(G, dtype=np.float64)
        if G.shape != (6, 6):
            raise ValueError(f"G must be (6, 6), got {G.shape}")
        I_b = G[:3, :3]
        mass = G[3, 3]  # Assume diagonal form
        return cls(I_b=I_b, mass=mass)

    def transform(self, T_ba: np.ndarray) -> "SpatialInertia":
        """Transform spatial inertia to a different frame.

        Based on Equation 8.42:
            G_a = [Ad_{T_ba}]^T * G_b * [Ad_{T_ba}]

        This is a generalization of Steiner's theorem.

        Args:
            T_ba: (4, 4) homogeneous transformation from {a} to {b}.

        Returns:
            Spatial inertia expressed in frame {a}.
        """
        Ad_T = SE3.adjoint(T_ba)  # 6x6 adjoint matrix
        G_b = self.to_matrix()
        G_a = Ad_T.T @ G_b @ Ad_T
        return SpatialInertia.from_matrix(G_a)

    @staticmethod
    def steiner_offset(I_cm: np.ndarray, mass: float, q: np.ndarray) -> np.ndarray:
        """Apply Steiner's theorem (parallel axis theorem).

        Based on Equation 8.27:
            I_q = I_b + m * (q^T * q * I - q * q^T)

        where q is the offset from the center of mass.

        Args:
            I_cm: (3, 3) inertia at center of mass.
            mass: Mass of the body.
            q: (3,) offset vector from center of mass to new origin.

        Returns:
            I_q: (3, 3) inertia about the new origin.
        """
        q = np.asarray(q, dtype=np.float64).flatten()
        I = np.eye(3)
        return I_cm + mass * (np.dot(q, q) * I - np.outer(q, q))

    def kinetic_energy(self, twist: np.ndarray) -> float:
        """Compute kinetic energy of the rigid body.

        Based on Equation 8.33:
            K = (1/2) * V_b^T * G_b * V_b

        Args:
            twist: (6,) body twist V_b = [omega_b, v_b].

        Returns:
            Kinetic energy in Joules.
        """
        twist = np.asarray(twist, dtype=np.float64).flatten()
        G = self.to_matrix()
        return 0.5 * twist @ G @ twist


def spatial_momentum(G: np.ndarray, twist: np.ndarray) -> np.ndarray:
    """Compute spatial momentum.

    Based on Equation 8.34:
        P_b = G_b * V_b = [I_b * omega_b, m * v_b]

    Args:
        G: (6, 6) spatial inertia matrix.
        twist: (6,) body twist.

    Returns:
        P_b: (6,) spatial momentum.
    """
    return G @ twist
