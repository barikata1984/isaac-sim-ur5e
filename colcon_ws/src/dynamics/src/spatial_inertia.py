"""Spatial inertia matrix utilities.

This module provides functions for computing 6x6 spatial inertia matrices
using pymlg twist convention: [ω, v] (angular velocity, linear velocity).

The spatial inertia matrix relates twists to wrenches:
    F = G @ V̇ - ad_V^T @ G @ V

where F = [τ, f] is a wrench (torque, force) in [ω, v] convention.
"""

import numpy as np
from pymlg.numpy import SE3, SO3


def skew(v: np.ndarray) -> np.ndarray:
    """Convert 3-vector to skew-symmetric matrix."""
    return SO3.wedge(v)


def spatial_inertia_at_com(
    mass: float,
    inertia: np.ndarray,
) -> np.ndarray:
    """Compute 6x6 spatial inertia matrix at center of mass.

    At the center of mass, the spatial inertia matrix has a simple form
    with no coupling between linear and angular parts.

    For pymlg convention [ω, v]:
        G = [[I_c,    0    ],
             [0,      m*I_3]]

    where I_c is the 3x3 rotational inertia about CoM.

    Args:
        mass: Link mass [kg].
        inertia: (3, 3) rotational inertia tensor at CoM [kg*m^2].

    Returns:
        (6, 6) spatial inertia matrix at CoM.
    """
    G = np.zeros((6, 6))
    G[:3, :3] = np.asarray(inertia)
    G[3:, 3:] = mass * np.eye(3)
    return G


def spatial_inertia_at_frame(
    mass: float,
    inertia_at_com: np.ndarray,
    com_position: np.ndarray,
) -> np.ndarray:
    """Compute 6x6 spatial inertia matrix at a different frame.

    Transforms the spatial inertia from the CoM to a frame displaced
    by com_position. Uses the parallel axis theorem for inertia.

    For pymlg convention [ω, v], when the frame is displaced from
    CoM by vector p (from frame origin to CoM):
        G = [[I_c + m*[p]×[p]×^T,    m*[p]×   ],
             [m*[p]×^T,              m*I_3    ]]

    Note: [p]×^T = -[p]×

    Args:
        mass: Link mass [kg].
        inertia_at_com: (3, 3) rotational inertia tensor at CoM [kg*m^2].
        com_position: (3,) position vector from frame origin to CoM [m].

    Returns:
        (6, 6) spatial inertia matrix at the frame origin.
    """
    p = np.asarray(com_position).ravel()
    p_skew = skew(p)

    G = np.zeros((6, 6))

    # Inertia block (upper-left): I_c + m*[p]×[p]×^T
    # [p]×[p]×^T = -[p]×[p]× = ||p||^2*I - p*p^T
    G[:3, :3] = inertia_at_com + mass * (np.dot(p, p) * np.eye(3) - np.outer(p, p))

    # Cross-coupling (upper-right): m*[p]×
    G[:3, 3:] = mass * p_skew

    # Cross-coupling (lower-left): m*[p]×^T = -m*[p]×
    G[3:, :3] = -mass * p_skew

    # Mass block (lower-right): m*I_3
    G[3:, 3:] = mass * np.eye(3)

    return G


def _adjoint_se3_inv(T: np.ndarray) -> np.ndarray:
    """Compute adjoint of inverse transformation Ad_{T^{-1}}.

    Uses pymlg SE3.adjoint with [ω, v] convention.

    Args:
        T: (4, 4) homogeneous transformation matrix.

    Returns:
        (6, 6) adjoint matrix of inverse transformation.
    """
    T_inv = SE3.inverse(T)
    return SE3.adjoint(T_inv)


def transform_spatial_inertia(
    G: np.ndarray,
    T: np.ndarray,
) -> np.ndarray:
    """Transform spatial inertia matrix to a new frame.

    If G_a is the spatial inertia in frame A, and T_ba is the
    transformation from A to B, then:
        G_b = Ad_{T_ba}^{-T} @ G_a @ Ad_{T_ba}^{-1}

    Uses pymlg convention [ω, v].

    Args:
        G: (6, 6) spatial inertia matrix in frame A.
        T: (4, 4) transformation from frame A to frame B.

    Returns:
        (6, 6) spatial inertia matrix in frame B.
    """
    Ad_inv = _adjoint_se3_inv(T)
    return Ad_inv.T @ G @ Ad_inv


def is_positive_definite(G: np.ndarray, tol: float = 1e-10) -> bool:
    """Check if spatial inertia matrix is positive definite.

    Args:
        G: (6, 6) spatial inertia matrix.
        tol: Tolerance for eigenvalue check.

    Returns:
        True if all eigenvalues are positive.
    """
    eigenvalues = np.linalg.eigvalsh(G)
    return np.all(eigenvalues > tol)


def is_symmetric(G: np.ndarray, tol: float = 1e-10) -> bool:
    """Check if spatial inertia matrix is symmetric.

    Args:
        G: (6, 6) spatial inertia matrix.
        tol: Tolerance for symmetry check.

    Returns:
        True if G is symmetric within tolerance.
    """
    return np.allclose(G, G.T, atol=tol)
