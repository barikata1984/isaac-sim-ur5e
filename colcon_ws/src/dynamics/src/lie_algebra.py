"""Lie algebra operations for SE(3) and SO(3).

Based on Lynch and Park (2017), Chapter 8.
Uses pymlg for underlying matrix operations.
"""

import numpy as np
from pymlg import SE3, SO3


def skew(v: np.ndarray) -> np.ndarray:
    """Convert a 3-vector to a skew-symmetric matrix.

    [v] = [[ 0, -v3,  v2],
           [v3,   0, -v1],
           [-v2, v1,   0]]

    Args:
        v: (3,) vector.

    Returns:
        (3, 3) skew-symmetric matrix.
    """
    v = np.asarray(v, dtype=np.float64).flatten()
    return SO3.wedge(v)


def unskew(S: np.ndarray) -> np.ndarray:
    """Convert a skew-symmetric matrix to a 3-vector.

    Args:
        S: (3, 3) skew-symmetric matrix.

    Returns:
        (3,) vector.
    """
    return SO3.vee(S).flatten()


def se3_wedge(twist: np.ndarray) -> np.ndarray:
    """Convert a 6-vector twist to a 4x4 se(3) matrix.

    Args:
        twist: (6,) twist [omega, v].

    Returns:
        (4, 4) se(3) matrix.
    """
    twist = np.asarray(twist, dtype=np.float64).flatten()
    return SE3.wedge(twist)


def se3_vee(xi_hat: np.ndarray) -> np.ndarray:
    """Convert a 4x4 se(3) matrix to a 6-vector twist.

    Args:
        xi_hat: (4, 4) se(3) matrix.

    Returns:
        (6,) twist [omega, v].
    """
    return SE3.vee(xi_hat).flatten()


def ad(twist: np.ndarray) -> np.ndarray:
    """Compute the Lie bracket operator [ad_V].

    Based on Equation 8.38:
        [ad_V] = [[[omega],    0    ],
                  [ [v]  , [omega] ]]

    where twist V = (omega, v).

    Args:
        twist: (6,) twist [omega, v].

    Returns:
        (6, 6) ad matrix.
    """
    twist = np.asarray(twist, dtype=np.float64).flatten()
    xi_hat = SE3.wedge(twist)
    return SE3.adjoint_algebra(xi_hat)


def ad_transpose(twist: np.ndarray) -> np.ndarray:
    """Compute the transpose of the Lie bracket operator [ad_V]^T.

    Based on Equation 8.39:
        [ad_V]^T * F = [-[omega]*m - [v]*f, -[omega]*f]

    where F = (m, f) is a wrench.

    Args:
        twist: (6,) twist [omega, v].

    Returns:
        (6, 6) ad^T matrix.
    """
    return ad(twist).T


def adjoint(T: np.ndarray) -> np.ndarray:
    """Compute the Adjoint representation [Ad_T].

    For T = [[R, p], [0, 1]] in SE(3):
        [Ad_T] = [[R,       0],
                  [[p]*R,   R]]

    Args:
        T: (4, 4) homogeneous transformation matrix.

    Returns:
        (6, 6) Adjoint matrix.
    """
    T = np.asarray(T, dtype=np.float64)
    return SE3.adjoint(T)


def adjoint_transpose(T: np.ndarray) -> np.ndarray:
    """Compute the transpose of the Adjoint representation [Ad_T]^T.

    Used for transforming wrenches between frames.

    Args:
        T: (4, 4) homogeneous transformation matrix.

    Returns:
        (6, 6) Adjoint transpose matrix.
    """
    return adjoint(T).T


def se3_exp(twist: np.ndarray, theta: float = 1.0) -> np.ndarray:
    """Compute the matrix exponential exp([S]*theta).

    Args:
        twist: (6,) screw axis S = [omega, v].
        theta: Rotation angle (rad) or displacement for prismatic joints.

    Returns:
        (4, 4) homogeneous transformation matrix.
    """
    twist = np.asarray(twist, dtype=np.float64).flatten()
    return SE3.Exp(twist * theta)


def se3_log(T: np.ndarray) -> np.ndarray:
    """Compute the matrix logarithm of a transformation.

    Args:
        T: (4, 4) homogeneous transformation matrix.

    Returns:
        (6,) twist-angle [omega*theta, v*theta].
    """
    return SE3.Log(T).flatten()


def inverse_transform(T: np.ndarray) -> np.ndarray:
    """Compute the inverse of a homogeneous transformation.

    T^{-1} = [[R^T, -R^T*p], [0, 1]]

    Args:
        T: (4, 4) homogeneous transformation matrix.

    Returns:
        (4, 4) inverse transformation matrix.
    """
    T = np.asarray(T, dtype=np.float64)
    return SE3.inverse(T)


def rotation_from_transform(T: np.ndarray) -> np.ndarray:
    """Extract rotation matrix from homogeneous transformation.

    Args:
        T: (4, 4) homogeneous transformation matrix.

    Returns:
        (3, 3) rotation matrix.
    """
    return T[:3, :3]


def translation_from_transform(T: np.ndarray) -> np.ndarray:
    """Extract translation vector from homogeneous transformation.

    Args:
        T: (4, 4) homogeneous transformation matrix.

    Returns:
        (3,) translation vector.
    """
    return T[:3, 3]


def transform_from_rotation_translation(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    """Create homogeneous transformation from rotation and translation.

    Args:
        R: (3, 3) rotation matrix.
        p: (3,) translation vector.

    Returns:
        (4, 4) homogeneous transformation matrix.
    """
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(p).flatten()
    return T
