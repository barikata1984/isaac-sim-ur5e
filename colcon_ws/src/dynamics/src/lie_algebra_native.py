"""Native Lie algebra operations for SE(3) and SO(3).

Pure NumPy implementation without external dependencies (pymlg).
Based on Lynch and Park (2017), Chapter 3 and 8.
"""

import numpy as np
from typing import Tuple


def skew(v: np.ndarray) -> np.ndarray:
    """Convert a 3-vector to a skew-symmetric matrix.

    [v]^ = [[ 0, -v3,  v2],
            [v3,   0, -v1],
            [-v2, v1,   0]]

    Args:
        v: (3,) vector.

    Returns:
        (3, 3) skew-symmetric matrix.
    """
    v = np.asarray(v, dtype=np.float64).flatten()
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ], dtype=np.float64)


def unskew(S: np.ndarray) -> np.ndarray:
    """Convert a skew-symmetric matrix to a 3-vector.

    Args:
        S: (3, 3) skew-symmetric matrix.

    Returns:
        (3,) vector.
    """
    return np.array([S[2, 1], S[0, 2], S[1, 0]], dtype=np.float64)


def so3_exp(omega: np.ndarray, theta: float = None) -> np.ndarray:
    """Compute the matrix exponential of so(3) element.

    Rodrigues' formula:
        exp([omega]^*theta) = I + sin(theta)*[omega]^ + (1-cos(theta))*[omega]^2

    Args:
        omega: (3,) rotation axis (unit vector if theta given separately).
        theta: Rotation angle [rad]. If None, uses norm of omega.

    Returns:
        (3, 3) rotation matrix in SO(3).
    """
    omega = np.asarray(omega, dtype=np.float64).flatten()

    if theta is None:
        theta = np.linalg.norm(omega)
        if theta < 1e-10:
            return np.eye(3)
        omega = omega / theta

    if abs(theta) < 1e-10:
        return np.eye(3)

    omega_hat = skew(omega)
    omega_hat_sq = omega_hat @ omega_hat

    R = np.eye(3) + np.sin(theta) * omega_hat + (1 - np.cos(theta)) * omega_hat_sq
    return R


def so3_log(R: np.ndarray) -> np.ndarray:
    """Compute the matrix logarithm of SO(3) element.

    Args:
        R: (3, 3) rotation matrix.

    Returns:
        (3,) rotation vector omega*theta.
    """
    R = np.asarray(R, dtype=np.float64)

    # Check for identity
    trace = np.trace(R)
    if abs(trace - 3) < 1e-10:
        return np.zeros(3)

    # Check for theta = pi
    if abs(trace + 1) < 1e-10:
        # Find the column of R + I with largest norm
        R_plus_I = R + np.eye(3)
        norms = [np.linalg.norm(R_plus_I[:, i]) for i in range(3)]
        i = np.argmax(norms)
        omega = R_plus_I[:, i] / norms[i]
        return omega * np.pi

    theta = np.arccos((trace - 1) / 2)
    omega_hat = (R - R.T) / (2 * np.sin(theta))
    omega = unskew(omega_hat)

    return omega * theta


def se3_wedge(twist: np.ndarray) -> np.ndarray:
    """Convert a 6-vector twist to a 4x4 se(3) matrix.

    Args:
        twist: (6,) twist [omega, v].

    Returns:
        (4, 4) se(3) matrix.
    """
    twist = np.asarray(twist, dtype=np.float64).flatten()
    omega = twist[:3]
    v = twist[3:]

    xi_hat = np.zeros((4, 4), dtype=np.float64)
    xi_hat[:3, :3] = skew(omega)
    xi_hat[:3, 3] = v
    return xi_hat


def se3_vee(xi_hat: np.ndarray) -> np.ndarray:
    """Convert a 4x4 se(3) matrix to a 6-vector twist.

    Args:
        xi_hat: (4, 4) se(3) matrix.

    Returns:
        (6,) twist [omega, v].
    """
    omega = unskew(xi_hat[:3, :3])
    v = xi_hat[:3, 3]
    return np.concatenate([omega, v])


def se3_exp(twist: np.ndarray, theta: float = 1.0) -> np.ndarray:
    """Compute the matrix exponential exp([S]*theta).

    Based on Lynch and Park Proposition 3.9 (Product of Exponentials).

    Args:
        twist: (6,) screw axis S = [omega, v].
        theta: Rotation angle (rad) or displacement for prismatic joints.

    Returns:
        (4, 4) homogeneous transformation matrix.
    """
    twist = np.asarray(twist, dtype=np.float64).flatten()
    omega = twist[:3]
    v = twist[3:]

    omega_norm = np.linalg.norm(omega)

    T = np.eye(4, dtype=np.float64)

    if omega_norm < 1e-10:
        # Pure translation
        T[:3, 3] = v * theta
    else:
        # Normalize
        omega_unit = omega / omega_norm
        v_scaled = v / omega_norm
        angle = omega_norm * theta

        # Rotation part
        R = so3_exp(omega_unit, angle)
        T[:3, :3] = R

        # Translation part (Equation 3.51 in Lynch & Park)
        omega_hat = skew(omega_unit)
        G = np.eye(3) * angle + (1 - np.cos(angle)) * omega_hat + \
            (angle - np.sin(angle)) * (omega_hat @ omega_hat)
        T[:3, 3] = G @ v_scaled

    return T


def se3_log(T: np.ndarray) -> np.ndarray:
    """Compute the matrix logarithm of a transformation.

    Args:
        T: (4, 4) homogeneous transformation matrix.

    Returns:
        (6,) twist-angle [omega*theta, v*theta].
    """
    T = np.asarray(T, dtype=np.float64)
    R = T[:3, :3]
    p = T[:3, 3]

    omega_theta = so3_log(R)
    theta = np.linalg.norm(omega_theta)

    if theta < 1e-10:
        # Pure translation
        return np.concatenate([np.zeros(3), p])

    omega = omega_theta / theta
    omega_hat = skew(omega)

    # G^{-1} from Equation 3.92
    G_inv = np.eye(3) / theta - 0.5 * omega_hat + \
            (1 / theta - 0.5 / np.tan(theta / 2)) * (omega_hat @ omega_hat)

    v_theta = G_inv @ p

    return np.concatenate([omega_theta, v_theta])


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
    omega = twist[:3]
    v = twist[3:]

    omega_hat = skew(omega)
    v_hat = skew(v)

    ad_V = np.zeros((6, 6), dtype=np.float64)
    ad_V[:3, :3] = omega_hat
    ad_V[3:, :3] = v_hat
    ad_V[3:, 3:] = omega_hat

    return ad_V


def ad_transpose(twist: np.ndarray) -> np.ndarray:
    """Compute the transpose of the Lie bracket operator [ad_V]^T.

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
    R = T[:3, :3]
    p = T[:3, 3]

    p_hat = skew(p)

    Ad_T = np.zeros((6, 6), dtype=np.float64)
    Ad_T[:3, :3] = R
    Ad_T[3:, :3] = p_hat @ R
    Ad_T[3:, 3:] = R

    return Ad_T


def adjoint_transpose(T: np.ndarray) -> np.ndarray:
    """Compute the transpose of the Adjoint representation [Ad_T]^T.

    Args:
        T: (4, 4) homogeneous transformation matrix.

    Returns:
        (6, 6) Adjoint transpose matrix.
    """
    return adjoint(T).T


def inverse_transform(T: np.ndarray) -> np.ndarray:
    """Compute the inverse of a homogeneous transformation.

    T^{-1} = [[R^T, -R^T*p], [0, 1]]

    Args:
        T: (4, 4) homogeneous transformation matrix.

    Returns:
        (4, 4) inverse transformation matrix.
    """
    T = np.asarray(T, dtype=np.float64)
    R = T[:3, :3]
    p = T[:3, 3]

    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ p

    return T_inv


def rotation_from_transform(T: np.ndarray) -> np.ndarray:
    """Extract rotation matrix from homogeneous transformation.

    Args:
        T: (4, 4) homogeneous transformation matrix.

    Returns:
        (3, 3) rotation matrix.
    """
    return T[:3, :3].copy()


def translation_from_transform(T: np.ndarray) -> np.ndarray:
    """Extract translation vector from homogeneous transformation.

    Args:
        T: (4, 4) homogeneous transformation matrix.

    Returns:
        (3,) translation vector.
    """
    return T[:3, 3].copy()


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
