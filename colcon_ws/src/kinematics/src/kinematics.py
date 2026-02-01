"""Forward kinematics and Jacobian computation for serial link robots.

This module provides kinematics computations using Modified DH convention
and pymlg twist convention [ω, v] (angular velocity, linear velocity).
"""

from typing import List, Literal

import numpy as np
from pymlg.numpy import SE3, SO3


def _dh_transform(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    """Compute Modified DH transformation matrix.

    The Modified DH convention transforms from frame i-1 to frame i:
    T = Rot_x(alpha_{i-1}) @ Trans_x(a_{i-1}) @ Rot_z(theta_i) @ Trans_z(d_i)

    Args:
        a: Link length (distance along x_{i-1}).
        d: Link offset (distance along z_i).
        alpha: Link twist (rotation about x_{i-1}).
        theta: Joint angle (rotation about z_i).

    Returns:
        (4, 4) homogeneous transformation matrix.
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -sa * d],
        [st * sa, ct * sa, ca, ca * d],
        [0, 0, 0, 1]
    ])


def forward_kinematics(
    q: np.ndarray,
    dh_params: np.ndarray,
) -> np.ndarray:
    """Compute forward kinematics to end-effector frame.

    Uses Modified DH convention (Craig) where DH parameters are [a, d, alpha].

    Args:
        q: Joint angles (n,).
        dh_params: DH parameters (n, 3) with columns [a, d, alpha].

    Returns:
        (4, 4) homogeneous transformation from base to end-effector.
    """
    q = np.asarray(q).ravel()
    n_joints = len(q)

    T = np.eye(4)
    for i in range(n_joints):
        a = dh_params[i, 0]
        d = dh_params[i, 1]
        alpha = dh_params[i, 2]
        theta = q[i]

        T_i = _dh_transform(a, d, alpha, theta)
        T = T @ T_i

    return T


def forward_kinematics_all_frames(
    q: np.ndarray,
    dh_params: np.ndarray,
) -> List[np.ndarray]:
    """Compute forward kinematics for all link frames.

    Returns transformation from base to each link frame.

    Args:
        q: Joint angles (n,).
        dh_params: DH parameters (n, 3) with columns [a, d, alpha].

    Returns:
        List of (4, 4) transformations [T_0, T_01, T_02, ..., T_0n].
        T_0 is identity (base frame), T_0n is end-effector frame.
    """
    q = np.asarray(q).ravel()
    n_joints = len(q)

    frames = [np.eye(4)]  # Base frame
    T = np.eye(4)

    for i in range(n_joints):
        a = dh_params[i, 0]
        d = dh_params[i, 1]
        alpha = dh_params[i, 2]
        theta = q[i]

        T_i = _dh_transform(a, d, alpha, theta)
        T = T @ T_i
        frames.append(T.copy())

    return frames


def body_jacobian(
    q: np.ndarray,
    dh_params: np.ndarray,
) -> np.ndarray:
    """Compute body Jacobian at end-effector frame.

    The body Jacobian relates joint velocities to end-effector twist
    expressed in the end-effector frame:
        V_body = J_b @ dq

    Uses pymlg convention: V = [ω, v].

    The body Jacobian is related to space Jacobian by:
        J_b = Ad_{T_0n}^{-1} @ J_s

    Args:
        q: Joint angles (n,).
        dh_params: DH parameters (n, 3) with columns [a, d, alpha].

    Returns:
        (6, n) body Jacobian matrix.
    """
    # Get space Jacobian
    J_s = space_jacobian(q, dh_params)

    # Get end-effector transformation
    T_0n = forward_kinematics(q, dh_params)

    # Body Jacobian: J_b = Ad_{T_0n^{-1}} @ J_s
    T_n0 = SE3.inverse(T_0n)
    Ad_inv = SE3.adjoint(T_n0)
    J_b = Ad_inv @ J_s

    return J_b


def _numerical_jacobian(
    q: np.ndarray,
    dh_params: np.ndarray,
    eps: float = 1e-7,
) -> np.ndarray:
    """Compute Jacobian via numerical differentiation.

    Uses pymlg [ω, v] convention.

    Args:
        q: Joint angles (n,).
        dh_params: DH parameters.
        eps: Perturbation size.

    Returns:
        (6, n) Jacobian matrix [angular; linear] = [ω; v].
    """
    q = np.asarray(q).ravel()
    n_joints = len(q)

    J = np.zeros((6, n_joints))
    T0 = forward_kinematics(q, dh_params)
    p0 = T0[:3, 3]
    R0 = T0[:3, :3]

    for i in range(n_joints):
        q_plus = q.copy()
        q_plus[i] += eps

        T_plus = forward_kinematics(q_plus, dh_params)
        p_plus = T_plus[:3, 3]
        R_plus = T_plus[:3, :3]

        # Angular velocity (from rotation derivative)
        # dR/dt = [ω]× @ R, so [ω]× = dR/dt @ R^T
        dR = (R_plus - R0) / eps
        omega_skew = dR @ R0.T
        J[0, i] = omega_skew[2, 1]  # ω_x
        J[1, i] = omega_skew[0, 2]  # ω_y
        J[2, i] = omega_skew[1, 0]  # ω_z

        # Linear velocity (position derivative)
        J[3:, i] = (p_plus - p0) / eps

    return J


def space_jacobian(
    q: np.ndarray,
    dh_params: np.ndarray,
) -> np.ndarray:
    """Compute space Jacobian at base frame.

    The space Jacobian relates joint velocities to end-effector twist
    expressed in the base (space) frame:
        V_space = J_s @ dq

    Uses pymlg convention: V = [ω, v].

    Currently uses numerical differentiation for robustness with
    Modified DH parameters.

    Args:
        q: Joint angles (n,).
        dh_params: DH parameters (n, 3) with columns [a, d, alpha].

    Returns:
        (6, n) space Jacobian matrix.
    """
    return _numerical_jacobian(q, dh_params)


def geometric_jacobian(
    q: np.ndarray,
    dh_params: np.ndarray,
    frame: Literal["space", "body"] = "space",
) -> np.ndarray:
    """Compute geometric Jacobian.

    Uses pymlg convention: V = [ω, v].

    Args:
        q: Joint angles (n,).
        dh_params: DH parameters (n, 3) with columns [a, d, alpha].
        frame: "space" for base frame, "body" for end-effector frame.

    Returns:
        (6, n) Jacobian matrix.
    """
    if frame == "space":
        return space_jacobian(q, dh_params)
    else:
        return body_jacobian(q, dh_params)


def tool0_twist(
    q: np.ndarray,
    dq: np.ndarray,
    dh_params: np.ndarray,
    frame: Literal["space", "body"] = "space",
) -> np.ndarray:
    """Compute end-effector twist (velocity).

    V = J(q) @ dq

    Uses pymlg convention: V = [ω, v] = [ωx, ωy, ωz, vx, vy, vz].

    Args:
        q: Joint angles (n,).
        dq: Joint velocities (n,).
        dh_params: DH parameters (n, 3) with columns [a, d, alpha].
        frame: "space" for base frame, "body" for end-effector frame.

    Returns:
        (6,) twist vector [ω, v].
    """
    J = geometric_jacobian(q, dh_params, frame)
    return J @ np.asarray(dq).ravel()


def tool0_acceleration(
    q: np.ndarray,
    dq: np.ndarray,
    ddq: np.ndarray,
    dh_params: np.ndarray,
    frame: Literal["space", "body"] = "space",
    dt: float = 1e-6,
) -> np.ndarray:
    """Compute end-effector acceleration.

    V̇ = J̇(q) @ dq + J(q) @ ddq

    The Jacobian time derivative is computed numerically:
    J̇ ≈ (J(q + dq*dt) - J(q)) / dt

    Uses pymlg convention: V̇ = [ω̇, v̇].

    Args:
        q: Joint angles (n,).
        dq: Joint velocities (n,).
        ddq: Joint accelerations (n,).
        dh_params: DH parameters (n, 3) with columns [a, d, alpha].
        frame: "space" for base frame, "body" for end-effector frame.
        dt: Time step for numerical differentiation.

    Returns:
        (6,) acceleration vector [ω̇, v̇].
    """
    q = np.asarray(q).ravel()
    dq = np.asarray(dq).ravel()
    ddq = np.asarray(ddq).ravel()

    # Current Jacobian
    J = geometric_jacobian(q, dh_params, frame)

    # Jacobian at perturbed configuration
    q_plus = q + dq * dt
    J_plus = geometric_jacobian(q_plus, dh_params, frame)

    # Numerical Jacobian derivative
    J_dot = (J_plus - J) / dt

    # Total acceleration
    return J_dot @ dq + J @ ddq
