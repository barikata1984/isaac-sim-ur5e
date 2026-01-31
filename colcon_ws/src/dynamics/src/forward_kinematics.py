"""Forward kinematics for UR5e robot.

Provides standard DH-based forward kinematics and Jacobian computation
for tool0 velocity calculation.
"""

import numpy as np
from typing import Tuple

from dynamics.ur5e_parameters import UR5eParameters


# UR5e DH parameters (Standard DH convention)
# Format: [d, a, alpha] for each joint
# From Universal Robots official documentation
UR5E_DH_PARAMS = np.array([
    [0.089159,   0,       np.pi/2],   # Joint 1
    [0,         -0.425,   0],          # Joint 2
    [0,         -0.39225, 0],          # Joint 3
    [0.10915,    0,       np.pi/2],   # Joint 4
    [0.09465,    0,      -np.pi/2],   # Joint 5
    [0.0823,     0,       0],          # Joint 6
])


def dh_transform_standard(d: float, a: float, alpha: float, theta: float) -> np.ndarray:
    """Compute Standard DH transformation matrix.

    Standard DH convention:
    T = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)

    Args:
        d: Link offset along z.
        a: Link length along x.
        alpha: Link twist about x.
        theta: Joint angle about z.

    Returns:
        (4, 4) transformation matrix T_{i}^{i-1}.
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1]
    ], dtype=np.float64)


def forward_kinematics(q: np.ndarray) -> np.ndarray:
    """Compute forward kinematics for UR5e.

    Args:
        q: Joint positions [rad] (6,).

    Returns:
        T_base_tool0: (4, 4) transformation matrix of tool0 in base frame.
    """
    q = np.asarray(q, dtype=np.float64).flatten()
    T = np.eye(4)

    for i in range(6):
        d = UR5E_DH_PARAMS[i, 0]
        a = UR5E_DH_PARAMS[i, 1]
        alpha = UR5E_DH_PARAMS[i, 2]
        theta = q[i]

        T_i = dh_transform_standard(d, a, alpha, theta)
        T = T @ T_i

    return T


def forward_kinematics_all_frames(q: np.ndarray) -> list:
    """Compute FK for all joint frames.

    Args:
        q: Joint positions [rad] (6,).

    Returns:
        List of (4, 4) transformation matrices T_0_i for i = 1 to 6, then tool0.
    """
    q = np.asarray(q, dtype=np.float64).flatten()
    frames = []
    T = np.eye(4)

    for i in range(6):
        d = UR5E_DH_PARAMS[i, 0]
        a = UR5E_DH_PARAMS[i, 1]
        alpha = UR5E_DH_PARAMS[i, 2]
        theta = q[i]

        T_i = dh_transform_standard(d, a, alpha, theta)
        T = T @ T_i
        frames.append(T.copy())

    # The last frame (frames[5]) is already tool0 since d6=0.0823 is included
    return frames


def geometric_jacobian(q: np.ndarray) -> np.ndarray:
    """Compute geometric Jacobian in base frame.

    The geometric Jacobian relates joint velocities to end-effector
    velocity in base frame:
        [v]   = J(q) * dq
        [ω]

    For revolute joints:
        J_v_i = z_{i-1} × (p_ee - p_{i-1})
        J_ω_i = z_{i-1}

    Args:
        q: Joint positions [rad] (6,).

    Returns:
        J: (6, 6) Jacobian matrix [J_v; J_ω] in base frame.
    """
    q = np.asarray(q, dtype=np.float64).flatten()

    # Get all frames
    frames = forward_kinematics_all_frames(q)
    p_ee = frames[-1][:3, 3]  # tool0 position

    J = np.zeros((6, 6))

    # Base frame z-axis (joint 0)
    z_prev = np.array([0, 0, 1])
    p_prev = np.zeros(3)

    for i in range(6):
        # Linear velocity part: z_{i-1} × (p_ee - p_{i-1})
        J[:3, i] = np.cross(z_prev, p_ee - p_prev)

        # Angular velocity part: z_{i-1}
        J[3:, i] = z_prev

        # Update for next iteration
        if i < 5:
            # z-axis of frame i (after joint i rotates)
            z_prev = frames[i][:3, 2]
            p_prev = frames[i][:3, 3]
        # For the last joint, we don't need to update

    return J


def compute_tool0_twist_base(q: np.ndarray, dq: np.ndarray) -> np.ndarray:
    """Compute tool0 twist in base frame using geometric Jacobian.

    Args:
        q: Joint positions [rad] (6,).
        dq: Joint velocities [rad/s] (6,).

    Returns:
        V_base: Twist [v, ω] in base frame (6,).
                v: linear velocity of tool0 origin [m/s]
                ω: angular velocity [rad/s]
    """
    J = geometric_jacobian(q)
    V = J @ dq
    return V  # [v_x, v_y, v_z, ω_x, ω_y, ω_z]


def compute_tool0_twist_tool(q: np.ndarray, dq: np.ndarray) -> np.ndarray:
    """Compute tool0 twist in tool0 frame.

    Args:
        q: Joint positions [rad] (6,).
        dq: Joint velocities [rad/s] (6,).

    Returns:
        V_tool: Twist [ω, v] in tool0 frame (6,).
                ω: angular velocity [rad/s]
                v: linear velocity of tool0 origin [m/s]
    """
    # Get twist in base frame
    V_base = compute_tool0_twist_base(q, dq)
    v_base = V_base[:3]
    omega_base = V_base[3:]

    # Get tool0 rotation
    T = forward_kinematics(q)
    R = T[:3, :3]

    # Transform to tool0 frame
    omega_tool = R.T @ omega_base
    v_tool = R.T @ v_base

    # Return in [ω, v] order (standard twist convention)
    return np.concatenate([omega_tool, v_tool])


def compute_tool0_acceleration_base(
    q: np.ndarray,
    dq: np.ndarray,
    ddq: np.ndarray,
) -> np.ndarray:
    """Compute tool0 acceleration in base frame.

    Uses: a = J * ddq + dJ * dq

    Args:
        q: Joint positions [rad] (6,).
        dq: Joint velocities [rad/s] (6,).
        ddq: Joint accelerations [rad/s²] (6,).

    Returns:
        A_base: Acceleration [a, α] in base frame (6,).
                a: linear acceleration of tool0 origin [m/s²]
                α: angular acceleration [rad/s²]
    """
    q = np.asarray(q, dtype=np.float64).flatten()
    dq = np.asarray(dq, dtype=np.float64).flatten()
    ddq = np.asarray(ddq, dtype=np.float64).flatten()

    J = geometric_jacobian(q)

    # Compute Jacobian derivative numerically
    dt = 1e-7
    q_plus = q + dq * dt
    J_plus = geometric_jacobian(q_plus)
    dJ = (J_plus - J) / dt

    # a = J * ddq + dJ * dq
    A = J @ ddq + dJ @ dq

    return A


def compute_tool0_acceleration_tool(
    q: np.ndarray,
    dq: np.ndarray,
    ddq: np.ndarray,
) -> np.ndarray:
    """Compute tool0 acceleration in tool0 frame.

    Args:
        q: Joint positions [rad] (6,).
        dq: Joint velocities [rad/s] (6,).
        ddq: Joint accelerations [rad/s²] (6,).

    Returns:
        A_tool: Acceleration [α, a] in tool0 frame (6,).
                α: angular acceleration [rad/s²]
                a: linear acceleration of tool0 origin [m/s²]
    """
    # Get acceleration in base frame
    A_base = compute_tool0_acceleration_base(q, dq, ddq)
    a_base = A_base[:3]
    alpha_base = A_base[3:]

    # Get tool0 rotation
    T = forward_kinematics(q)
    R = T[:3, :3]

    # Transform to tool0 frame
    alpha_tool = R.T @ alpha_base
    a_tool = R.T @ a_base

    # Return in [α, a] order
    return np.concatenate([alpha_tool, a_tool])
