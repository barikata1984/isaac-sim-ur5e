"""Offline verification of dynamics calculations.

This module verifies the dynamics calculations using:
1. Pre-recorded trajectory data (JSON files with q, dq, ddq)
2. Analytical checks (consistency, symmetry, etc.)
3. Comparison between different implementations
"""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple, Optional

import numpy as np

from dynamics.newton_euler import NewtonEulerDynamics, create_ur5e_dynamics
from dynamics.newton_euler_native import NewtonEulerNative, create_ur5e_native_dynamics
from dynamics.lie_algebra import (
    adjoint,
    rotation_from_transform,
    transform_from_rotation_translation,
    se3_exp,
)
from dynamics.ur5e_parameters import UR5eParameters


@dataclass
class TrajectoryFrame:
    """Single frame from trajectory data."""

    qpos: np.ndarray  # Joint positions [rad]
    qvel: np.ndarray  # Joint velocities [rad/s]
    qacc: np.ndarray  # Joint accelerations [rad/s²]


def load_trajectory(json_path: str) -> Tuple[List[TrajectoryFrame], float]:
    """Load trajectory from JSON file.

    Args:
        json_path: Path to trajectory JSON file.

    Returns:
        Tuple of (frames, fps).
    """
    with open(json_path, 'r') as f:
        data = json.load(f)

    fps = data.get('fps', 60.0)
    frames = []

    for frame_data in data['frames']:
        if isinstance(frame_data, list) and len(frame_data) >= 3:
            qpos = np.array(frame_data[0])
            qvel = np.array(frame_data[1])
            qacc = np.array(frame_data[2])
        else:
            # Handle old format
            qpos = np.array(frame_data)
            qvel = np.zeros(6)
            qacc = np.zeros(6)

        frames.append(TrajectoryFrame(qpos=qpos, qvel=qvel, qacc=qacc))

    return frames, fps


def compute_forward_kinematics_dh(
    params: UR5eParameters,
    q: np.ndarray,
) -> np.ndarray:
    """Compute forward kinematics using standard DH parameters.

    Uses Modified DH convention (Craig) to compute transformation
    from base frame to tool0 frame.

    Args:
        params: Robot parameters.
        q: Joint positions [rad] (6,).

    Returns:
        T_base_tool0: (4, 4) transformation matrix of tool0 in base frame.
    """
    T = np.eye(4)
    dh = params.dh_params

    for i in range(params.n_joints):
        theta = q[i]

        if i == 0:
            # First joint: use alpha from row 0, d from row 0
            a = dh[0, 0]      # 0
            d = dh[0, 1]      # 0.089159
            alpha = dh[0, 2]  # pi/2
        else:
            # Modified DH: use a, alpha from previous row, d from current row
            a = dh[i - 1, 0]
            alpha = dh[i - 1, 2]
            d = dh[i, 1]

        # Modified DH transformation matrix
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        if i == 0:
            # For first joint, the standard form is:
            # Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
            T_i = np.array([
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0, sa, ca, d],
                [0, 0, 0, 1]
            ], dtype=np.float64)
        else:
            # Standard Modified DH
            T_i = np.array([
                [ct, -st, 0, a],
                [st * ca, ct * ca, -sa, -d * sa],
                [st * sa, ct * sa, ca, d * ca],
                [0, 0, 0, 1]
            ], dtype=np.float64)

        T = T @ T_i

    return T


def compute_forward_kinematics(
    params: UR5eParameters,
    q: np.ndarray,
) -> np.ndarray:
    """Compute forward kinematics to get tool0 pose.

    This function uses the standard UR5e DH parameters to compute
    forward kinematics, which is independent of the Newton-Euler
    CoM frame convention.

    Args:
        params: Robot parameters.
        q: Joint positions [rad] (6,).

    Returns:
        T_base_tool0: (4, 4) transformation matrix of tool0 in base frame.
    """
    # UR5e standard DH parameters (from Universal Robots documentation)
    # Using Modified DH convention
    # | i | a_{i-1} | d_i      | alpha_{i-1} |
    # |---|---------|----------|-------------|
    # | 1 | 0       | 0.089159 | 0           |
    # | 2 | 0       | 0        | pi/2        |
    # | 3 | -0.425  | 0        | 0           |
    # | 4 | -0.392  | 0        | 0           |
    # | 5 | 0       | 0.10915  | pi/2        |
    # | 6 | 0       | 0.09465  | -pi/2       |
    # | 7 | 0       | 0.0823   | 0           |  (tool0)

    # Standard UR5e DH parameters (different ordering than params.dh_params)
    # Format: [a_{i-1}, d_i, alpha_{i-1}]
    dh_standard = np.array([
        [0,       0.089159,  0],          # Joint 1
        [0,       0,         np.pi/2],    # Joint 2
        [-0.425,  0,         0],          # Joint 3
        [-0.392,  0,         0],          # Joint 4
        [0,       0.10915,   np.pi/2],    # Joint 5
        [0,       0.09465,  -np.pi/2],    # Joint 6
    ])

    T = np.eye(4)

    for i in range(6):
        a = dh_standard[i, 0]
        d = dh_standard[i, 1]
        alpha = dh_standard[i, 2]
        theta = q[i]

        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        # Modified DH transformation
        T_i = np.array([
            [ct, -st, 0, a],
            [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca],
            [0, 0, 0, 1]
        ], dtype=np.float64)

        T = T @ T_i

    # Apply tool0 offset (d7 = 0.0823)
    T_tool = np.eye(4)
    T_tool[2, 3] = 0.0823
    T = T @ T_tool

    return T


def verify_twist_transformation(
    dynamics: NewtonEulerDynamics,
    q: np.ndarray,
    dq: np.ndarray,
) -> dict:
    """Verify that twist transformation is correct.

    The twist V_tool0 in tool0 frame should satisfy:
    - Angular velocity part ω is the same as body angular velocity
    - Linear velocity part v is the velocity of the tool0 origin

    Args:
        dynamics: Dynamics calculator.
        q: Joint positions [rad] (6,).
        dq: Joint velocities [rad/s] (6,).

    Returns:
        Dictionary with verification results.
    """
    # Get twist from dynamics
    V_tool0 = dynamics.get_end_effector_twist(q, dq)
    omega_tool0 = V_tool0[:3]  # Angular velocity in tool0 frame
    v_tool0 = V_tool0[3:]      # Linear velocity of tool0 origin in tool0 frame

    # Get forward kinematics
    params = dynamics.params
    T_base_tool0 = compute_forward_kinematics(params, q)
    R_base_tool0 = T_base_tool0[:3, :3]

    # Transform to base frame for comparison
    omega_base = R_base_tool0 @ omega_tool0
    v_base = R_base_tool0 @ v_tool0

    return {
        'twist_tool0': V_tool0,
        'omega_tool0': omega_tool0,
        'v_tool0': v_tool0,
        'omega_base': omega_base,
        'v_base': v_base,
        'T_base_tool0': T_base_tool0,
    }


def numerical_differentiation_check(
    dynamics: NewtonEulerDynamics,
    q: np.ndarray,
    dq: np.ndarray,
    dt: float = 1e-5,
) -> dict:
    """Verify twist calculation using numerical differentiation.

    Compare analytical twist with numerical approximation:
    v ≈ (p(q + dq*dt) - p(q)) / dt

    Args:
        dynamics: Dynamics calculator.
        q: Joint positions [rad] (6,).
        dq: Joint velocities [rad/s] (6,).
        dt: Time step for numerical differentiation.

    Returns:
        Dictionary with comparison results.
    """
    params = dynamics.params

    # Current pose
    T0 = compute_forward_kinematics(params, q)
    p0 = T0[:3, 3]
    R0 = T0[:3, :3]

    # Pose after small time step
    q_next = q + dq * dt
    T1 = compute_forward_kinematics(params, q_next)
    p1 = T1[:3, 3]
    R1 = T1[:3, :3]

    # Numerical velocity (in base frame)
    v_numerical_base = (p1 - p0) / dt

    # Numerical angular velocity (approximate)
    # ω × dt ≈ log(R1 @ R0.T)
    dR = R1 @ R0.T
    # For small rotations: ω ≈ [dR32-dR23, dR13-dR31, dR21-dR12] / (2*dt)
    omega_numerical_base = np.array([
        dR[2, 1] - dR[1, 2],
        dR[0, 2] - dR[2, 0],
        dR[1, 0] - dR[0, 1],
    ]) / (2 * dt)

    # Analytical twist
    V_tool0 = dynamics.get_end_effector_twist(q, dq)
    omega_analytical_tool0 = V_tool0[:3]
    v_analytical_tool0 = V_tool0[3:]

    # Transform analytical to base frame
    omega_analytical_base = R0 @ omega_analytical_tool0
    v_analytical_base = R0 @ v_analytical_tool0

    # Compute errors
    v_error = np.linalg.norm(v_analytical_base - v_numerical_base)
    omega_error = np.linalg.norm(omega_analytical_base - omega_numerical_base)

    return {
        'v_analytical_base': v_analytical_base,
        'v_numerical_base': v_numerical_base,
        'v_error': v_error,
        'omega_analytical_base': omega_analytical_base,
        'omega_numerical_base': omega_numerical_base,
        'omega_error': omega_error,
        'dt': dt,
    }


def run_trajectory_verification(
    json_path: str,
    output_path: Optional[str] = None,
) -> dict:
    """Run verification on a trajectory file.

    Args:
        json_path: Path to trajectory JSON file.
        output_path: Optional path to save results.

    Returns:
        Dictionary with verification statistics.
    """
    frames, fps = load_trajectory(json_path)
    dynamics = create_ur5e_dynamics()

    results = {
        'num_frames': len(frames),
        'fps': fps,
        'v_errors': [],
        'omega_errors': [],
        'max_v_error': 0.0,
        'max_omega_error': 0.0,
        'mean_v_error': 0.0,
        'mean_omega_error': 0.0,
    }

    for i, frame in enumerate(frames):
        if np.allclose(frame.qvel, 0):
            continue  # Skip stationary frames

        check = numerical_differentiation_check(
            dynamics, frame.qpos, frame.qvel
        )

        results['v_errors'].append(check['v_error'])
        results['omega_errors'].append(check['omega_error'])

    if results['v_errors']:
        results['max_v_error'] = max(results['v_errors'])
        results['max_omega_error'] = max(results['omega_errors'])
        results['mean_v_error'] = np.mean(results['v_errors'])
        results['mean_omega_error'] = np.mean(results['omega_errors'])

    if output_path:
        with open(output_path, 'w') as f:
            json.dump(results, f, indent=2, default=float)

    return results


def run_comprehensive_verification() -> dict:
    """Run comprehensive verification tests.

    Returns:
        Dictionary with all verification results.
    """
    dynamics = create_ur5e_dynamics()
    dynamics_native = create_ur5e_native_dynamics()

    results = {
        'numerical_check': [],
        'implementation_comparison': [],
        'jacobian_check': [],
    }

    # Test configurations
    test_configs = [
        # (q, dq) pairs
        (np.zeros(6), np.array([1.0, 0, 0, 0, 0, 0])),
        (np.zeros(6), np.array([0, 1.0, 0, 0, 0, 0])),
        (np.zeros(6), np.array([0, 0, 1.0, 0, 0, 0])),
        (np.zeros(6), np.array([0, 0, 0, 1.0, 0, 0])),
        (np.zeros(6), np.array([0, 0, 0, 0, 1.0, 0])),
        (np.zeros(6), np.array([0, 0, 0, 0, 0, 1.0])),
        (np.array([0.1, -0.5, 0.3, -1.0, 0.5, 0.2]),
         np.array([0.5, -0.3, 0.2, -0.1, 0.4, -0.2])),
        (np.array([np.pi/4, -np.pi/3, np.pi/6, -np.pi/2, np.pi/4, 0]),
         np.array([1.0, 0.5, -0.3, 0.2, -0.1, 0.4])),
    ]

    print("=" * 60)
    print("Comprehensive Dynamics Verification")
    print("=" * 60)

    # 1. Numerical differentiation check
    print("\n1. Numerical Differentiation Check")
    print("-" * 40)

    for i, (q, dq) in enumerate(test_configs):
        check = numerical_differentiation_check(dynamics, q, dq)
        results['numerical_check'].append({
            'config_index': i,
            'v_error': check['v_error'],
            'omega_error': check['omega_error'],
        })
        print(f"  Config {i}: v_error={check['v_error']:.2e}, "
              f"omega_error={check['omega_error']:.2e}")

    # 2. Implementation comparison (pymlg vs native)
    print("\n2. Implementation Comparison (pymlg vs native)")
    print("-" * 40)

    for i, (q, dq) in enumerate(test_configs):
        V_pymlg = dynamics.get_end_effector_twist(q, dq)
        V_native = dynamics_native.get_end_effector_twist(q, dq)
        error = np.linalg.norm(V_pymlg - V_native)
        results['implementation_comparison'].append({
            'config_index': i,
            'error': error,
        })
        status = "✓" if error < 1e-10 else "✗"
        print(f"  Config {i}: error={error:.2e} {status}")

    # 3. Jacobian-based check
    print("\n3. Jacobian-based Velocity Check")
    print("-" * 40)

    for i, (q, dq) in enumerate(test_configs):
        # Compute Jacobian numerically
        eps = 1e-7
        J = np.zeros((6, 6))

        for j in range(6):
            q_plus = q.copy()
            q_plus[j] += eps
            q_minus = q.copy()
            q_minus[j] -= eps

            T_plus = compute_forward_kinematics(dynamics.params, q_plus)
            T_minus = compute_forward_kinematics(dynamics.params, q_minus)

            # Position part
            J[3:, j] = (T_plus[:3, 3] - T_minus[:3, 3]) / (2 * eps)

            # Rotation part (approximate)
            R_plus = T_plus[:3, :3]
            R_minus = T_minus[:3, :3]
            dR = R_plus @ R_minus.T
            J[:3, j] = np.array([
                dR[2, 1] - dR[1, 2],
                dR[0, 2] - dR[2, 0],
                dR[1, 0] - dR[0, 1],
            ]) / (2 * eps)

        # Velocity from Jacobian (in base frame)
        V_jacobian_base = J @ dq

        # Velocity from dynamics (transform to base)
        V_tool0 = dynamics.get_end_effector_twist(q, dq)
        T = compute_forward_kinematics(dynamics.params, q)
        R = T[:3, :3]
        V_dynamics_base = np.concatenate([R @ V_tool0[:3], R @ V_tool0[3:]])

        error = np.linalg.norm(V_jacobian_base - V_dynamics_base)
        results['jacobian_check'].append({
            'config_index': i,
            'error': error,
        })
        status = "✓" if error < 1e-4 else "✗"
        print(f"  Config {i}: error={error:.2e} {status}")

    # Summary
    print("\n" + "=" * 60)
    print("Summary")
    print("=" * 60)

    numerical_errors = [r['v_error'] + r['omega_error']
                        for r in results['numerical_check']]
    impl_errors = [r['error'] for r in results['implementation_comparison']]
    jacobian_errors = [r['error'] for r in results['jacobian_check']]

    print(f"  Numerical check max error: {max(numerical_errors):.2e}")
    print(f"  Implementation comparison max error: {max(impl_errors):.2e}")
    print(f"  Jacobian check max error: {max(jacobian_errors):.2e}")

    return results


if __name__ == '__main__':
    run_comprehensive_verification()
