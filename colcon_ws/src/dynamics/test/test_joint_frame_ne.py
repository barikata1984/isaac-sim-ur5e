"""Test joint-frame based Newton-Euler implementation.

Verifies that the velocity calculation matches the geometric Jacobian.
"""

import numpy as np

from dynamics.newton_euler_joint_frame import create_ur5e_joint_frame_dynamics
from dynamics.newton_euler import create_ur5e_dynamics
from dynamics.forward_kinematics import (
    compute_tool0_twist_tool,
    compute_tool0_twist_base,
    forward_kinematics,
    forward_kinematics_all_frames,
)
from dynamics.lie_algebra import inverse_transform


def test_frame_positions():
    """Verify that joint frame chain matches standard DH FK."""
    dynamics = create_ur5e_joint_frame_dynamics()

    q = np.zeros(6)

    print("\n" + "="*80)
    print("FRAME POSITION VERIFICATION (q=0)")
    print("="*80)

    # Get DH frame positions
    dh_frames = forward_kinematics_all_frames(q)

    # Get Newton-Euler frame positions
    state = dynamics._forward_iterations(q, np.zeros(6), np.zeros(6))

    T_base = np.eye(4)
    print("\n  Joint Frame Positions:")
    print(f"  {'Link':>6} {'DH Position':>40} {'NE Position':>40} {'Error':>12}")
    print("  " + "-"*100)

    for i in range(6):
        # T_{i,i-1} is from frame i-1 to frame i
        # To get T_{0,i}, we need to invert and chain
        T_inv = inverse_transform(state.transforms[i])
        T_base = T_base @ T_inv

        p_dh = dh_frames[i][:3, 3]
        p_ne = T_base[:3, 3]
        error = np.linalg.norm(p_dh - p_ne)

        print(f"  {i+1:>6} {str(p_dh):>40} {str(p_ne):>40} {error:>12.2e}")

    # Final tool0 position
    T_tool0_dh = forward_kinematics(q)
    print(f"\n  tool0 DH: {T_tool0_dh[:3, 3]}")
    print(f"  tool0 NE: {T_base[:3, 3]}")
    print(f"  Error:    {np.linalg.norm(T_tool0_dh[:3, 3] - T_base[:3, 3]):.2e}")


def test_rotation_consistency():
    """Verify rotation matrices match between DH and NE."""
    dynamics = create_ur5e_joint_frame_dynamics()

    q = np.zeros(6)

    print("\n" + "="*80)
    print("ROTATION VERIFICATION (q=0)")
    print("="*80)

    # Get DH rotation
    T_dh = forward_kinematics(q)
    R_dh = T_dh[:3, :3]

    # Get NE rotation
    state = dynamics._forward_iterations(q, np.zeros(6), np.zeros(6))
    T_base = np.eye(4)
    for T_i in state.transforms:
        T_inv = inverse_transform(T_i)
        T_base = T_base @ T_inv
    R_ne = T_base[:3, :3]

    print(f"\n  DH rotation:\n{R_dh}")
    print(f"\n  NE rotation:\n{R_ne}")
    print(f"\n  Frobenius norm of difference: {np.linalg.norm(R_dh - R_ne):.2e}")
    print(f"  Rotations match: {np.allclose(R_dh, R_ne)}")


def test_velocity_against_jacobian():
    """Verify that NE velocity matches geometric Jacobian."""
    dynamics = create_ur5e_joint_frame_dynamics()

    test_configs = [
        (np.zeros(6), np.array([0.1, 0, 0, 0, 0, 0])),
        (np.zeros(6), np.array([0, 0.1, 0, 0, 0, 0])),
        (np.zeros(6), np.array([0, 0, 0.1, 0, 0, 0])),
        (np.zeros(6), np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])),
        (np.array([0.5, -1.0, 0.5, -0.5, 0.5, 0.3]), np.array([0.1, 0.2, -0.1, 0.15, -0.05, 0.1])),
        (np.array([-0.3, -1.57, 1.2, -0.8, -0.5, 0.2]), np.array([0.05, 0.1, 0.2, 0.1, 0.15, 0.05])),
    ]

    print("\n" + "="*80)
    print("VELOCITY VERIFICATION (NE vs Jacobian)")
    print("="*80)

    all_pass = True
    for q, dq in test_configs:
        # Ground truth from Jacobian
        V_jacobian = compute_tool0_twist_tool(q, dq)  # [ω, v] in tool0 frame

        # Newton-Euler
        V_ne = dynamics.get_tool0_twist(q, dq)

        error = np.linalg.norm(V_jacobian - V_ne)
        status = "PASS" if error < 1e-10 else "FAIL"
        if error >= 1e-10:
            all_pass = False

        print(f"\n  q = {q}")
        print(f"  dq = {dq}")
        print(f"  Jacobian: {V_jacobian}")
        print(f"  NE:       {V_ne}")
        print(f"  Error:    {error:.2e} [{status}]")

    print("\n" + "="*80)
    print(f"  Overall: {'ALL TESTS PASSED' if all_pass else 'SOME TESTS FAILED'}")
    print("="*80)


def test_acceleration_numerical():
    """Verify acceleration by numerical differentiation of velocity."""
    dynamics = create_ur5e_joint_frame_dynamics()

    q = np.array([0.5, -1.0, 0.5, -0.5, 0.5, 0.3])
    dq = np.array([0.1, 0.2, -0.1, 0.15, -0.05, 0.1])
    ddq = np.array([0.05, -0.1, 0.08, -0.05, 0.1, -0.02])

    print("\n" + "="*80)
    print("ACCELERATION VERIFICATION (Numerical)")
    print("="*80)

    # Get acceleration from NE
    Vdot_ne = dynamics.get_tool0_acceleration(q, dq, ddq)

    # Numerical differentiation
    dt = 1e-7
    q_plus = q + dq * dt + 0.5 * ddq * dt**2
    dq_plus = dq + ddq * dt

    V_now = dynamics.get_tool0_twist(q, dq)
    V_plus = dynamics.get_tool0_twist(q_plus, dq_plus)

    Vdot_numerical = (V_plus - V_now) / dt

    # Note: NE includes gravity, numerical doesn't
    # For fair comparison, use zero gravity
    dynamics_no_gravity = create_ur5e_joint_frame_dynamics(gravity=np.zeros(3))
    Vdot_ne_no_grav = dynamics_no_gravity.get_tool0_acceleration(q, dq, ddq)

    V_now_ng = dynamics_no_gravity.get_tool0_twist(q, dq)
    V_plus_ng = dynamics_no_gravity.get_tool0_twist(q_plus, dq_plus)
    Vdot_numerical_ng = (V_plus_ng - V_now_ng) / dt

    error = np.linalg.norm(Vdot_ne_no_grav - Vdot_numerical_ng)

    print(f"\n  NE acceleration (no gravity): {Vdot_ne_no_grav}")
    print(f"  Numerical:                    {Vdot_numerical_ng}")
    print(f"  Error: {error:.2e}")
    print(f"  Status: {'PASS' if error < 1e-6 else 'FAIL'}")


def test_torque_comparison():
    """Compare torques between joint-frame NE and original CoM-based NE."""
    dynamics_jf = create_ur5e_joint_frame_dynamics()
    dynamics_com = create_ur5e_dynamics()

    test_configs = [
        (np.zeros(6), np.zeros(6), np.zeros(6)),
        (np.array([0.5, -1.0, 0.5, -0.5, 0.5, 0.3]), np.zeros(6), np.zeros(6)),
        (np.zeros(6), np.array([0.1, 0.2, -0.1, 0.15, -0.05, 0.1]), np.zeros(6)),
        (np.array([0.5, -1.0, 0.5, -0.5, 0.5, 0.3]),
         np.array([0.1, 0.2, -0.1, 0.15, -0.05, 0.1]),
         np.array([0.05, -0.1, 0.08, -0.05, 0.1, -0.02])),
    ]

    print("\n" + "="*80)
    print("TORQUE COMPARISON (Joint-Frame NE vs CoM-based NE)")
    print("="*80)

    for q, dq, ddq in test_configs:
        tau_jf = dynamics_jf.inverse_dynamics(q, dq, ddq)
        tau_com = dynamics_com.inverse_dynamics(q, dq, ddq)

        error = np.linalg.norm(tau_jf - tau_com)

        print(f"\n  q = {q}")
        print(f"  dq = {dq}")
        print(f"  ddq = {ddq}")
        print(f"  Joint-frame τ: {tau_jf}")
        print(f"  CoM-based τ:   {tau_com}")
        print(f"  Error: {error:.2e}")


if __name__ == "__main__":
    test_frame_positions()
    test_rotation_consistency()
    test_velocity_against_jacobian()
    test_acceleration_numerical()
    test_torque_comparison()
