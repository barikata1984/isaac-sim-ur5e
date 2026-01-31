"""Test CoM-based Newton-Euler tool0 velocity computation.

Verifies that the new get_tool0_twist() method produces correct results.
"""

import numpy as np

from dynamics.newton_euler import create_ur5e_dynamics
from dynamics.newton_euler_joint_frame import create_ur5e_joint_frame_dynamics
from dynamics.forward_kinematics import compute_tool0_twist_tool


def test_com_based_tool0_velocity():
    """Verify CoM-based NE produces correct tool0 velocity."""
    dynamics_com = create_ur5e_dynamics()
    dynamics_jf = create_ur5e_joint_frame_dynamics()

    test_configs = [
        (np.zeros(6), np.array([0.1, 0, 0, 0, 0, 0])),
        (np.zeros(6), np.array([0, 0.1, 0, 0, 0, 0])),
        (np.zeros(6), np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])),
        (np.array([0.5, -1.0, 0.5, -0.5, 0.5, 0.3]),
         np.array([0.1, 0.2, -0.1, 0.15, -0.05, 0.1])),
        (np.array([-0.3, -1.57, 1.2, -0.8, -0.5, 0.2]),
         np.array([0.05, 0.1, 0.2, 0.1, 0.15, 0.05])),
    ]

    print("\n" + "="*80)
    print("CoM-BASED TOOL0 VELOCITY VERIFICATION")
    print("="*80)

    all_pass = True
    for q, dq in test_configs:
        # Ground truth from geometric Jacobian
        V_jacobian = compute_tool0_twist_tool(q, dq)

        # Joint-frame based NE (already verified)
        V_jf = dynamics_jf.get_tool0_twist(q, dq)

        # CoM-based NE with new method
        V_com = dynamics_com.get_tool0_twist(q, dq)

        error_com = np.linalg.norm(V_jacobian - V_com)
        error_jf = np.linalg.norm(V_jacobian - V_jf)

        status = "PASS" if error_com < 1e-10 else "FAIL"
        if error_com >= 1e-10:
            all_pass = False

        print(f"\n  q = {q}")
        print(f"  dq = {dq}")
        print(f"  Jacobian:      {V_jacobian}")
        print(f"  Joint-frame:   {V_jf}")
        print(f"  CoM-based:     {V_com}")
        print(f"  Error (CoM):   {error_com:.2e} [{status}]")

    print("\n" + "="*80)
    print(f"  Overall: {'ALL TESTS PASSED' if all_pass else 'SOME TESTS FAILED'}")
    print("="*80)

    assert all_pass, "Some tests failed"


def test_com_based_tool0_acceleration():
    """Verify CoM-based NE produces correct tool0 acceleration."""
    dynamics_com = create_ur5e_dynamics(gravity=np.zeros(3))
    dynamics_jf = create_ur5e_joint_frame_dynamics(gravity=np.zeros(3))

    q = np.array([0.5, -1.0, 0.5, -0.5, 0.5, 0.3])
    dq = np.array([0.1, 0.2, -0.1, 0.15, -0.05, 0.1])
    ddq = np.array([0.05, -0.1, 0.08, -0.05, 0.1, -0.02])

    print("\n" + "="*80)
    print("CoM-BASED TOOL0 ACCELERATION VERIFICATION")
    print("="*80)

    # Get accelerations
    Vdot_jf = dynamics_jf.get_tool0_acceleration(q, dq, ddq)
    Vdot_com = dynamics_com.get_tool0_acceleration(q, dq, ddq)

    error = np.linalg.norm(Vdot_jf - Vdot_com)

    print(f"\n  Joint-frame:   {Vdot_jf}")
    print(f"  CoM-based:     {Vdot_com}")
    print(f"  Error:         {error:.2e}")
    print(f"  Status:        {'PASS' if error < 1e-6 else 'FAIL'}")

    print("\n" + "="*80)

    assert error < 1e-6, f"Acceleration error {error} exceeds threshold"


if __name__ == "__main__":
    test_com_based_tool0_velocity()
    test_com_based_tool0_acceleration()
