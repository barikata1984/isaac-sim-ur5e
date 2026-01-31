"""Verify torque calculations between implementations.

Compares Joint-frame NE and CoM-based NE torque outputs.
"""

import numpy as np

from dynamics.newton_euler import create_ur5e_dynamics
from dynamics.newton_euler_joint_frame import create_ur5e_joint_frame_dynamics


def test_gravity_torques_at_home():
    """Analyze gravity torques at home position (q=0)."""
    dynamics_com = create_ur5e_dynamics()
    dynamics_jf = create_ur5e_joint_frame_dynamics()

    q = np.zeros(6)

    print("\n" + "="*80)
    print("GRAVITY TORQUES AT HOME POSITION (q=0)")
    print("="*80)

    tau_com = dynamics_com.gravity_torques(q)
    tau_jf = dynamics_jf.gravity_torques(q)

    print(f"\n  CoM-based:    {tau_com}")
    print(f"  Joint-frame:  {tau_jf}")
    print(f"  Difference:   {tau_com - tau_jf}")

    # At q=0, the robot arm is stretched out horizontally
    # Joint 1 rotates about vertical z-axis, so gravity shouldn't affect it
    # Joint 2 supports most of the arm weight
    print("\n  Physical analysis at q=0:")
    print("  - Joint 1: rotates about vertical z, gravity torque should be ~0")
    print("  - Joint 2: supports upper arm + forearm + wrist, large torque expected")
    print("  - Joint 3: supports forearm + wrist, medium torque expected")
    print("  - Joints 4-6: wrist joints, small torques expected")


def test_gravity_torques_various_configs():
    """Test gravity torques at various configurations."""
    dynamics_com = create_ur5e_dynamics()
    dynamics_jf = create_ur5e_joint_frame_dynamics()

    test_configs = [
        np.zeros(6),
        np.array([0, -np.pi/2, 0, 0, 0, 0]),  # Joint 2 at -90 deg (arm pointing up)
        np.array([0, -np.pi/2, np.pi/2, 0, 0, 0]),  # Elbow bent
        np.array([np.pi/4, -np.pi/3, np.pi/4, -np.pi/6, np.pi/4, 0]),
    ]

    print("\n" + "="*80)
    print("GRAVITY TORQUES AT VARIOUS CONFIGURATIONS")
    print("="*80)

    for i, q in enumerate(test_configs):
        tau_com = dynamics_com.gravity_torques(q)
        tau_jf = dynamics_jf.gravity_torques(q)

        print(f"\n  Config {i+1}: q = {np.round(q, 3)}")
        print(f"  CoM-based:    {np.round(tau_com, 4)}")
        print(f"  Joint-frame:  {np.round(tau_jf, 4)}")

        # Check if joint 1 torque is reasonable (should be ~0 for gravity)
        # Since joint 1 rotates about z (vertical), gravity shouldn't create torque
        if abs(tau_com[0]) > 1.0:
            print(f"  WARNING: CoM-based joint 1 torque is large ({tau_com[0]:.2f} N·m)")
        if abs(tau_jf[0]) > 1.0:
            print(f"  WARNING: Joint-frame joint 1 torque is large ({tau_jf[0]:.2f} N·m)")


def test_mass_matrix_comparison():
    """Compare mass matrices from both implementations."""
    dynamics_com = create_ur5e_dynamics()
    dynamics_jf = create_ur5e_joint_frame_dynamics()

    q = np.zeros(6)

    print("\n" + "="*80)
    print("MASS MATRIX COMPARISON (q=0)")
    print("="*80)

    M_com = dynamics_com.mass_matrix(q)
    M_jf = dynamics_jf.mass_matrix(q)

    print(f"\n  CoM-based diagonal:    {np.diag(M_com)}")
    print(f"  Joint-frame diagonal:  {np.diag(M_jf)}")
    print(f"  Frobenius norm diff:   {np.linalg.norm(M_com - M_jf):.4f}")

    # Check symmetry
    print(f"\n  CoM-based symmetric:   {np.allclose(M_com, M_com.T)}")
    print(f"  Joint-frame symmetric: {np.allclose(M_jf, M_jf.T)}")

    # Check positive definiteness
    eigvals_com = np.linalg.eigvalsh(M_com)
    eigvals_jf = np.linalg.eigvalsh(M_jf)
    print(f"\n  CoM-based min eigenvalue:    {eigvals_com.min():.6f}")
    print(f"  Joint-frame min eigenvalue:  {eigvals_jf.min():.6f}")


def test_coriolis_comparison():
    """Compare Coriolis vectors from both implementations."""
    dynamics_com = create_ur5e_dynamics()
    dynamics_jf = create_ur5e_joint_frame_dynamics()

    q = np.array([0.5, -1.0, 0.5, -0.5, 0.5, 0.3])
    dq = np.array([0.1, 0.2, -0.1, 0.15, -0.05, 0.1])

    print("\n" + "="*80)
    print("CORIOLIS VECTOR COMPARISON")
    print("="*80)

    c_com = dynamics_com.coriolis_vector(q, dq)
    c_jf = dynamics_jf.coriolis_vector(q, dq)

    print(f"\n  q =  {q}")
    print(f"  dq = {dq}")
    print(f"\n  CoM-based:    {np.round(c_com, 6)}")
    print(f"  Joint-frame:  {np.round(c_jf, 6)}")
    print(f"  Difference:   {np.round(c_com - c_jf, 6)}")


if __name__ == "__main__":
    test_gravity_torques_at_home()
    test_gravity_torques_various_configs()
    test_mass_matrix_comparison()
    test_coriolis_comparison()
