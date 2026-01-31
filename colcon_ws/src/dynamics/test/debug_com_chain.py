"""Debug CoM frame chain computation."""

import numpy as np
from pymlg import SE3

from dynamics.newton_euler import create_ur5e_dynamics
from dynamics.forward_kinematics import forward_kinematics



def debug_com_chain():
    """Debug the CoM frame chain computation."""
    dynamics = create_ur5e_dynamics()

    q = np.zeros(6)
    dq = np.array([0.1, 0, 0, 0, 0, 0])

    print("\n" + "="*80)
    print("DEBUG: CoM FRAME CHAIN")
    print("="*80)

    state = dynamics._forward_iterations(q, dq, np.zeros(6))

    # Compute T_base_com6 using the internal method
    T_base_com6 = dynamics._compute_base_to_com6(state)
    print(f"\nT_base_com6 from internal method:")
    print(f"  Position: {T_base_com6[:3, 3]}")
    print(f"  Rotation:\n{T_base_com6[:3, :3]}")

    # Compare with standard DH tool0
    T_base_tool0 = forward_kinematics(q)
    print(f"\nT_base_tool0 from forward_kinematics:")
    print(f"  Position: {T_base_tool0[:3, 3]}")
    print(f"  Rotation:\n{T_base_tool0[:3, :3]}")

    # Show the difference
    print(f"\nPosition difference: {T_base_tool0[:3, 3] - T_base_com6[:3, 3]}")
    print(f"Rotation difference norm: {np.linalg.norm(T_base_tool0[:3, :3] - T_base_com6[:3, :3])}")

    # Get twist in CoM6 frame
    V_com6 = state.twists[-1]
    print(f"\nV_com6 (twist in CoM6 frame): {V_com6}")
    print(f"  omega: {V_com6[:3]}")
    print(f"  v: {V_com6[3:]}")

    # Transform omega to base frame using CoM6 rotation
    R_com6 = T_base_com6[:3, :3]
    omega_base_from_com = R_com6 @ V_com6[:3]
    print(f"\nomega in base frame (from CoM): {omega_base_from_com}")

    # What should omega be?
    # Joint 1 rotates about base z-axis with dq[0] = 0.1
    print(f"Expected omega in base frame: [0, 0, 0.1]")

    # So the issue is that R_com6 is not the same as the rotation that
    # should be applied to transform omega from CoM6 to base


if __name__ == "__main__":
    debug_com_chain()
