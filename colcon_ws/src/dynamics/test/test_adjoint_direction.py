"""Test to verify the adjoint transformation direction in Newton-Euler.

This test checks whether the transformation from link 6 CoM frame to tool0
frame is correctly applied.
"""

import numpy as np
import pytest
from pymlg import SE3

from dynamics.newton_euler import create_ur5e_dynamics
from dynamics.ur5e_parameters import UR5eParameters
from dynamics.forward_kinematics import (
    compute_tool0_twist_base,
    compute_tool0_twist_tool,
    forward_kinematics,
)






def test_adjoint_transformation_direction():
    """Test that adjoint transformation is applied in correct direction.

    Lynch & Park states that for two frames {a} and {b} on the same rigid body:
        V_b = [Ad_{T_{ba}}] V_a

    where T_{ba} means "frame a expressed in frame b coordinates".
    """
    dynamics = create_ur5e_dynamics()
    params = UR5eParameters()

    # Test configurations
    test_configs = [
        (np.zeros(6), np.array([0.1, 0, 0, 0, 0, 0])),
        (np.array([0.5, -1.0, 0.5, -0.5, 0.5, 0.3]), np.array([0.1, 0.2, -0.1, 0.15, -0.05, 0.1])),
        (np.array([-0.3, -1.57, 1.2, -0.8, -0.5, 0.2]), np.array([0.05, 0.1, 0.2, 0.1, 0.15, 0.05])),
    ]

    print("\n" + "="*80)
    print("ADJOINT TRANSFORMATION DIRECTION TEST")
    print("="*80)

    for q, dq in test_configs:
        print(f"\nConfiguration: q = {q}")
        print(f"              dq = {dq}")

        # 1. Get twist using current Newton-Euler implementation
        V_ne_current = dynamics.get_end_effector_twist(q, dq)

        # 2. Get twist using geometric Jacobian (ground truth)
        V_jacobian_tool = compute_tool0_twist_tool(q, dq)  # Returns [ω, v] in tool0 frame

        # 3. Try with inverse adjoint transformation
        # Access internal state to get V_n (last link CoM frame twist)
        ddq = np.zeros(6)
        state = dynamics._forward_iterations(q, dq, ddq)
        V_n = state.twists[-1]  # Twist in link 6 CoM frame

        # Current implementation uses: Ad_T_ee @ V_n
        # where _ee_frame = T_{CoM6, tool0} (tool0 position in CoM6 coordinates)
        T_CoM6_tool0 = dynamics._ee_frame

        # Alternative: use inverse transform
        T_tool0_CoM6 = SE3.inverse(T_CoM6_tool0)

        Ad_current = SE3.adjoint(T_CoM6_tool0)
        Ad_inverse = SE3.adjoint(T_tool0_CoM6)

        V_with_current = Ad_current @ V_n
        V_with_inverse = Ad_inverse @ V_n

        print(f"\n  V_n (link 6 CoM frame):      {V_n}")
        print(f"\n  Ground truth (Jacobian):     {V_jacobian_tool}")
        print(f"  Current NE (Ad_T_CoM6_tool0): {V_with_current}")
        print(f"  Inverse NE (Ad_T_tool0_CoM6): {V_with_inverse}")

        error_current = np.linalg.norm(V_with_current - V_jacobian_tool)
        error_inverse = np.linalg.norm(V_with_inverse - V_jacobian_tool)

        print(f"\n  Error with current transform: {error_current:.6e}")
        print(f"  Error with inverse transform: {error_inverse:.6e}")

        # Show which one matches better
        if error_current < error_inverse:
            print("  --> Current implementation is closer")
        elif error_inverse < error_current:
            print("  --> INVERSE transformation should be used!")
        else:
            print("  --> Both have same error (both wrong?)")

    print("\n" + "="*80)


def test_velocity_components_comparison():
    """Compare velocity components in detail to understand the mismatch."""
    dynamics = create_ur5e_dynamics()
    params = UR5eParameters()

    q = np.array([0.5, -1.0, 0.5, -0.5, 0.5, 0.3])
    dq = np.array([0.1, 0.2, -0.1, 0.15, -0.05, 0.1])

    print("\n" + "="*80)
    print("DETAILED VELOCITY COMPONENT COMPARISON")
    print("="*80)

    # Jacobian-based (ground truth)
    V_tool_jacobian = compute_tool0_twist_tool(q, dq)  # [ω, v] in tool0
    V_base_jacobian = compute_tool0_twist_base(q, dq)  # [v, ω] in base

    # Newton-Euler
    V_ne = dynamics.get_end_effector_twist(q, dq)  # [ω, v] in tool0 (claimed)

    # Get FK transform
    T_base_tool0 = forward_kinematics(q)
    R = T_base_tool0[:3, :3]
    p = T_base_tool0[:3, 3]

    print(f"\ntool0 position in base frame: {p}")
    print(f"\ntool0 rotation matrix:\n{R}")

    # Transform base velocity to tool0 for verification
    v_base = V_base_jacobian[:3]
    omega_base = V_base_jacobian[3:]

    omega_tool_from_base = R.T @ omega_base
    v_tool_from_base = R.T @ v_base

    print(f"\n--- Angular velocity ---")
    print(f"  Base frame (Jacobian): {omega_base}")
    print(f"  Tool frame (Jacobian): {V_tool_jacobian[:3]}")
    print(f"  Tool (from base):      {omega_tool_from_base}")
    print(f"  Newton-Euler:          {V_ne[:3]}")

    print(f"\n--- Linear velocity ---")
    print(f"  Base frame (Jacobian): {v_base}")
    print(f"  Tool frame (Jacobian): {V_tool_jacobian[3:]}")
    print(f"  Tool (from base):      {v_tool_from_base}")
    print(f"  Newton-Euler:          {V_ne[3:]}")

    # Errors
    omega_error = np.linalg.norm(V_ne[:3] - V_tool_jacobian[:3])
    v_error = np.linalg.norm(V_ne[3:] - V_tool_jacobian[3:])

    print(f"\n--- Errors ---")
    print(f"  Angular velocity error: {omega_error:.6e}")
    print(f"  Linear velocity error:  {v_error:.6e}")

    print("\n" + "="*80)


def test_ee_frame_definition():
    """Examine the end-effector frame definition."""
    params = UR5eParameters()

    print("\n" + "="*80)
    print("END-EFFECTOR FRAME DEFINITION")
    print("="*80)

    T_ee = params.get_end_effector_frame()

    print(f"\nCoM6 position: {params.link_com_positions[5]}")
    print(f"DH d6 parameter: {params.dh_params[5, 1]}")

    print(f"\n_ee_frame (T_CoM6_to_tool0):\n{T_ee}")

    # This should be pure translation along z
    p_ee = T_ee[:3, 3]
    R_ee = T_ee[:3, :3]

    print(f"\nTranslation: {p_ee}")
    print(f"Rotation (should be identity):\n{R_ee}")

    # The translation should be:
    # tool0 is at d6 = 0.0823 along z from joint 6
    # CoM6 is at com6 = [0, 0, -0.001159] from joint 6
    # So tool0 - CoM6 = [0, 0, 0.0823] - [0, 0, -0.001159] = [0, 0, 0.084459]
    expected_p = np.array([0, 0, params.dh_params[5, 1]]) - params.link_com_positions[5]
    print(f"\nExpected translation: {expected_p}")
    print(f"Match: {np.allclose(p_ee, expected_p)}")

    print("\n" + "="*80)


if __name__ == "__main__":
    test_ee_frame_definition()
    test_adjoint_transformation_direction()
    test_velocity_components_comparison()
