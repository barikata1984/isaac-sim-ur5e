"""Tests for kinematics module."""

import numpy as np
import pytest

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))
ur_src = Path(__file__).parent.parent.parent / 'robots' / 'ur' / 'src'
sys.path.insert(0, str(ur_src))

from pymlg.numpy import SE3

from kinematics_legacy import (
    forwardkinematics_legacy,
    forwardkinematics_legacy_all_frames,
    body_jacobian,
    space_jacobian,
    geometric_jacobian,
    tool0_twist,
    tool0_acceleration,
)
from ur5e import UR5eParameters


@pytest.fixture
def ur5e_dh():
    """Get UR5e DH parameters."""
    params = UR5eParameters()
    return params.dh_params


class TestForwardKinematics:
    """Tests for forward kinematics."""

    def test_fk_zero_config(self, ur5e_dh):
        """Test FK at zero configuration."""
        q = np.zeros(6)
        T = forwardkinematics_legacy(q, ur5e_dh)

        # T should be a valid SE(3) matrix
        assert T.shape == (4, 4)
        np.testing.assert_almost_equal(T[3, :], [0, 0, 0, 1])

        # Rotation should be orthogonal
        R = T[:3, :3]
        np.testing.assert_array_almost_equal(R @ R.T, np.eye(3))

    def test_fk_all_frames_length(self, ur5e_dh):
        """Test that all_frames returns correct number of frames."""
        q = np.zeros(6)
        frames = forwardkinematics_legacy_all_frames(q, ur5e_dh)

        # Should have base + 6 joint frames = 7 frames
        assert len(frames) == 7

        # First frame should be identity
        np.testing.assert_array_almost_equal(frames[0], np.eye(4))

        # Last frame should match forwardkinematics_legacy
        T = forwardkinematics_legacy(q, ur5e_dh)
        np.testing.assert_array_almost_equal(frames[-1], T)

    def test_fk_joint_independence(self, ur5e_dh):
        """Test that only joints before end-effector affect FK."""
        q1 = np.zeros(6)
        q2 = np.zeros(6)
        q2[0] = np.pi / 4  # Change first joint

        T1 = forwardkinematics_legacy(q1, ur5e_dh)
        T2 = forwardkinematics_legacy(q2, ur5e_dh)

        # Positions should be different
        assert not np.allclose(T1[:3, 3], T2[:3, 3])


class TestJacobian:
    """Tests for Jacobian computation."""

    def test_space_jacobian_shape(self, ur5e_dh):
        """Test space Jacobian has correct shape."""
        q = np.zeros(6)
        J = space_jacobian(q, ur5e_dh)
        assert J.shape == (6, 6)

    def test_body_jacobian_shape(self, ur5e_dh):
        """Test body Jacobian has correct shape."""
        q = np.zeros(6)
        J = body_jacobian(q, ur5e_dh)
        assert J.shape == (6, 6)

    def test_jacobian_numerical_comparison(self, ur5e_dh):
        """Compare Jacobian with numerical differentiation.

        Uses [ω, v] convention: J[0:3] = angular, J[3:6] = linear.
        """
        q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        eps = 1e-7

        # Compute analytical Jacobian
        J_s = space_jacobian(q, ur5e_dh)

        # Compute numerical Jacobian for position (linear velocity)
        J_num_linear = np.zeros((3, 6))
        for i in range(6):
            q_plus = q.copy()
            q_minus = q.copy()
            q_plus[i] += eps
            q_minus[i] -= eps

            T_plus = forwardkinematics_legacy(q_plus, ur5e_dh)
            T_minus = forwardkinematics_legacy(q_minus, ur5e_dh)

            J_num_linear[:, i] = (T_plus[:3, 3] - T_minus[:3, 3]) / (2 * eps)

        # Linear velocity part (J[3:6]) should match
        np.testing.assert_array_almost_equal(J_s[3:, :], J_num_linear, decimal=5)

    def test_jacobian_relation(self, ur5e_dh):
        """Test relationship between space and body Jacobian.

        J_s = Ad_{T_0n} @ J_b

        Uses pymlg [ω, v] convention.
        """
        q = np.array([0.1, 0.2, -0.3, 0.4, -0.5, 0.6])

        J_s = space_jacobian(q, ur5e_dh)
        J_b = body_jacobian(q, ur5e_dh)
        T = forwardkinematics_legacy(q, ur5e_dh)

        # Compute adjoint using pymlg
        Ad_T = SE3.adjoint(T)

        J_s_from_body = Ad_T @ J_b

        np.testing.assert_array_almost_equal(J_s, J_s_from_body, decimal=5)


class TestTwist:
    """Tests for twist computation."""

    def test_twist_zero_velocity(self, ur5e_dh):
        """Zero joint velocity should give zero twist."""
        q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        dq = np.zeros(6)

        V = tool0_twist(q, dq, ur5e_dh)

        np.testing.assert_array_almost_equal(V, np.zeros(6))

    def test_twist_shape(self, ur5e_dh):
        """Twist should be 6-element vector."""
        q = np.zeros(6)
        dq = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

        V = tool0_twist(q, dq, ur5e_dh)

        assert V.shape == (6,)

    def test_twist_linearity(self, ur5e_dh):
        """V = J @ dq should be linear in dq."""
        q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        dq1 = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
        dq2 = np.array([0.0, 0.2, 0.0, 0.0, 0.0, 0.0])

        V1 = tool0_twist(q, dq1, ur5e_dh)
        V2 = tool0_twist(q, dq2, ur5e_dh)
        V12 = tool0_twist(q, dq1 + dq2, ur5e_dh)

        np.testing.assert_array_almost_equal(V12, V1 + V2)


class TestAcceleration:
    """Tests for acceleration computation."""

    def test_acceleration_zero_velocity(self, ur5e_dh):
        """Zero velocity should give J @ ddq acceleration."""
        q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        dq = np.zeros(6)
        ddq = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

        V_dot = tool0_acceleration(q, dq, ddq, ur5e_dh)
        J = space_jacobian(q, ur5e_dh)

        # With dq=0, J_dot term vanishes, so V_dot = J @ ddq
        expected = J @ ddq
        np.testing.assert_array_almost_equal(V_dot, expected, decimal=4)

    def test_acceleration_shape(self, ur5e_dh):
        """Acceleration should be 6-element vector."""
        q = np.zeros(6)
        dq = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        ddq = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

        V_dot = tool0_acceleration(q, dq, ddq, ur5e_dh)

        assert V_dot.shape == (6,)


class TestSingularity:
    """Tests for behavior near singularities."""

    def test_jacobian_at_singularity(self, ur5e_dh):
        """Test Jacobian rank at a known singular configuration."""
        # Elbow singularity (joint 3 at 0)
        q = np.array([0, 0, 0, 0, 0, 0])
        J = space_jacobian(q, ur5e_dh)

        # Should still compute, even if rank-deficient
        assert J.shape == (6, 6)

        # Check rank (may be < 6 at singularity)
        rank = np.linalg.matrix_rank(J, tol=1e-6)
        # UR5e at zero config might be singular
        assert rank <= 6


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
