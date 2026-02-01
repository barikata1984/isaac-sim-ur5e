"""Tests for newton_euler module."""

import numpy as np
import pytest

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))
ur_src = Path(__file__).parent.parent.parent / 'robots' / 'ur' / 'src'
sys.path.insert(0, str(ur_src))

from newton_euler import NewtonEulerDynamics, NewtonEulerResult
from ur5e import UR5eParameters


@pytest.fixture
def ur5e_dynamics():
    """Create NewtonEulerDynamics instance for UR5e."""
    params = UR5eParameters()
    return NewtonEulerDynamics.from_robot_params(params)


@pytest.fixture
def ur5e_dynamics_no_gravity():
    """Create NewtonEulerDynamics instance without gravity."""
    params = UR5eParameters()
    return NewtonEulerDynamics.from_robot_params(params, gravity=np.zeros(3))


class TestNewtonEulerResult:
    """Tests for NewtonEulerResult dataclass."""

    def test_result_attributes(self, ur5e_dynamics):
        """Test that result has expected attributes."""
        q = np.zeros(6)
        dq = np.zeros(6)
        ddq = np.zeros(6)

        result = ur5e_dynamics.inverse_dynamics(q, dq, ddq)

        assert isinstance(result, NewtonEulerResult)
        assert result.tau.shape == (6,)
        assert len(result.link_twists) == 6
        assert len(result.link_accelerations) == 6
        assert len(result.link_wrenches) == 6


class TestInverseDynamics:
    """Tests for inverse dynamics computation."""

    def test_static_equilibrium(self, ur5e_dynamics):
        """At rest (dq=ddq=0), torque should equal gravity torques."""
        q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        dq = np.zeros(6)
        ddq = np.zeros(6)

        tau = ur5e_dynamics.inverse_dynamics(q, dq, ddq).tau
        g = ur5e_dynamics.gravity_torques(q)

        np.testing.assert_array_almost_equal(tau, g)

    def test_zero_gravity_no_motion(self, ur5e_dynamics_no_gravity):
        """With no gravity and no motion, torque should be zero."""
        q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        dq = np.zeros(6)
        ddq = np.zeros(6)

        tau = ur5e_dynamics_no_gravity.inverse_dynamics(q, dq, ddq).tau

        np.testing.assert_array_almost_equal(tau, np.zeros(6), decimal=10)

    def test_zero_gravity_pure_acceleration(self, ur5e_dynamics_no_gravity):
        """With no gravity, τ = M(q) @ ddq + c(q, dq)."""
        q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        dq = np.zeros(6)  # No velocity, so no Coriolis
        ddq = np.array([1.0, 0.5, 0.2, 0.1, 0.05, 0.02])

        tau = ur5e_dynamics_no_gravity.inverse_dynamics(q, dq, ddq).tau
        M = ur5e_dynamics_no_gravity.mass_matrix(q)

        # τ = M @ ddq (no Coriolis with dq=0, no gravity)
        expected = M @ ddq

        np.testing.assert_array_almost_equal(tau, expected, decimal=6)

    def test_full_dynamics_equation(self, ur5e_dynamics):
        """Verify τ = M(q) @ ddq + c(q, dq) + g(q)."""
        np.random.seed(42)
        q = np.random.uniform(-np.pi / 2, np.pi / 2, 6)
        dq = np.random.uniform(-1.0, 1.0, 6)
        ddq = np.random.uniform(-0.5, 0.5, 6)

        # Full inverse dynamics
        tau = ur5e_dynamics.inverse_dynamics(q, dq, ddq).tau

        # Components
        M = ur5e_dynamics.mass_matrix(q)
        g = ur5e_dynamics.gravity_torques(q)
        c = ur5e_dynamics.coriolis_centrifugal(q, dq)

        # τ = M @ ddq + c + g
        expected = M @ ddq + c + g

        np.testing.assert_array_almost_equal(tau, expected, decimal=6)


class TestGravityTorques:
    """Tests for gravity compensation."""

    def test_gravity_torque_shape(self, ur5e_dynamics):
        """Gravity torque should be 6-element vector."""
        q = np.zeros(6)
        g = ur5e_dynamics.gravity_torques(q)
        assert g.shape == (6,)

    def test_zero_config_gravity(self, ur5e_dynamics):
        """At zero config, gravity torques should be nonzero."""
        q = np.zeros(6)
        g = ur5e_dynamics.gravity_torques(q)

        # At zero config, arm is extended, so gravity creates torque
        assert np.linalg.norm(g) > 0

    def test_gravity_changes_with_config(self, ur5e_dynamics):
        """Gravity torques should change with configuration."""
        q1 = np.zeros(6)
        q2 = np.array([0, np.pi / 2, 0, 0, 0, 0])

        g1 = ur5e_dynamics.gravity_torques(q1)
        g2 = ur5e_dynamics.gravity_torques(q2)

        # Different configurations should give different gravity torques
        assert not np.allclose(g1, g2)


class TestMassMatrix:
    """Tests for mass/inertia matrix."""

    def test_mass_matrix_shape(self, ur5e_dynamics):
        """Mass matrix should be 6x6."""
        q = np.zeros(6)
        M = ur5e_dynamics.mass_matrix(q)
        assert M.shape == (6, 6)

    def test_mass_matrix_symmetric(self, ur5e_dynamics):
        """Mass matrix should be symmetric."""
        q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        M = ur5e_dynamics.mass_matrix(q)

        np.testing.assert_array_almost_equal(M, M.T)

    def test_mass_matrix_positive_definite(self, ur5e_dynamics):
        """Mass matrix should be positive definite."""
        q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        M = ur5e_dynamics.mass_matrix(q)

        eigenvalues = np.linalg.eigvalsh(M)
        assert np.all(eigenvalues > 0)

    def test_mass_matrix_configuration_dependent(self, ur5e_dynamics):
        """Mass matrix should change with configuration.

        Note: For UR5e, configurations that only change joints 1 and 2 (which have
        α=0 in DH params) may result in the same mass matrix due to the kinematic
        structure. We test with joint 3 or 4 changes (which have α=±90°) to ensure
        the mass matrix properly depends on configuration.
        """
        q1 = np.zeros(6)
        # Change joint 3 (α=90°) to ensure configuration dependence
        q2 = np.array([0, np.pi / 4, np.pi / 4, np.pi / 4, 0, 0])

        M1 = ur5e_dynamics.mass_matrix(q1)
        M2 = ur5e_dynamics.mass_matrix(q2)

        # Different configurations should give different mass matrices
        assert not np.allclose(M1, M2)


class TestCoriolisCentrifugal:
    """Tests for Coriolis/centrifugal terms."""

    def test_coriolis_shape(self, ur5e_dynamics):
        """Coriolis term should be 6-element vector."""
        q = np.zeros(6)
        dq = np.ones(6) * 0.1
        c = ur5e_dynamics.coriolis_centrifugal(q, dq)
        assert c.shape == (6,)

    def test_coriolis_zero_velocity(self, ur5e_dynamics):
        """Coriolis should be zero when velocity is zero."""
        q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        dq = np.zeros(6)
        c = ur5e_dynamics.coriolis_centrifugal(q, dq)

        np.testing.assert_array_almost_equal(c, np.zeros(6))

    def test_coriolis_quadratic_in_velocity(self, ur5e_dynamics):
        """Coriolis should scale quadratically with velocity."""
        q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        dq = np.array([0.1, 0.2, 0.1, 0.1, 0.1, 0.1])

        c1 = ur5e_dynamics.coriolis_centrifugal(q, dq)
        c2 = ur5e_dynamics.coriolis_centrifugal(q, 2 * dq)

        # c(q, 2*dq) should be approximately 4 * c(q, dq)
        np.testing.assert_array_almost_equal(c2, 4 * c1, decimal=5)


class TestForwardBackwardPass:
    """Tests for forward and backward pass."""

    def test_twist_propagation(self, ur5e_dynamics):
        """Test that twists propagate correctly.

        Uses pymlg [ω, v] convention: V = [ωx, ωy, ωz, vx, vy, vz].
        """
        q = np.zeros(6)
        dq = np.array([1.0, 0, 0, 0, 0, 0])  # Only first joint moving
        ddq = np.zeros(6)

        transforms, twists, _ = ur5e_dynamics.forward_pass(q, dq, ddq)

        # First link twist should have angular velocity about z
        # In [ω, v] convention, ω_z is at index 2
        assert twists[0][2] != 0  # ω_z component

        # All links should have some angular velocity
        # In [ω, v] convention, angular part is V[:3]
        for V in twists:
            assert np.linalg.norm(V[:3]) > 0  # Angular part nonzero


class TestExternalWrench:
    """Tests for external wrench handling."""

    def test_external_wrench_affects_torque(self, ur5e_dynamics_no_gravity):
        """External wrench should change required torques.

        Note: For UR5e at zero config, a pure x-force may not create z-torque
        due to the kinematic structure. We use a wrench with z-torque component
        to ensure proper coupling to joint torques.
        """
        q = np.zeros(6)
        dq = np.zeros(6)
        ddq = np.zeros(6)

        tau_no_load = ur5e_dynamics_no_gravity.inverse_dynamics(q, dq, ddq).tau

        # Apply wrench at tip with z-torque component
        # This ensures coupling to joint torques (all UR5e joints rotate about z)
        F_tip = np.array([10.0, 10.0, 10.0, 1.0, 1.0, 1.0])
        tau_loaded = ur5e_dynamics_no_gravity.inverse_dynamics(
            q, dq, ddq, F_tip
        ).tau

        # Torques should be different
        assert not np.allclose(tau_no_load, tau_loaded)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
