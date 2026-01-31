"""Unit tests for Newton-Euler inverse dynamics."""

import numpy as np
import pytest
from numpy.testing import assert_allclose

from dynamics.newton_euler import NewtonEulerDynamics, DynamicsState, create_ur5e_dynamics
from dynamics.ur5e_parameters import UR5eParameters


class TestNewtonEulerCreation:
    """Tests for NewtonEulerDynamics creation."""

    def test_default_creation(self) -> None:
        """Test creation with default parameters."""
        dynamics = NewtonEulerDynamics()
        assert dynamics.params.n_joints == 6
        assert_allclose(dynamics.gravity, [0, 0, -9.81])

    def test_custom_gravity(self) -> None:
        """Test creation with custom gravity."""
        gravity = np.array([0, 0, -10.0])
        dynamics = NewtonEulerDynamics(gravity=gravity)
        assert_allclose(dynamics.gravity, gravity)

    def test_factory_function(self) -> None:
        """Test factory function."""
        dynamics = create_ur5e_dynamics()
        assert isinstance(dynamics, NewtonEulerDynamics)


class TestGravityCompensation:
    """Tests for gravity compensation torques."""

    def test_zero_position(self) -> None:
        """Test gravity torques at zero configuration."""
        dynamics = create_ur5e_dynamics()
        q = np.zeros(6)

        tau_g = dynamics.gravity_torques(q)

        assert tau_g.shape == (6,)
        # Gravity torques should be non-zero in general
        assert not np.allclose(tau_g, 0)

    def test_gravity_torques_zero_gravity(self) -> None:
        """Test gravity torques are zero when gravity is zero."""
        dynamics = NewtonEulerDynamics(gravity=np.zeros(3))
        q = np.zeros(6)

        tau_g = dynamics.gravity_torques(q)

        assert_allclose(tau_g, np.zeros(6), atol=1e-10)

    def test_gravity_direction_effect(self) -> None:
        """Test that gravity direction affects torques correctly."""
        dynamics_down = NewtonEulerDynamics(gravity=np.array([0, 0, -9.81]))
        dynamics_up = NewtonEulerDynamics(gravity=np.array([0, 0, 9.81]))
        q = np.zeros(6)

        tau_down = dynamics_down.gravity_torques(q)
        tau_up = dynamics_up.gravity_torques(q)

        # Torques should be opposite in sign
        assert_allclose(tau_down, -tau_up, atol=1e-10)


class TestInverseDynamics:
    """Tests for inverse dynamics computation."""

    def test_stationary_equals_gravity(self) -> None:
        """Test that stationary robot has torques equal to gravity."""
        dynamics = create_ur5e_dynamics()
        q = np.array([0.1, -0.5, 0.3, -1.0, 0.5, 0.2])
        dq = np.zeros(6)
        ddq = np.zeros(6)

        tau = dynamics.inverse_dynamics(q, dq, ddq)
        tau_g = dynamics.gravity_torques(q)

        assert_allclose(tau, tau_g, atol=1e-10)

    def test_input_validation(self) -> None:
        """Test input size validation."""
        dynamics = create_ur5e_dynamics()

        with pytest.raises(ValueError):
            dynamics.inverse_dynamics(
                np.zeros(5),  # Wrong size
                np.zeros(6),
                np.zeros(6),
            )

    def test_pure_acceleration(self) -> None:
        """Test inverse dynamics with only acceleration (no velocity)."""
        dynamics = NewtonEulerDynamics(gravity=np.zeros(3))
        q = np.zeros(6)
        dq = np.zeros(6)
        ddq = np.array([1.0, 0, 0, 0, 0, 0])

        tau = dynamics.inverse_dynamics(q, dq, ddq)

        # Torques should be non-zero due to inertia
        assert not np.allclose(tau, 0)


class TestMassMatrix:
    """Tests for mass matrix computation."""

    def test_mass_matrix_symmetric(self) -> None:
        """Test that mass matrix is symmetric."""
        dynamics = create_ur5e_dynamics()
        q = np.array([0.1, -0.5, 0.3, -1.0, 0.5, 0.2])

        M = dynamics.mass_matrix(q)

        assert M.shape == (6, 6)
        assert_allclose(M, M.T, atol=1e-10)

    def test_mass_matrix_positive_definite(self) -> None:
        """Test that mass matrix is positive definite."""
        dynamics = create_ur5e_dynamics()
        q = np.zeros(6)

        M = dynamics.mass_matrix(q)

        # Check eigenvalues are positive
        eigenvalues = np.linalg.eigvalsh(M)
        assert np.all(eigenvalues > 0)

    def test_mass_matrix_zero_position(self) -> None:
        """Test mass matrix at zero configuration."""
        dynamics = create_ur5e_dynamics()
        q = np.zeros(6)

        M = dynamics.mass_matrix(q)

        # Diagonal elements should be positive
        assert np.all(np.diag(M) > 0)


class TestCoriolisVector:
    """Tests for Coriolis/centrifugal vector."""

    def test_coriolis_zero_velocity(self) -> None:
        """Test Coriolis vector is zero for zero velocity."""
        dynamics = create_ur5e_dynamics()
        q = np.array([0.1, -0.5, 0.3, -1.0, 0.5, 0.2])
        dq = np.zeros(6)

        c = dynamics.coriolis_vector(q, dq)

        assert_allclose(c, np.zeros(6), atol=1e-10)

    def test_coriolis_nonzero_velocity(self) -> None:
        """Test Coriolis vector with velocity."""
        dynamics = create_ur5e_dynamics()
        q = np.array([0.1, -0.5, 0.3, -1.0, 0.5, 0.2])
        dq = np.array([1.0, 0.5, -0.3, 0.2, -0.1, 0.4])

        c = dynamics.coriolis_vector(q, dq)

        # Should be non-zero in general
        assert not np.allclose(c, 0)


class TestEndEffectorKinematics:
    """Tests for end-effector velocity and acceleration."""

    def test_ee_twist_zero_velocity(self) -> None:
        """Test end-effector twist is zero for zero joint velocity."""
        dynamics = create_ur5e_dynamics()
        q = np.zeros(6)
        dq = np.zeros(6)

        V_ee = dynamics.get_end_effector_twist(q, dq)

        assert V_ee.shape == (6,)
        assert_allclose(V_ee, np.zeros(6), atol=1e-10)

    def test_ee_twist_single_joint(self) -> None:
        """Test end-effector twist with single joint velocity."""
        dynamics = create_ur5e_dynamics()
        q = np.zeros(6)
        dq = np.zeros(6)
        dq[0] = 1.0  # Only base joint rotating

        V_ee = dynamics.get_end_effector_twist(q, dq)

        # Should have non-zero angular velocity
        assert not np.allclose(V_ee[:3], 0)

    def test_ee_acceleration_zero(self) -> None:
        """Test end-effector acceleration at rest."""
        dynamics = NewtonEulerDynamics(gravity=np.zeros(3))
        q = np.zeros(6)
        dq = np.zeros(6)
        ddq = np.zeros(6)

        Vdot_ee = dynamics.get_end_effector_acceleration(q, dq, ddq)

        assert_allclose(Vdot_ee, np.zeros(6), atol=1e-10)


class TestDynamicsState:
    """Tests for DynamicsState dataclass."""

    def test_default_state(self) -> None:
        """Test default state initialization."""
        state = DynamicsState()
        assert state.twists == []
        assert state.twist_dots == []
        assert state.wrenches == []
        assert state.transforms == []


class TestInverseDynamicsFull:
    """Tests for full inverse dynamics with state output."""

    def test_full_output_structure(self) -> None:
        """Test that full output includes all states."""
        dynamics = create_ur5e_dynamics()
        q = np.zeros(6)
        dq = np.zeros(6)
        ddq = np.zeros(6)

        tau, state = dynamics.inverse_dynamics_full(q, dq, ddq)

        assert tau.shape == (6,)
        assert len(state.twists) == 6
        assert len(state.twist_dots) == 6
        assert len(state.wrenches) == 6
        assert len(state.transforms) == 6

    def test_full_matches_simple(self) -> None:
        """Test that full output matches simple output."""
        dynamics = create_ur5e_dynamics()
        q = np.array([0.1, -0.5, 0.3, -1.0, 0.5, 0.2])
        dq = np.array([0.5, -0.3, 0.2, -0.1, 0.4, -0.2])
        ddq = np.array([0.1, 0.2, -0.1, 0.3, -0.2, 0.1])

        tau_simple = dynamics.inverse_dynamics(q, dq, ddq)
        tau_full, _ = dynamics.inverse_dynamics_full(q, dq, ddq)

        assert_allclose(tau_simple, tau_full)


class TestExternalWrench:
    """Tests for external wrench at end-effector."""

    def test_zero_external_wrench(self) -> None:
        """Test with explicit zero external wrench."""
        dynamics = create_ur5e_dynamics()
        q = np.zeros(6)
        dq = np.zeros(6)
        ddq = np.zeros(6)

        tau_none = dynamics.inverse_dynamics(q, dq, ddq, F_tip=None)
        tau_zero = dynamics.inverse_dynamics(q, dq, ddq, F_tip=np.zeros(6))

        assert_allclose(tau_none, tau_zero, atol=1e-10)

    def test_nonzero_external_wrench(self) -> None:
        """Test that external wrench affects torques."""
        dynamics = create_ur5e_dynamics()
        q = np.zeros(6)
        dq = np.zeros(6)
        ddq = np.zeros(6)

        tau_no_wrench = dynamics.inverse_dynamics(q, dq, ddq)
        F_tip = np.array([0, 0, 0, 0, 0, 10.0])  # 10N force in z
        tau_with_wrench = dynamics.inverse_dynamics(q, dq, ddq, F_tip=F_tip)

        # Torques should be different
        assert not np.allclose(tau_no_wrench, tau_with_wrench)
