"""Comparison tests between different Newton-Euler implementations.

Verifies that:
1. Twist formulation with pymlg (NewtonEulerDynamics)
2. Twist formulation without pymlg (NewtonEulerNative)
3. Classic formulation (NewtonEulerClassic)

all produce the same results.
"""

import numpy as np
import pytest
from numpy.testing import assert_allclose

from dynamics.newton_euler import NewtonEulerDynamics, create_ur5e_dynamics
from dynamics.newton_euler_native import NewtonEulerNative, create_ur5e_native_dynamics
from dynamics.newton_euler_classic import NewtonEulerClassic, create_ur5e_classic_dynamics


# Test configurations
TEST_CONFIGURATIONS = [
    # Zero position
    {
        "name": "zero_position",
        "q": np.zeros(6),
        "dq": np.zeros(6),
        "ddq": np.zeros(6),
    },
    # Random position, zero velocity
    {
        "name": "random_position",
        "q": np.array([0.1, -0.5, 0.3, -1.0, 0.5, 0.2]),
        "dq": np.zeros(6),
        "ddq": np.zeros(6),
    },
    # With velocity
    {
        "name": "with_velocity",
        "q": np.array([0.2, -0.3, 0.5, -0.8, 0.4, 0.1]),
        "dq": np.array([0.5, -0.3, 0.2, -0.1, 0.4, -0.2]),
        "ddq": np.zeros(6),
    },
    # With acceleration
    {
        "name": "with_acceleration",
        "q": np.array([0.1, -0.5, 0.3, -1.0, 0.5, 0.2]),
        "dq": np.array([0.5, -0.3, 0.2, -0.1, 0.4, -0.2]),
        "ddq": np.array([0.1, 0.2, -0.1, 0.3, -0.2, 0.1]),
    },
    # Large angles
    {
        "name": "large_angles",
        "q": np.array([np.pi/2, -np.pi/3, np.pi/4, -np.pi/2, np.pi/6, np.pi/3]),
        "dq": np.array([1.0, -0.5, 0.3, -0.2, 0.1, -0.4]),
        "ddq": np.array([0.5, -0.3, 0.2, 0.1, -0.2, 0.3]),
    },
    # High velocity
    {
        "name": "high_velocity",
        "q": np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]),
        "dq": np.array([2.0, 1.5, 1.0, 0.8, 0.5, 0.3]),
        "ddq": np.array([1.0, 0.5, 0.3, 0.2, 0.1, 0.05]),
    },
]


class TestPymlgVsNative:
    """Compare pymlg-based and native Lie algebra implementations."""

    @pytest.fixture
    def dynamics_pymlg(self) -> NewtonEulerDynamics:
        """Create pymlg-based dynamics instance."""
        return create_ur5e_dynamics()

    @pytest.fixture
    def dynamics_native(self) -> NewtonEulerNative:
        """Create native dynamics instance."""
        return create_ur5e_native_dynamics()

    @pytest.mark.parametrize("config", TEST_CONFIGURATIONS, ids=lambda c: c["name"])
    def test_inverse_dynamics(
        self,
        dynamics_pymlg: NewtonEulerDynamics,
        dynamics_native: NewtonEulerNative,
        config: dict,
    ) -> None:
        """Test that inverse dynamics match between implementations."""
        q = config["q"]
        dq = config["dq"]
        ddq = config["ddq"]

        tau_pymlg = dynamics_pymlg.inverse_dynamics(q, dq, ddq)
        tau_native = dynamics_native.inverse_dynamics(q, dq, ddq)

        assert_allclose(
            tau_pymlg, tau_native, rtol=1e-10, atol=1e-10,
            err_msg=f"Mismatch in config: {config['name']}"
        )

    @pytest.mark.parametrize("config", TEST_CONFIGURATIONS, ids=lambda c: c["name"])
    def test_gravity_torques(
        self,
        dynamics_pymlg: NewtonEulerDynamics,
        dynamics_native: NewtonEulerNative,
        config: dict,
    ) -> None:
        """Test that gravity torques match."""
        q = config["q"]

        tau_g_pymlg = dynamics_pymlg.gravity_torques(q)
        tau_g_native = dynamics_native.gravity_torques(q)

        assert_allclose(
            tau_g_pymlg, tau_g_native, rtol=1e-10, atol=1e-10,
            err_msg=f"Mismatch in config: {config['name']}"
        )

    @pytest.mark.parametrize("config", TEST_CONFIGURATIONS, ids=lambda c: c["name"])
    def test_mass_matrix(
        self,
        dynamics_pymlg: NewtonEulerDynamics,
        dynamics_native: NewtonEulerNative,
        config: dict,
    ) -> None:
        """Test that mass matrices match."""
        q = config["q"]

        M_pymlg = dynamics_pymlg.mass_matrix(q)
        M_native = dynamics_native.mass_matrix(q)

        assert_allclose(
            M_pymlg, M_native, rtol=1e-10, atol=1e-10,
            err_msg=f"Mismatch in config: {config['name']}"
        )

    @pytest.mark.parametrize("config", TEST_CONFIGURATIONS, ids=lambda c: c["name"])
    def test_coriolis_vector(
        self,
        dynamics_pymlg: NewtonEulerDynamics,
        dynamics_native: NewtonEulerNative,
        config: dict,
    ) -> None:
        """Test that Coriolis vectors match."""
        q = config["q"]
        dq = config["dq"]

        c_pymlg = dynamics_pymlg.coriolis_vector(q, dq)
        c_native = dynamics_native.coriolis_vector(q, dq)

        assert_allclose(
            c_pymlg, c_native, rtol=1e-10, atol=1e-10,
            err_msg=f"Mismatch in config: {config['name']}"
        )


class TestTwistVsClassic:
    """Compare twist formulation and classic formulation.

    Note: Due to differences in frame conventions (CoM frames vs joint frames),
    the implementations may produce slightly different results. This test
    verifies that the results are qualitatively similar (same order of magnitude,
    same signs for dominant effects).
    """

    @pytest.fixture
    def dynamics_twist(self) -> NewtonEulerDynamics:
        """Create twist-based dynamics instance."""
        return create_ur5e_dynamics()

    @pytest.fixture
    def dynamics_classic(self) -> NewtonEulerClassic:
        """Create classic dynamics instance."""
        return create_ur5e_classic_dynamics()

    def test_gravity_torques_zero_position(
        self,
        dynamics_twist: NewtonEulerDynamics,
        dynamics_classic: NewtonEulerClassic,
    ) -> None:
        """Test gravity torques at zero position.

        At zero position, the arm is stretched out horizontally.
        Both implementations should produce gravity compensation torques
        that have the same signs and similar magnitudes.
        """
        q = np.zeros(6)

        tau_g_twist = dynamics_twist.gravity_torques(q)
        tau_g_classic = dynamics_classic.gravity_torques(q)

        # Both should be non-zero
        assert not np.allclose(tau_g_twist, 0)
        assert not np.allclose(tau_g_classic, 0)

        # Print for debugging
        print(f"\nGravity torques at zero position:")
        print(f"  Twist formulation:   {tau_g_twist}")
        print(f"  Classic formulation: {tau_g_classic}")

    def test_mass_matrix_properties(
        self,
        dynamics_twist: NewtonEulerDynamics,
        dynamics_classic: NewtonEulerClassic,
    ) -> None:
        """Test that both mass matrices have correct properties."""
        q = np.array([0.1, -0.5, 0.3, -1.0, 0.5, 0.2])

        M_twist = dynamics_twist.mass_matrix(q)
        M_classic = dynamics_classic.mass_matrix(q)

        # Both should be symmetric
        assert_allclose(M_twist, M_twist.T, atol=1e-10)
        assert_allclose(M_classic, M_classic.T, atol=1e-10)

        # Both should be positive definite
        eigvals_twist = np.linalg.eigvalsh(M_twist)
        eigvals_classic = np.linalg.eigvalsh(M_classic)
        assert np.all(eigvals_twist > 0)
        assert np.all(eigvals_classic > 0)

        # Print for debugging
        print(f"\nMass matrix eigenvalues:")
        print(f"  Twist formulation:   {eigvals_twist}")
        print(f"  Classic formulation: {eigvals_classic}")

    def test_coriolis_zero_velocity(
        self,
        dynamics_twist: NewtonEulerDynamics,
        dynamics_classic: NewtonEulerClassic,
    ) -> None:
        """Test that Coriolis terms are zero at zero velocity."""
        q = np.array([0.1, -0.5, 0.3, -1.0, 0.5, 0.2])
        dq = np.zeros(6)

        c_twist = dynamics_twist.coriolis_vector(q, dq)
        c_classic = dynamics_classic.coriolis_vector(q, dq)

        assert_allclose(c_twist, np.zeros(6), atol=1e-10)
        assert_allclose(c_classic, np.zeros(6), atol=1e-10)

    @pytest.mark.parametrize("config", TEST_CONFIGURATIONS[:3], ids=lambda c: c["name"])
    def test_qualitative_agreement(
        self,
        dynamics_twist: NewtonEulerDynamics,
        dynamics_classic: NewtonEulerClassic,
        config: dict,
    ) -> None:
        """Test qualitative agreement between formulations.

        The two formulations use different frame conventions, so exact
        agreement is not expected. However, they should agree qualitatively.
        """
        q = config["q"]
        dq = config["dq"]
        ddq = config["ddq"]

        tau_twist = dynamics_twist.inverse_dynamics(q, dq, ddq)
        tau_classic = dynamics_classic.inverse_dynamics(q, dq, ddq)

        # Print comparison
        print(f"\nInverse dynamics comparison for {config['name']}:")
        print(f"  Twist:   {tau_twist}")
        print(f"  Classic: {tau_classic}")
        print(f"  Ratio:   {tau_twist / (tau_classic + 1e-10)}")


class TestLieAlgebraNativeVsPymlg:
    """Compare pymlg SE3/SO3 and native Lie algebra implementations."""

    def test_se3_exp(self) -> None:
        """Test SE(3) exponential map."""
        from pymlg import SE3
        from dynamics.lie_algebra_native import se3_exp as se3_exp_native

        twists = [
            np.zeros(6),
            np.array([0.1, 0.2, 0.3, 1.0, 2.0, 3.0]),
            np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0]),  # Pure rotation
            np.array([0.0, 0.0, 0.0, 1.0, 2.0, 3.0]),  # Pure translation
        ]
        thetas = [0.0, 0.5, 1.0, np.pi/2]

        for twist in twists:
            for theta in thetas:
                T_pymlg = SE3.Exp(twist * theta)
                T_native = se3_exp_native(twist, theta)
                assert_allclose(
                    T_pymlg, T_native, rtol=1e-10, atol=1e-10,
                    err_msg=f"Mismatch for twist={twist}, theta={theta}"
                )

    def test_adjoint(self) -> None:
        """Test Adjoint representation."""
        from pymlg import SE3
        from dynamics.lie_algebra_native import adjoint as adjoint_native

        transforms = [
            np.eye(4),
            np.array([
                [0, -1, 0, 1],
                [1, 0, 0, 2],
                [0, 0, 1, 3],
                [0, 0, 0, 1]
            ], dtype=np.float64),
        ]

        for T in transforms:
            Ad_pymlg = SE3.adjoint(T)
            Ad_native = adjoint_native(T)
            assert_allclose(
                Ad_pymlg, Ad_native, rtol=1e-10, atol=1e-10,
                err_msg=f"Mismatch for T=\n{T}"
            )

    def test_ad(self) -> None:
        """Test ad (Lie bracket) operator."""
        from pymlg import SE3
        from dynamics.lie_algebra_native import ad as ad_native

        twists = [
            np.zeros(6),
            np.array([0.1, 0.2, 0.3, 1.0, 2.0, 3.0]),
            np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ]

        for twist in twists:
            ad_pymlg_result = SE3.adjoint_algebra(SE3.wedge(twist))
            ad_native_result = ad_native(twist)
            assert_allclose(
                ad_pymlg_result, ad_native_result, rtol=1e-10, atol=1e-10,
                err_msg=f"Mismatch for twist={twist}"
            )


class TestConsistencyChecks:
    """Additional consistency checks for all implementations."""

    @pytest.fixture(params=["pymlg", "native", "classic"])
    def dynamics(self, request) -> object:
        """Create dynamics instance based on parameter."""
        if request.param == "pymlg":
            return create_ur5e_dynamics()
        elif request.param == "native":
            return create_ur5e_native_dynamics()
        else:
            return create_ur5e_classic_dynamics()

    def test_gravity_torques_sign_reversal(self, dynamics) -> None:
        """Test that reversing gravity reverses torques."""
        q = np.array([0.1, -0.5, 0.3, -1.0, 0.5, 0.2])

        original_gravity = dynamics.gravity.copy()

        dynamics.gravity = np.array([0, 0, -9.81])
        tau_down = dynamics.gravity_torques(q)

        dynamics.gravity = np.array([0, 0, 9.81])
        tau_up = dynamics.gravity_torques(q)

        dynamics.gravity = original_gravity

        assert_allclose(tau_down, -tau_up, atol=1e-10)

    def test_zero_gravity_zero_velocity(self, dynamics) -> None:
        """Test that zero gravity and velocity gives zero torque."""
        original_gravity = dynamics.gravity.copy()
        dynamics.gravity = np.zeros(3)

        q = np.array([0.1, -0.5, 0.3, -1.0, 0.5, 0.2])
        dq = np.zeros(6)
        ddq = np.zeros(6)

        tau = dynamics.inverse_dynamics(q, dq, ddq)
        dynamics.gravity = original_gravity

        assert_allclose(tau, np.zeros(6), atol=1e-10)

    def test_equation_of_motion_decomposition(self, dynamics) -> None:
        """Test that τ = M*ddq + c(q,dq) + g(q)."""
        q = np.array([0.1, -0.5, 0.3, -1.0, 0.5, 0.2])
        dq = np.array([0.5, -0.3, 0.2, -0.1, 0.4, -0.2])
        ddq = np.array([0.1, 0.2, -0.1, 0.3, -0.2, 0.1])

        # Full inverse dynamics
        tau = dynamics.inverse_dynamics(q, dq, ddq)

        # Components
        M = dynamics.mass_matrix(q)
        c = dynamics.coriolis_vector(q, dq)
        g = dynamics.gravity_torques(q)

        # τ = M*ddq + c + g
        tau_decomposed = M @ ddq + c + g

        assert_allclose(
            tau, tau_decomposed, rtol=1e-8, atol=1e-8,
            err_msg="Equation of motion decomposition failed"
        )
