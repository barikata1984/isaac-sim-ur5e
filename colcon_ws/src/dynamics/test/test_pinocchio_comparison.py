"""Cross-validation tests against Pinocchio library.

These tests compare our Newton-Euler implementation against Pinocchio,
a widely-used robotics library that implements the same algorithms.

To run these tests, install pinocchio:
    pip install pin

Note: These tests require a UR5e URDF file. If not available, tests will be skipped.
"""

import numpy as np
import pytest
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))
ur_src = Path(__file__).parent.parent.parent / 'robots' / 'ur' / 'src'
sys.path.insert(0, str(ur_src))

# Try to import pinocchio
try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False

from newton_euler import NewtonEulerDynamics, _dh_transform
from ur5e import UR5eParameters


# Skip all tests if pinocchio is not available
pytestmark = pytest.mark.skipif(
    not PINOCCHIO_AVAILABLE,
    reason="Pinocchio not installed. Install with: pip install pin"
)


def create_ur5e_pinocchio_model():
    """Create a UR5e Pinocchio model programmatically.

    Since we may not have a URDF file, we build the model using
    Pinocchio's model builder API based on the same DH parameters
    we use in our implementation.

    The key insight is that for Modified DH convention:
    - _dh_transform(a, d, alpha, theta) gives T_{i-1, i}
    - At theta=0, this gives the "home" transform between frames
    - Pinocchio's joint_placement is this home transform

    Returns:
        Tuple of (model, data) or None if creation fails.
    """
    if not PINOCCHIO_AVAILABLE:
        return None, None

    params = UR5eParameters()

    # Create an empty model
    model = pin.Model()

    # Set gravity (Pinocchio uses [linear, angular] convention)
    model.gravity = pin.Motion(np.array([0, 0, -9.81, 0, 0, 0]))

    # Parent frame for first joint is the universe (0)
    parent_id = 0

    # Build joints and links
    for i in range(params.n_joints):
        a = params.dh_params[i, 0]
        d = params.dh_params[i, 1]
        alpha = params.dh_params[i, 2]

        # Get DH transform at theta=0 (home configuration)
        # This defines the joint placement relative to parent
        T = _dh_transform(a, d, alpha, theta=0)
        joint_placement = pin.SE3(T[:3, :3], T[:3, 3])

        # Add revolute joint about z
        joint_id = model.addJoint(
            parent_id,
            pin.JointModelRZ(),
            joint_placement,
            f"joint_{i+1}"
        )

        # Add link (body) with inertia
        mass = params.link_masses[i]
        com = params.link_com_positions[i]
        inertia = params.link_inertias[i]

        # Create Pinocchio inertia from our parameters
        # Pinocchio uses inertia at CoM in link frame
        pin_inertia = pin.Inertia(
            mass,
            com,  # CoM position in link frame
            inertia  # Inertia matrix at CoM
        )

        model.appendBodyToJoint(joint_id, pin_inertia, pin.SE3.Identity())

        parent_id = joint_id

    # Create data
    data = model.createData()

    return model, data


class TestPinocchioComparison:
    """Tests comparing our implementation against Pinocchio."""

    @pytest.fixture
    def pinocchio_model(self):
        """Create Pinocchio model and data."""
        model, data = create_ur5e_pinocchio_model()
        if model is None:
            pytest.skip("Could not create Pinocchio model")
        return model, data

    @pytest.fixture
    def our_dynamics(self):
        """Create our Newton-Euler dynamics instance."""
        params = UR5eParameters()
        return NewtonEulerDynamics.from_robot_params(params)

    def test_gravity_torques(self, pinocchio_model, our_dynamics):
        """Compare gravity torque computation."""
        model, data = pinocchio_model

        np.random.seed(42)
        for _ in range(5):
            q = np.random.uniform(-np.pi, np.pi, 6)

            # Pinocchio
            g_pin = pin.computeGeneralizedGravity(model, data, q)

            # Our implementation
            g_ours = our_dynamics.gravity_torques(q)

            np.testing.assert_array_almost_equal(
                g_ours, g_pin,
                decimal=4,
                err_msg=f"Gravity torques differ at q={q}"
            )

    def test_mass_matrix(self, pinocchio_model, our_dynamics):
        """Compare mass matrix computation."""
        model, data = pinocchio_model

        np.random.seed(42)
        for _ in range(5):
            q = np.random.uniform(-np.pi, np.pi, 6)

            # Pinocchio - CRBA (Composite Rigid Body Algorithm)
            pin.crba(model, data, q)
            M_pin = data.M.copy()
            # Make symmetric (Pinocchio only fills upper triangle)
            M_pin = 0.5 * (M_pin + M_pin.T)

            # Our implementation
            M_ours = our_dynamics.mass_matrix(q)

            np.testing.assert_array_almost_equal(
                M_ours, M_pin,
                decimal=4,
                err_msg=f"Mass matrices differ at q={q}"
            )

    def test_inverse_dynamics(self, pinocchio_model, our_dynamics):
        """Compare inverse dynamics (RNEA) computation."""
        model, data = pinocchio_model

        np.random.seed(42)
        for _ in range(5):
            q = np.random.uniform(-np.pi, np.pi, 6)
            dq = np.random.uniform(-1.0, 1.0, 6)
            ddq = np.random.uniform(-0.5, 0.5, 6)

            # Pinocchio - RNEA
            tau_pin = pin.rnea(model, data, q, dq, ddq)

            # Our implementation
            tau_ours = our_dynamics.inverse_dynamics(q, dq, ddq).tau

            np.testing.assert_array_almost_equal(
                tau_ours, tau_pin,
                decimal=4,
                err_msg=f"Inverse dynamics differ at q={q}, dq={dq}, ddq={ddq}"
            )

    def test_zero_velocity_inverse_dynamics(self, pinocchio_model, our_dynamics):
        """Compare inverse dynamics with zero velocity."""
        model, data = pinocchio_model

        np.random.seed(42)
        q = np.random.uniform(-np.pi, np.pi, 6)
        dq = np.zeros(6)
        ddq = np.random.uniform(-1.0, 1.0, 6)

        # Pinocchio
        tau_pin = pin.rnea(model, data, q, dq, ddq)

        # Our implementation
        tau_ours = our_dynamics.inverse_dynamics(q, dq, ddq).tau

        np.testing.assert_array_almost_equal(
            tau_ours, tau_pin,
            decimal=4,
            err_msg="Zero-velocity inverse dynamics differ"
        )

    def test_static_inverse_dynamics(self, pinocchio_model, our_dynamics):
        """Compare static inverse dynamics (only gravity)."""
        model, data = pinocchio_model

        np.random.seed(42)
        q = np.random.uniform(-np.pi, np.pi, 6)
        dq = np.zeros(6)
        ddq = np.zeros(6)

        # Pinocchio
        tau_pin = pin.rnea(model, data, q, dq, ddq)

        # Our implementation
        tau_ours = our_dynamics.inverse_dynamics(q, dq, ddq).tau

        np.testing.assert_array_almost_equal(
            tau_ours, tau_pin,
            decimal=4,
            err_msg="Static inverse dynamics differ"
        )


class TestTwistComparison:
    """Direct twist comparison between our implementation and Pinocchio.

    Our implementation uses pymlg [ω, v] convention.
    Pinocchio uses [v, ω] convention.
    After permuting Pinocchio's [v, ω] to [ω, v], they should match.
    """

    @pytest.fixture
    def pinocchio_model(self):
        """Create Pinocchio model and data."""
        model, data = create_ur5e_pinocchio_model()
        if model is None:
            pytest.skip("Could not create Pinocchio model")
        return model, data

    @pytest.fixture
    def our_dynamics(self):
        """Create our Newton-Euler dynamics instance."""
        params = UR5eParameters()
        return NewtonEulerDynamics.from_robot_params(params)

    @staticmethod
    def _permute_v_w_to_w_v(twist_vw: np.ndarray) -> np.ndarray:
        """Convert Pinocchio [v, ω] to pymlg [ω, v] convention."""
        return np.concatenate([twist_vw[3:], twist_vw[:3]])

    def test_link_twists_match(self, pinocchio_model, our_dynamics):
        """Verify link twists match after convention permutation.

        This is a direct validation that intermediate calculations
        (not just final torques) are correct.
        """
        model, data = pinocchio_model

        np.random.seed(42)
        for _ in range(3):
            q = np.random.uniform(-np.pi / 2, np.pi / 2, 6)
            dq = np.random.uniform(-1.0, 1.0, 6)
            ddq = np.random.uniform(-0.5, 0.5, 6)

            # Our implementation: get twists from forward pass
            transforms, twists_ours, accels_ours = our_dynamics.forward_pass(q, dq, ddq)

            # Pinocchio: compute kinematics
            pin.forwardKinematics(model, data, q, dq, ddq)

            # Compare link-by-link
            for i in range(6):
                # Pinocchio twist in body frame (link frame)
                # data.v[i+1] because index 0 is universe
                twist_pin = data.v[i + 1].np  # [v, ω] convention
                twist_pin_converted = self._permute_v_w_to_w_v(twist_pin)  # [ω, v]

                twist_ours = twists_ours[i]  # [ω, v]

                np.testing.assert_array_almost_equal(
                    twist_ours, twist_pin_converted,
                    decimal=6,
                    err_msg=f"Link {i+1} twist differs at q={q}"
                )


class TestMassMatrixProperties:
    """Additional mass matrix property tests."""

    @pytest.fixture
    def pinocchio_model(self):
        """Create Pinocchio model and data."""
        model, data = create_ur5e_pinocchio_model()
        if model is None:
            pytest.skip("Could not create Pinocchio model")
        return model, data

    @pytest.fixture
    def our_dynamics(self):
        """Create our Newton-Euler dynamics instance."""
        params = UR5eParameters()
        return NewtonEulerDynamics.from_robot_params(params)

    def test_mass_matrix_eigenvalue_range(self, pinocchio_model, our_dynamics):
        """Mass matrix eigenvalues should be in similar range for both."""
        model, data = pinocchio_model

        np.random.seed(42)
        q = np.random.uniform(-np.pi, np.pi, 6)

        # Pinocchio
        pin.crba(model, data, q)
        M_pin = 0.5 * (data.M + data.M.T)
        eig_pin = np.linalg.eigvalsh(M_pin)

        # Our implementation
        M_ours = our_dynamics.mass_matrix(q)
        eig_ours = np.linalg.eigvalsh(M_ours)

        # Eigenvalues should be in similar range
        np.testing.assert_array_almost_equal(
            eig_ours, eig_pin,
            decimal=4,
            err_msg="Mass matrix eigenvalues differ"
        )


if __name__ == '__main__':
    if not PINOCCHIO_AVAILABLE:
        print("Pinocchio not installed. Install with: pip install pin")
        print("Tests will be skipped.")
    pytest.main([__file__, '-v'])
