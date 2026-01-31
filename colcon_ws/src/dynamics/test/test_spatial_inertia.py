"""Unit tests for spatial inertia module."""

import numpy as np
import pytest
from numpy.testing import assert_allclose

from dynamics.spatial_inertia import SpatialInertia, spatial_momentum


class TestSpatialInertia:
    """Tests for SpatialInertia class."""

    def test_creation(self) -> None:
        """Test basic creation of SpatialInertia."""
        I_b = np.eye(3) * 0.1
        mass = 1.0
        si = SpatialInertia(I_b=I_b, mass=mass)
        assert si.mass == mass
        assert_allclose(si.I_b, I_b)

    def test_to_matrix(self) -> None:
        """Test conversion to 6x6 matrix."""
        I_b = np.diag([0.1, 0.2, 0.3])
        mass = 2.5
        si = SpatialInertia(I_b=I_b, mass=mass)

        G = si.to_matrix()
        assert G.shape == (6, 6)

        # Check structure: [[I_b, 0], [0, m*I]]
        assert_allclose(G[:3, :3], I_b)
        assert_allclose(G[3:, 3:], mass * np.eye(3))
        assert_allclose(G[:3, 3:], np.zeros((3, 3)))
        assert_allclose(G[3:, :3], np.zeros((3, 3)))

    def test_from_matrix(self) -> None:
        """Test creation from 6x6 matrix."""
        I_b = np.diag([0.1, 0.2, 0.3])
        mass = 2.5
        si_orig = SpatialInertia(I_b=I_b, mass=mass)

        G = si_orig.to_matrix()
        si_new = SpatialInertia.from_matrix(G)

        assert_allclose(si_new.I_b, I_b)
        assert si_new.mass == mass

    def test_invalid_inertia_shape(self) -> None:
        """Test validation of inertia matrix shape."""
        with pytest.raises(ValueError, match="must be"):
            SpatialInertia(I_b=np.eye(4), mass=1.0)

    def test_invalid_mass(self) -> None:
        """Test validation of positive mass."""
        with pytest.raises(ValueError, match="positive"):
            SpatialInertia(I_b=np.eye(3), mass=-1.0)

    def test_kinetic_energy_zero_velocity(self) -> None:
        """Test kinetic energy is zero for zero velocity."""
        si = SpatialInertia(I_b=np.eye(3), mass=1.0)
        twist = np.zeros(6)
        assert si.kinetic_energy(twist) == 0.0

    def test_kinetic_energy_pure_translation(self) -> None:
        """Test kinetic energy for pure translation."""
        mass = 2.0
        si = SpatialInertia(I_b=np.eye(3), mass=mass)
        v = np.array([1.0, 0.0, 0.0])
        twist = np.concatenate([np.zeros(3), v])

        # K = 0.5 * m * v^2
        expected = 0.5 * mass * np.dot(v, v)
        assert_allclose(si.kinetic_energy(twist), expected)

    def test_kinetic_energy_pure_rotation(self) -> None:
        """Test kinetic energy for pure rotation."""
        I_b = np.diag([0.1, 0.2, 0.3])
        si = SpatialInertia(I_b=I_b, mass=1.0)
        omega = np.array([1.0, 0.0, 0.0])
        twist = np.concatenate([omega, np.zeros(3)])

        # K = 0.5 * omega^T * I * omega
        expected = 0.5 * omega @ I_b @ omega
        assert_allclose(si.kinetic_energy(twist), expected)

    def test_steiner_offset_zero(self) -> None:
        """Test Steiner's theorem with zero offset."""
        I_cm = np.diag([0.1, 0.2, 0.3])
        mass = 2.0
        q = np.zeros(3)

        I_q = SpatialInertia.steiner_offset(I_cm, mass, q)
        assert_allclose(I_q, I_cm)

    def test_steiner_offset_nonzero(self) -> None:
        """Test Steiner's theorem with nonzero offset."""
        I_cm = np.diag([0.1, 0.1, 0.1])
        mass = 2.0
        q = np.array([1.0, 0.0, 0.0])

        I_q = SpatialInertia.steiner_offset(I_cm, mass, q)

        # I_q = I_cm + m * (q^T*q*I - q*q^T)
        expected = I_cm + mass * (np.dot(q, q) * np.eye(3) - np.outer(q, q))
        assert_allclose(I_q, expected)


class TestSpatialMomentum:
    """Tests for spatial momentum function."""

    def test_zero_velocity(self) -> None:
        """Test momentum is zero for zero velocity."""
        G = np.eye(6)
        twist = np.zeros(6)
        P = spatial_momentum(G, twist)
        assert_allclose(P, np.zeros(6))

    def test_pure_translation(self) -> None:
        """Test momentum for pure translation."""
        mass = 3.0
        si = SpatialInertia(I_b=np.eye(3), mass=mass)
        G = si.to_matrix()

        v = np.array([1.0, 2.0, 3.0])
        twist = np.concatenate([np.zeros(3), v])

        P = spatial_momentum(G, twist)

        # Linear momentum = m * v
        assert_allclose(P[3:], mass * v)
        assert_allclose(P[:3], np.zeros(3))
