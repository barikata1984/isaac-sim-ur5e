"""Tests for spatial_inertia module."""

import numpy as np
import pytest

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))
ur_src = Path(__file__).parent.parent.parent / 'robots' / 'ur' / 'src'
sys.path.insert(0, str(ur_src))

from spatial_inertia import (
    spatial_inertia_at_com,
    spatial_inertia_at_frame,
    transform_spatial_inertia,
    is_positive_definite,
    is_symmetric,
)
from ur5e import UR5eParameters


class TestSpatialInertiaAtCom:
    """Tests for spatial inertia at center of mass."""

    def test_shape(self):
        """Spatial inertia should be 6x6."""
        mass = 1.0
        inertia = np.eye(3) * 0.1
        G = spatial_inertia_at_com(mass, inertia)
        assert G.shape == (6, 6)

    def test_block_diagonal(self):
        """At CoM, spatial inertia should be block diagonal.

        For [ω, v] convention:
            G = [[I_c,    0    ],
                 [0,      m*I_3]]
        """
        mass = 2.5
        inertia = np.diag([0.1, 0.2, 0.3])
        G = spatial_inertia_at_com(mass, inertia)

        # Check block structure [ω, v]: inertia upper-left, mass lower-right
        np.testing.assert_array_almost_equal(G[:3, :3], inertia)
        np.testing.assert_array_almost_equal(G[3:, 3:], mass * np.eye(3))
        np.testing.assert_array_almost_equal(G[:3, 3:], np.zeros((3, 3)))
        np.testing.assert_array_almost_equal(G[3:, :3], np.zeros((3, 3)))

    def test_symmetric(self):
        """Spatial inertia should be symmetric."""
        mass = 3.0
        inertia = np.array([
            [0.1, 0.01, 0.02],
            [0.01, 0.2, 0.03],
            [0.02, 0.03, 0.3]
        ])
        G = spatial_inertia_at_com(mass, inertia)
        assert is_symmetric(G)

    def test_positive_definite(self):
        """Spatial inertia with positive mass should be positive definite."""
        mass = 2.0
        inertia = np.eye(3) * 0.1
        G = spatial_inertia_at_com(mass, inertia)
        assert is_positive_definite(G)


class TestSpatialInertiaAtFrame:
    """Tests for spatial inertia at arbitrary frame."""

    def test_zero_offset(self):
        """Zero offset should give same as at_com."""
        mass = 2.0
        inertia = np.diag([0.1, 0.2, 0.3])
        p = np.zeros(3)

        G_com = spatial_inertia_at_com(mass, inertia)
        G_frame = spatial_inertia_at_frame(mass, inertia, p)

        np.testing.assert_array_almost_equal(G_frame, G_com)

    def test_parallel_axis_theorem(self):
        """Verify parallel axis theorem for rotational inertia.

        For [ω, v] convention, inertia block is in upper-left G[:3, :3].
        """
        mass = 3.0
        inertia = np.diag([0.1, 0.2, 0.15])
        p = np.array([0.1, 0.2, 0.3])

        G = spatial_inertia_at_frame(mass, inertia, p)

        # Extract inertia block (upper-left for [ω, v] convention)
        I_frame = G[:3, :3]

        # Expected from parallel axis theorem
        I_expected = inertia + mass * (np.dot(p, p) * np.eye(3) - np.outer(p, p))

        np.testing.assert_array_almost_equal(I_frame, I_expected)

    def test_symmetric(self):
        """Spatial inertia at frame should be symmetric."""
        mass = 2.5
        inertia = np.diag([0.1, 0.15, 0.2])
        p = np.array([0.05, -0.1, 0.15])

        G = spatial_inertia_at_frame(mass, inertia, p)
        assert is_symmetric(G)

    def test_positive_definite(self):
        """Spatial inertia at frame should be positive definite."""
        mass = 2.5
        inertia = np.diag([0.1, 0.15, 0.2])
        p = np.array([0.05, -0.1, 0.15])

        G = spatial_inertia_at_frame(mass, inertia, p)
        assert is_positive_definite(G)

    def test_coupling_terms(self):
        """Check that coupling terms are correct."""
        mass = 1.0
        inertia = np.eye(3)
        p = np.array([1.0, 0.0, 0.0])

        G = spatial_inertia_at_frame(mass, inertia, p)

        # With p = [1, 0, 0], the skew matrix is:
        # [0, 0, 0]
        # [0, 0, -1]
        # [0, 1, 0]
        p_skew = np.array([
            [0, 0, 0],
            [0, 0, -1],
            [0, 1, 0]
        ])

        np.testing.assert_array_almost_equal(G[:3, 3:], mass * p_skew)
        np.testing.assert_array_almost_equal(G[3:, :3], -mass * p_skew)


class TestUR5eInertia:
    """Tests with UR5e parameters."""

    def test_ur5e_spatial_inertias(self):
        """Verify UR5e spatial inertias are valid."""
        params = UR5eParameters()

        for i in range(params.n_joints):
            mass = params.link_masses[i]
            inertia = params.link_inertias[i]
            com_pos = params.link_com_positions[i]

            # At CoM
            G_com = spatial_inertia_at_com(mass, inertia)
            assert is_symmetric(G_com)
            assert is_positive_definite(G_com)

            # At joint frame
            G_joint = spatial_inertia_at_frame(mass, inertia, com_pos)
            assert is_symmetric(G_joint)
            assert is_positive_definite(G_joint)


class TestTransformSpatialInertia:
    """Tests for transforming spatial inertia."""

    def test_identity_transform(self):
        """Identity transform should not change inertia."""
        mass = 2.0
        inertia = np.diag([0.1, 0.2, 0.3])
        G = spatial_inertia_at_com(mass, inertia)

        T = np.eye(4)
        G_transformed = transform_spatial_inertia(G, T)

        np.testing.assert_array_almost_equal(G_transformed, G)

    def test_pure_rotation(self):
        """Rotation should preserve eigenvalues."""
        mass = 2.0
        inertia = np.diag([0.1, 0.2, 0.3])
        G = spatial_inertia_at_com(mass, inertia)

        # Rotation about z by 90 degrees
        T = np.eye(4)
        T[:3, :3] = np.array([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 1]
        ])

        G_rotated = transform_spatial_inertia(G, T)

        # Eigenvalues should be preserved
        eig_original = np.sort(np.linalg.eigvalsh(G))
        eig_rotated = np.sort(np.linalg.eigvalsh(G_rotated))

        np.testing.assert_array_almost_equal(eig_original, eig_rotated)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
