"""Unit tests for pymlg-based Lie algebra operations.

Tests that pymlg SE3/SO3 methods work correctly for dynamics calculations.
"""

import numpy as np
import pytest
from numpy.testing import assert_allclose
from pymlg import SE3, SO3


class TestSkew:
    """Tests for skew symmetric matrix operations."""

    def test_skew_basic(self) -> None:
        """Test skew matrix construction."""
        v = np.array([1.0, 2.0, 3.0])
        S = SO3.wedge(v)

        expected = np.array([
            [0, -3, 2],
            [3, 0, -1],
            [-2, 1, 0]
        ], dtype=np.float64)
        assert_allclose(S, expected)

    def test_skew_antisymmetric(self) -> None:
        """Test that skew matrix is antisymmetric."""
        v = np.array([1.0, 2.0, 3.0])
        S = SO3.wedge(v)
        assert_allclose(S, -S.T)

    def test_wedge_vee_inverse(self) -> None:
        """Test wedge and vee are inverses."""
        v = np.array([1.0, 2.0, 3.0])
        S = SO3.wedge(v)
        v_recovered = SO3.vee(S).flatten()
        assert_allclose(v_recovered, v)


class TestSE3Operations:
    """Tests for SE(3) operations."""

    def test_se3_wedge_vee_inverse(self) -> None:
        """Test that wedge and vee are inverses."""
        twist = np.array([0.1, 0.2, 0.3, 1.0, 2.0, 3.0])
        xi_hat = SE3.wedge(twist)
        twist_recovered = SE3.vee(xi_hat).flatten()
        assert_allclose(twist_recovered, twist)

    def test_se3_exp_identity(self) -> None:
        """Test exponential of zero is identity."""
        twist = np.zeros(6)
        T = SE3.Exp(twist)
        assert_allclose(T, np.eye(4), atol=1e-10)

    def test_se3_exp_pure_translation(self) -> None:
        """Test exponential for pure translation."""
        v = np.array([1.0, 2.0, 3.0])
        twist = np.concatenate([np.zeros(3), v])
        T = SE3.Exp(twist)

        assert_allclose(T[:3, :3], np.eye(3), atol=1e-10)
        assert_allclose(T[:3, 3], v, atol=1e-10)

    def test_se3_exp_pure_rotation_z(self) -> None:
        """Test exponential for pure rotation about z-axis."""
        omega = np.array([0, 0, np.pi / 2])
        twist = np.concatenate([omega, np.zeros(3)])

        T = SE3.Exp(twist)

        # 90 degree rotation about z
        expected_R = np.array([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 1]
        ], dtype=np.float64)
        assert_allclose(T[:3, :3], expected_R, atol=1e-10)
        assert_allclose(T[:3, 3], np.zeros(3), atol=1e-10)

    def test_se3_log_exp_inverse(self) -> None:
        """Test that log and exp are inverses."""
        omega = np.array([0.1, 0.2, 0.3])
        v = np.array([1.0, 2.0, 3.0])
        twist = np.concatenate([omega, v])

        T = SE3.Exp(twist)
        twist_recovered = SE3.Log(T).flatten()
        assert_allclose(twist_recovered, twist, atol=1e-10)


class TestAdjoint:
    """Tests for Adjoint representation."""

    def test_adjoint_identity(self) -> None:
        """Test Adjoint of identity transform."""
        T = np.eye(4)
        Ad_T = SE3.adjoint(T)
        assert_allclose(Ad_T, np.eye(6))

    def test_adjoint_pure_rotation(self) -> None:
        """Test Adjoint for pure rotation."""
        theta = np.pi / 4
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])
        T = SE3.from_components(R, np.zeros(3))
        Ad_T = SE3.adjoint(T)

        # For pure rotation: Ad = [[R, 0], [0, R]]
        assert_allclose(Ad_T[:3, :3], R)
        assert_allclose(Ad_T[3:, 3:], R)
        assert_allclose(Ad_T[:3, 3:], np.zeros((3, 3)), atol=1e-10)


class TestAd:
    """Tests for ad (Lie bracket) operator."""

    def test_ad_zero_twist(self) -> None:
        """Test ad of zero twist is zero matrix."""
        twist = np.zeros(6)
        ad_V = SE3.adjoint_algebra(SE3.wedge(twist))
        assert_allclose(ad_V, np.zeros((6, 6)))

    def test_ad_structure(self) -> None:
        """Test structure of ad matrix."""
        omega = np.array([0.1, 0.2, 0.3])
        v = np.array([1.0, 2.0, 3.0])
        twist = np.concatenate([omega, v])

        ad_V = SE3.adjoint_algebra(SE3.wedge(twist))

        # ad_V = [[[omega], 0], [[v], [omega]]]
        omega_hat = SO3.wedge(omega)
        v_hat = SO3.wedge(v)

        assert_allclose(ad_V[:3, :3], omega_hat)
        assert_allclose(ad_V[:3, 3:], np.zeros((3, 3)), atol=1e-10)
        assert_allclose(ad_V[3:, :3], v_hat)
        assert_allclose(ad_V[3:, 3:], omega_hat)


class TestTransformUtilities:
    """Tests for transformation utilities."""

    def test_inverse_transform_identity(self) -> None:
        """Test inverse of identity is identity."""
        T = np.eye(4)
        T_inv = SE3.inverse(T)
        assert_allclose(T_inv, np.eye(4))

    def test_inverse_transform_product(self) -> None:
        """Test T * T^{-1} = I."""
        R = np.array([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 1]
        ], dtype=np.float64)
        p = np.array([1.0, 2.0, 3.0])
        T = SE3.from_components(R, p)

        T_inv = SE3.inverse(T)
        product = T @ T_inv

        assert_allclose(product, np.eye(4), atol=1e-10)
