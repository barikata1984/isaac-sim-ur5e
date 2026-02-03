"""Pytest configuration and fixtures for iparam_identification tests."""

import numpy as np
import pytest


@pytest.fixture
def sample_joint_state():
    """Sample joint state for testing."""
    return {
        "q": np.array([0.1, -0.5, 0.3, -1.2, 0.8, 0.2]),
        "dq": np.array([0.05, -0.02, 0.01, -0.03, 0.02, 0.01]),
        "ddq": np.array([0.01, -0.005, 0.002, -0.008, 0.003, 0.001]),
    }


@pytest.fixture
def sample_wrench():
    """Sample force-torque measurement."""
    return {
        "force": np.array([1.0, 2.0, 9.81]),
        "torque": np.array([0.1, 0.2, 0.05]),
    }


@pytest.fixture
def sample_phi():
    """Sample inertial parameter vector.

    Represents a 1kg payload with:
    - CoM at (0, 0, 0.05) m
    - Diagonal inertia 0.01 kg·m²
    """
    return np.array([
        1.0,      # m
        0.0,      # m*cx
        0.0,      # m*cy
        0.05,     # m*cz
        0.01,     # Ixx
        0.0,      # Ixy
        0.0,      # Ixz
        0.01,     # Iyy
        0.0,      # Iyz
        0.01,     # Izz
    ])


@pytest.fixture
def gravity():
    """Standard gravity vector."""
    return np.array([0.0, 0.0, -9.81])


@pytest.fixture
def kinematics():
    """PinocchioKinematics instance for UR5e.

    This fixture requires kinematics package and ur_description to be available.
    """
    try:
        from kinematics import PinocchioKinematics
        return PinocchioKinematics.for_ur5e()
    except ImportError:
        pytest.skip("kinematics package not available")
    except Exception as e:
        pytest.skip(f"Failed to load UR5e model: {e}")
