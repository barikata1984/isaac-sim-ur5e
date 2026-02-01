"""Pytest fixtures for dynamics tests."""

import sys
from pathlib import Path

import numpy as np
import pytest

# Add parent paths for imports
dynamics_src = Path(__file__).parent.parent / 'src'
ur_src = Path(__file__).parent.parent.parent / 'robots' / 'ur' / 'src'
sys.path.insert(0, str(dynamics_src))
sys.path.insert(0, str(ur_src))

from ur5e import UR5eParameters


@pytest.fixture
def ur5e_params() -> UR5eParameters:
    """Fixture providing UR5e robot parameters."""
    return UR5eParameters()


@pytest.fixture
def zero_config() -> np.ndarray:
    """Zero joint configuration."""
    return np.zeros(6)


@pytest.fixture
def random_config() -> np.ndarray:
    """Random joint configuration within limits."""
    np.random.seed(42)
    return np.random.uniform(-np.pi, np.pi, 6)


@pytest.fixture
def random_velocity() -> np.ndarray:
    """Random joint velocity."""
    np.random.seed(43)
    return np.random.uniform(-1.0, 1.0, 6)


@pytest.fixture
def random_acceleration() -> np.ndarray:
    """Random joint acceleration."""
    np.random.seed(44)
    return np.random.uniform(-0.5, 0.5, 6)


@pytest.fixture
def gravity_vector() -> np.ndarray:
    """Standard gravity vector [m/s^2]."""
    return np.array([0.0, 0.0, -9.81])
