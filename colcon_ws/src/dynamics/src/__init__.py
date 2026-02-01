"""Dynamics package for Newton-Euler inverse dynamics.

This package provides:
- Spatial inertia matrix utilities
- Newton-Euler inverse dynamics algorithm

Based on Lynch and Park 2017, Chapter 8 (Dynamics of Open Chains).
Uses pymlg [ω, v] convention for internal calculations.

For kinematics (forward kinematics, Jacobian), use the kinematics package.
"""

from dynamics.spatial_inertia import (
    spatial_inertia_at_com,
    spatial_inertia_at_frame,
    transform_spatial_inertia,
)
from dynamics.newton_euler import (
    NewtonEulerResult,
    NewtonEulerDynamics,
)

__all__ = [
    # spatial_inertia
    'spatial_inertia_at_com',
    'spatial_inertia_at_frame',
    'transform_spatial_inertia',
    # newton_euler
    'NewtonEulerResult',
    'NewtonEulerDynamics',
]
