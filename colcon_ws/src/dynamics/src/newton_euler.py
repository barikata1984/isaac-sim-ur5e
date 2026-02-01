"""Newton-Euler inverse dynamics algorithm.

This module implements the Newton-Euler algorithm for computing inverse
dynamics of serial link robots, based on Lynch and Park 2017, Chapter 8.

Uses pymlg convention: [ω, v] (angular velocity, linear velocity).
"""

from dataclasses import dataclass
from typing import List, Optional

import numpy as np
from pymlg.numpy import SE3, SO3


def _dh_transform(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    """Compute Modified DH transformation matrix."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -sa * d],
        [st * sa, ct * sa, ca, ca * d],
        [0, 0, 0, 1]
    ])


def _spatial_inertia(mass: float, inertia: np.ndarray, com: np.ndarray) -> np.ndarray:
    """Compute spatial inertia matrix in [ω, v] convention (Lynch & Park).

    For [ω, v] convention:
        G = [[I + m*[p]×[p]×^T,  m*[p]×  ],
             [m*[p]×^T,          m*I     ]]
    """
    p = np.asarray(com).ravel()
    p_skew = SO3.wedge(p)
    parallel_axis = mass * (np.dot(p, p) * np.eye(3) - np.outer(p, p))

    G = np.zeros((6, 6))
    G[:3, :3] = inertia + parallel_axis
    G[:3, 3:] = mass * p_skew
    G[3:, :3] = -mass * p_skew  # = (mass * p_skew).T
    G[3:, 3:] = mass * np.eye(3)

    return G


@dataclass
class NewtonEulerResult:
    """Result of Newton-Euler inverse dynamics computation."""
    tau: np.ndarray
    link_twists: List[np.ndarray]
    link_accelerations: List[np.ndarray]
    link_wrenches: List[np.ndarray]


class NewtonEulerDynamics:
    """Newton-Euler inverse dynamics for serial link robots.

    Uses pymlg [ω, v] convention throughout.
    """

    def __init__(
        self,
        n_joints: int,
        dh_params: np.ndarray,
        link_masses: np.ndarray,
        link_com_positions: np.ndarray,
        link_inertias: np.ndarray,
        gravity: Optional[np.ndarray] = None,
    ):
        self.n_joints = n_joints
        self.dh_params = np.asarray(dh_params)
        self.link_masses = np.asarray(link_masses)
        self.link_com_positions = np.asarray(link_com_positions)
        self.link_inertias = np.asarray(link_inertias)
        self.gravity = np.array([0.0, 0.0, -9.81]) if gravity is None else np.asarray(gravity)

        # Precompute spatial inertia matrices in [ω, v] convention
        self._spatial_inertias = [
            _spatial_inertia(
                self.link_masses[i],
                self.link_inertias[i],
                self.link_com_positions[i]
            )
            for i in range(n_joints)
        ]

    @classmethod
    def from_robot_params(cls, params, gravity: Optional[np.ndarray] = None):
        """Create from RobotParametersBase instance."""
        return cls(
            n_joints=params.n_joints,
            dh_params=params.dh_params,
            link_masses=params.link_masses,
            link_com_positions=params.link_com_positions,
            link_inertias=params.link_inertias,
            gravity=gravity,
        )

    def _joint_screw_axis(self) -> np.ndarray:
        """Joint screw axis in [ω, v] convention: revolute about z."""
        return np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

    def _compute_link_transforms(self, q: np.ndarray) -> List[np.ndarray]:
        """Compute DH transforms for each joint."""
        return [
            _dh_transform(
                self.dh_params[i, 0],
                self.dh_params[i, 1],
                self.dh_params[i, 2],
                q[i]
            )
            for i in range(self.n_joints)
        ]

    def forward_pass(self, q: np.ndarray, dq: np.ndarray, ddq: np.ndarray) -> tuple:
        """Forward iteration: compute link twists and accelerations.

        All quantities in [ω, v] convention.
        """
        q = np.asarray(q).ravel()
        dq = np.asarray(dq).ravel()
        ddq = np.asarray(ddq).ravel()

        transforms = self._compute_link_transforms(q)
        A = self._joint_screw_axis()

        # Base: V_0 = 0, V̇_0 = [0, -g] in [ω, v] convention
        V_prev = np.zeros(6)
        V_dot_prev = np.concatenate([np.zeros(3), -self.gravity])

        twists = []
        accelerations = []

        for i in range(self.n_joints):
            T_prev_i = transforms[i]  # T_{i-1, i}
            T_i_prev = SE3.inverse(T_prev_i)  # T_{i, i-1}
            Ad_i_prev = SE3.adjoint(T_i_prev)  # Ad_{T_{i, i-1}}

            # Propagate twist
            V_i = Ad_i_prev @ V_prev + A * dq[i]

            # Lie bracket via adjoint_algebra
            Xi = SE3.wedge(V_i)
            ad_V_i = SE3.adjoint_algebra(Xi)

            # Propagate acceleration
            V_dot_i = Ad_i_prev @ V_dot_prev + ad_V_i @ A * dq[i] + A * ddq[i]

            twists.append(V_i)
            accelerations.append(V_dot_i)

            V_prev = V_i
            V_dot_prev = V_dot_i

        return transforms, twists, accelerations

    def backward_pass(
        self,
        transforms: List[np.ndarray],
        twists: List[np.ndarray],
        accelerations: List[np.ndarray],
        F_tip: Optional[np.ndarray] = None,
    ) -> tuple:
        """Backward iteration: compute link wrenches and joint torques.

        All quantities in [τ, f] convention (dual of [ω, v]).
        """
        F_next = np.zeros(6) if F_tip is None else np.asarray(F_tip).ravel()
        A = self._joint_screw_axis()

        wrenches = [None] * self.n_joints
        tau = np.zeros(self.n_joints)

        for i in range(self.n_joints - 1, -1, -1):
            V_i = twists[i]
            V_dot_i = accelerations[i]
            G_i = self._spatial_inertias[i]

            Xi = SE3.wedge(V_i)
            ad_V_i = SE3.adjoint_algebra(Xi)

            # F_i = G_i @ V̇_i - ad_{V_i}^T @ G_i @ V_i
            F_dynamics = G_i @ V_dot_i - ad_V_i.T @ G_i @ V_i

            if i < self.n_joints - 1:
                # Transform wrench from frame i+1 to frame i
                T_i_next = transforms[i + 1]  # T_{i, i+1}
                T_next_i = SE3.inverse(T_i_next)  # T_{i+1, i}
                Ad_next_i = SE3.adjoint(T_next_i)
                F_i = F_dynamics + Ad_next_i.T @ F_next
            else:
                F_i = F_dynamics + F_next

            wrenches[i] = F_i
            tau[i] = A @ F_i
            F_next = F_i

        return wrenches, tau

    def inverse_dynamics(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        ddq: np.ndarray,
        F_tip: Optional[np.ndarray] = None,
    ) -> NewtonEulerResult:
        """Compute inverse dynamics: τ = M(q)q̈ + c(q,q̇) + g(q)."""
        transforms, twists, accelerations = self.forward_pass(q, dq, ddq)
        wrenches, tau = self.backward_pass(transforms, twists, accelerations, F_tip)

        return NewtonEulerResult(
            tau=tau,
            link_twists=twists,
            link_accelerations=accelerations,
            link_wrenches=wrenches,
        )

    def gravity_torques(self, q: np.ndarray) -> np.ndarray:
        """Compute gravity compensation torques: g(q)."""
        zero = np.zeros(self.n_joints)
        return self.inverse_dynamics(q, zero, zero).tau

    def coriolis_centrifugal(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """Compute Coriolis/centrifugal torques: c(q, q̇)."""
        zero = np.zeros(self.n_joints)
        tau_full = self.inverse_dynamics(q, dq, zero).tau
        return tau_full - self.gravity_torques(q)

    def mass_matrix(self, q: np.ndarray) -> np.ndarray:
        """Compute mass matrix M(q)."""
        zero = np.zeros(self.n_joints)
        M = np.zeros((self.n_joints, self.n_joints))

        gravity_backup = self.gravity.copy()
        self.gravity = np.zeros(3)

        for i in range(self.n_joints):
            ddq = np.zeros(self.n_joints)
            ddq[i] = 1.0
            M[:, i] = self.inverse_dynamics(q, zero, ddq).tau

        self.gravity = gravity_backup
        return 0.5 * (M + M.T)
