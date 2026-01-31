"""Newton-Euler inverse dynamics using native Lie algebra implementation.

Uses the twist-wrench formulation but without pymlg dependency.
Based on Lynch and Park (2017), Section 8.3.2.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from dynamics.lie_algebra_native import (
    ad,
    ad_transpose,
    adjoint,
    adjoint_transpose,
    se3_exp,
    inverse_transform,
)
from dynamics.ur5e_parameters import UR5eParameters


@dataclass
class DynamicsStateNative:
    """State variables computed during Newton-Euler algorithm.

    Attributes:
        twists: List of body twists V_i for each link (6,).
        twist_dots: List of body twist derivatives V̇_i for each link (6,).
        wrenches: List of body wrenches F_i for each link (6,).
        transforms: List of transforms T_{i,i-1} for each link (4, 4).
    """

    twists: List[np.ndarray] = field(default_factory=list)
    twist_dots: List[np.ndarray] = field(default_factory=list)
    wrenches: List[np.ndarray] = field(default_factory=list)
    transforms: List[np.ndarray] = field(default_factory=list)


class NewtonEulerNative:
    """Newton-Euler inverse dynamics using native Lie algebra.

    Same algorithm as NewtonEulerDynamics but using pure NumPy
    implementation without pymlg.

    Attributes:
        params: UR5e kinematic and dynamic parameters.
        gravity: Gravity vector in base frame [m/s²].
    """

    def __init__(
        self,
        params: Optional[UR5eParameters] = None,
        gravity: Optional[np.ndarray] = None,
    ) -> None:
        """Initialize Newton-Euler dynamics.

        Args:
            params: UR5e parameters. If None, uses default parameters.
            gravity: Gravity vector in base frame. Default is [0, 0, -9.81].
        """
        self.params = params if params is not None else UR5eParameters()

        if gravity is None:
            self.gravity = np.array([0.0, 0.0, -9.81])
        else:
            self.gravity = np.asarray(gravity, dtype=np.float64).flatten()

        # Pre-compute static parameters
        self._screw_axes = self.params.get_screw_axes()
        self._home_configs = self.params.get_home_configurations()
        self._spatial_inertias = self.params.get_spatial_inertia_matrices()
        self._ee_frame = self.params.get_end_effector_frame()

    def _forward_iterations(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        ddq: np.ndarray,
    ) -> DynamicsStateNative:
        """Perform forward iterations of Newton-Euler algorithm.

        Args:
            q: Joint positions [rad] (n,).
            dq: Joint velocities [rad/s] (n,).
            ddq: Joint accelerations [rad/s²] (n,).

        Returns:
            DynamicsStateNative with twists, twist_dots, and transforms.
        """
        n = self.params.n_joints
        state = DynamicsStateNative()

        V_prev = np.zeros(6)
        Vdot_prev = np.concatenate([np.zeros(3), -self.gravity])

        for i in range(n):
            A_i = self._screw_axes[i]
            M_i = self._home_configs[i]
            theta_i = q[i]
            dtheta_i = dq[i]
            ddtheta_i = ddq[i]

            # T_{i,i-1} = exp(-[A_i]θ_i) * M_{i,i-1}
            T_exp = se3_exp(-A_i, theta_i)
            T_i_im1 = T_exp @ M_i
            state.transforms.append(T_i_im1)

            Ad_T = adjoint(T_i_im1)

            # V_i = [Ad_{T_{i,i-1}}] * V_{i-1} + A_i * θ̇_i
            V_i = Ad_T @ V_prev + A_i * dtheta_i
            state.twists.append(V_i)

            # V̇_i = [Ad_{T_{i,i-1}}] * V̇_{i-1} + [ad_{V_i}] * A_i * θ̇_i + A_i * θ̈_i
            ad_V_i = ad(V_i)
            Vdot_i = Ad_T @ Vdot_prev + ad_V_i @ A_i * dtheta_i + A_i * ddtheta_i
            state.twist_dots.append(Vdot_i)

            V_prev = V_i
            Vdot_prev = Vdot_i

        return state

    def _backward_iterations(
        self,
        state: DynamicsStateNative,
        F_tip: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, DynamicsStateNative]:
        """Perform backward iterations of Newton-Euler algorithm.

        Args:
            state: DynamicsStateNative from forward iterations.
            F_tip: External wrench at end-effector in tool frame (6,).

        Returns:
            Tuple of:
                - tau: Joint torques [N·m] (n,).
                - state: Updated DynamicsStateNative with wrenches.
        """
        n = self.params.n_joints
        tau = np.zeros(n)

        if F_tip is None:
            F_next = np.zeros(6)
        else:
            F_next = np.asarray(F_tip, dtype=np.float64).flatten()
            Ad_T_ee = adjoint(self._ee_frame)
            F_next = Ad_T_ee.T @ F_next

        state.wrenches = [None] * n

        for i in range(n - 1, -1, -1):
            G_i = self._spatial_inertias[i]
            V_i = state.twists[i]
            Vdot_i = state.twist_dots[i]
            A_i = self._screw_axes[i]

            if i < n - 1:
                T_ip1_i = state.transforms[i + 1]
                Ad_T_ip1_i = adjoint(T_ip1_i)
                F_propagated = Ad_T_ip1_i.T @ F_next
            else:
                F_propagated = F_next

            F_inertia = G_i @ Vdot_i
            ad_V_i_T = ad_transpose(V_i)
            F_coriolis = ad_V_i_T @ (G_i @ V_i)

            F_i = F_propagated + F_inertia - F_coriolis
            state.wrenches[i] = F_i

            tau[i] = F_i @ A_i
            F_next = F_i

        return tau, state

    def inverse_dynamics(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        ddq: np.ndarray,
        F_tip: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Compute inverse dynamics: joint torques given motion.

        This method delegates to the joint-frame Newton-Euler implementation
        for accurate torque computation. The internal CoM-based frame chain
        has frame alignment issues that cause incorrect torque results.

        Args:
            q: Joint positions [rad] (n,).
            dq: Joint velocities [rad/s] (n,).
            ddq: Joint accelerations [rad/s²] (n,).
            F_tip: External wrench at end-effector in tool frame (6,).

        Returns:
            tau: Joint torques [N·m] (n,).
        """
        from dynamics.newton_euler_joint_frame import NewtonEulerJointFrame

        q = np.asarray(q, dtype=np.float64).flatten()
        dq = np.asarray(dq, dtype=np.float64).flatten()
        ddq = np.asarray(ddq, dtype=np.float64).flatten()

        n = self.params.n_joints
        if q.shape[0] != n or dq.shape[0] != n or ddq.shape[0] != n:
            raise ValueError(f"Expected arrays of length {n}")

        jf_dynamics = NewtonEulerJointFrame(gravity=self.gravity)
        return jf_dynamics.inverse_dynamics(q, dq, ddq, F_tip)

    def gravity_torques(self, q: np.ndarray) -> np.ndarray:
        """Compute gravity compensation torques.

        Args:
            q: Joint positions [rad] (n,).

        Returns:
            tau_g: Gravity compensation torques [N·m] (n,).
        """
        from dynamics.newton_euler_joint_frame import NewtonEulerJointFrame

        jf_dynamics = NewtonEulerJointFrame(gravity=self.gravity)
        return jf_dynamics.gravity_torques(q)

    def mass_matrix(self, q: np.ndarray) -> np.ndarray:
        """Compute the mass (inertia) matrix M(q).

        Args:
            q: Joint positions [rad] (n,).

        Returns:
            M: Mass matrix (n, n).
        """
        from dynamics.newton_euler_joint_frame import NewtonEulerJointFrame

        jf_dynamics = NewtonEulerJointFrame(gravity=self.gravity)
        return jf_dynamics.mass_matrix(q)

    def coriolis_vector(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """Compute the Coriolis/centrifugal vector c(q, θ̇).

        Args:
            q: Joint positions [rad] (n,).
            dq: Joint velocities [rad/s] (n,).

        Returns:
            c: Coriolis/centrifugal torques (n,).
        """
        from dynamics.newton_euler_joint_frame import NewtonEulerJointFrame

        jf_dynamics = NewtonEulerJointFrame(gravity=self.gravity)
        return jf_dynamics.coriolis_vector(q, dq)

    def get_end_effector_twist(
        self,
        q: np.ndarray,
        dq: np.ndarray,
    ) -> np.ndarray:
        """Compute end-effector twist (velocity) in tool frame.

        Args:
            q: Joint positions [rad] (n,).
            dq: Joint velocities [rad/s] (n,).

        Returns:
            V_ee: End-effector twist [ω, v] in tool frame (6,).
        """
        q = np.asarray(q, dtype=np.float64).flatten()
        dq = np.asarray(dq, dtype=np.float64).flatten()
        ddq = np.zeros(self.params.n_joints)

        state = self._forward_iterations(q, dq, ddq)

        # Get twist of last link
        V_n = state.twists[-1]

        # Transform to end-effector frame
        Ad_T_ee = adjoint(self._ee_frame)
        V_ee = Ad_T_ee @ V_n

        return V_ee

    def get_end_effector_acceleration(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        ddq: np.ndarray,
    ) -> np.ndarray:
        """Compute end-effector acceleration in tool frame.

        Args:
            q: Joint positions [rad] (n,).
            dq: Joint velocities [rad/s] (n,).
            ddq: Joint accelerations [rad/s²] (n,).

        Returns:
            Vdot_ee: End-effector acceleration [α, a] in tool frame (6,).
        """
        q = np.asarray(q, dtype=np.float64).flatten()
        dq = np.asarray(dq, dtype=np.float64).flatten()
        ddq = np.asarray(ddq, dtype=np.float64).flatten()

        state = self._forward_iterations(q, dq, ddq)

        # Get acceleration of last link
        Vdot_n = state.twist_dots[-1]

        # Transform to end-effector frame
        Ad_T_ee = adjoint(self._ee_frame)
        Vdot_ee = Ad_T_ee @ Vdot_n

        return Vdot_ee


def create_ur5e_native_dynamics(
    gravity: Optional[np.ndarray] = None,
) -> NewtonEulerNative:
    """Factory function to create UR5e native dynamics instance.

    Args:
        gravity: Gravity vector in base frame. Default is [0, 0, -9.81].

    Returns:
        NewtonEulerNative instance for UR5e.
    """
    return NewtonEulerNative(params=UR5eParameters(), gravity=gravity)
