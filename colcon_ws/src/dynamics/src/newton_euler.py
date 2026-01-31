"""Newton-Euler inverse dynamics implementation.

Based on Lynch and Park (2017), Section 8.3.2.
Implements the twist-wrench formulation of Newton-Euler algorithm.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
from pymlg import SE3

from dynamics.ur5e_parameters import UR5eParameters



@dataclass
class DynamicsState:
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


class NewtonEulerDynamics:
    """Newton-Euler inverse dynamics for UR5e robot.

    Implements the recursive Newton-Euler algorithm using twist-wrench
    formulation from Lynch and Park (2017) Section 8.3.2.

    The algorithm computes joint torques τ given:
    - Joint positions θ
    - Joint velocities θ̇
    - Joint accelerations θ̈
    - External wrench at end-effector (optional)

    Coordinate convention:
    - All quantities are expressed in body (link) frames
    - Frames are placed at link centers of mass
    - Gravity is specified in base frame coordinates

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
    ) -> DynamicsState:
        """Perform forward iterations of Newton-Euler algorithm.

        Computes transforms, twists, and twist derivatives for each link.

        Based on Equations 8.50-8.52:
            T_{i,i-1} = exp(-[A_i]θ_i) * M_{i,i-1}
            V_i = [Ad_{T_{i,i-1}}] * V_{i-1} + A_i * θ̇_i
            V̇_i = [Ad_{T_{i,i-1}}] * V̇_{i-1} + [ad_{V_i}] * A_i * θ̇_i + A_i * θ̈_i

        Args:
            q: Joint positions [rad] (n,).
            dq: Joint velocities [rad/s] (n,).
            ddq: Joint accelerations [rad/s²] (n,).

        Returns:
            DynamicsState with twists, twist_dots, and transforms.
        """
        n = self.params.n_joints
        state = DynamicsState()

        # Initial conditions (base frame, link 0)
        # V_0 = 0 (base is stationary)
        # V̇_0 = -[0, g] (accounts for gravity as fictitious acceleration)
        V_prev = np.zeros(6)
        Vdot_prev = np.concatenate([np.zeros(3), -self.gravity])

        for i in range(n):
            A_i = self._screw_axes[i]
            M_i = self._home_configs[i]
            theta_i = q[i]
            dtheta_i = dq[i]
            ddtheta_i = ddq[i]

            # Equation 8.50: T_{i,i-1} = exp(-[A_i]θ_i) * M_{i,i-1}
            # Transform from frame {i-1} to frame {i}
            T_exp = SE3.Exp(-A_i * theta_i)
            T_i_im1 = T_exp @ M_i
            state.transforms.append(T_i_im1)

            # Adjoint of transform
            Ad_T = SE3.adjoint(T_i_im1)

            # Equation 8.51: V_i = [Ad_{T_{i,i-1}}] * V_{i-1} + A_i * θ̇_i
            V_i = Ad_T @ V_prev + A_i * dtheta_i
            state.twists.append(V_i)

            # Equation 8.52: V̇_i = [Ad_{T_{i,i-1}}] * V̇_{i-1}
            #                     + [ad_{V_i}] * A_i * θ̇_i + A_i * θ̈_i
            ad_V_i = SE3.adjoint_algebra(SE3.wedge(V_i))
            Vdot_i = Ad_T @ Vdot_prev + ad_V_i @ A_i * dtheta_i + A_i * ddtheta_i
            state.twist_dots.append(Vdot_i)

            # Update for next iteration
            V_prev = V_i
            Vdot_prev = Vdot_i

        return state

    def _backward_iterations(
        self,
        state: DynamicsState,
        F_tip: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, DynamicsState]:
        """Perform backward iterations of Newton-Euler algorithm.

        Computes wrenches and joint torques.

        Based on Equations 8.53-8.54:
            F_i = [Ad_{T_{i+1,i}}]^T * F_{i+1} + G_i * V̇_i - [ad_{V_i}]^T * G_i * V_i
            τ_i = F_i^T * A_i

        Args:
            state: DynamicsState from forward iterations.
            F_tip: External wrench at end-effector in tool frame (6,).
                   Format: [moment, force]. Default is zero.

        Returns:
            Tuple of:
                - tau: Joint torques [N·m] (n,).
                - state: Updated DynamicsState with wrenches.
        """
        n = self.params.n_joints
        tau = np.zeros(n)

        # Initial wrench (end-effector)
        if F_tip is None:
            F_next = np.zeros(6)
        else:
            F_next = np.asarray(F_tip, dtype=np.float64).flatten()
            # Transform external wrench from tool frame to link n frame
            # F_n = [Ad_{T_{n+1,n}}]^T * F_tip
            Ad_T_ee = SE3.adjoint(self._ee_frame)
            F_next = Ad_T_ee.T @ F_next

        # Initialize wrenches list
        state.wrenches = [None] * n

        for i in range(n - 1, -1, -1):
            G_i = self._spatial_inertias[i]
            V_i = state.twists[i]
            Vdot_i = state.twist_dots[i]
            A_i = self._screw_axes[i]

            # Equation 8.53: F_i = [Ad_{T_{i+1,i}}]^T * F_{i+1}
            #                    + G_i * V̇_i - [ad_{V_i}]^T * G_i * V_i
            if i < n - 1:
                # Get transform from link i to link i+1
                T_ip1_i = state.transforms[i + 1]
                Ad_T_ip1_i = SE3.adjoint(T_ip1_i)
                F_propagated = Ad_T_ip1_i.T @ F_next
            else:
                # For last link, F_next is already the end-effector wrench
                F_propagated = F_next

            # Inertial wrench
            F_inertia = G_i @ Vdot_i

            # Coriolis/centrifugal wrench
            ad_V_i_T = SE3.adjoint_algebra(SE3.wedge(V_i)).T
            F_coriolis = ad_V_i_T @ (G_i @ V_i)

            F_i = F_propagated + F_inertia - F_coriolis
            state.wrenches[i] = F_i

            # Equation 8.54: τ_i = F_i^T * A_i
            tau[i] = F_i @ A_i

            # Update for next iteration (propagate wrench to parent)
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
                   Format: [moment, force]. Default is zero.

        Returns:
            tau: Joint torques [N·m] (n,).
        """
        from dynamics.newton_euler_joint_frame import NewtonEulerJointFrame

        q = np.asarray(q, dtype=np.float64).flatten()
        dq = np.asarray(dq, dtype=np.float64).flatten()
        ddq = np.asarray(ddq, dtype=np.float64).flatten()

        # Validate inputs
        n = self.params.n_joints
        if q.shape[0] != n or dq.shape[0] != n or ddq.shape[0] != n:
            raise ValueError(f"Expected arrays of length {n}")

        jf_dynamics = NewtonEulerJointFrame(gravity=self.gravity)
        return jf_dynamics.inverse_dynamics(q, dq, ddq, F_tip)

    def inverse_dynamics_full(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        ddq: np.ndarray,
        F_tip: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, DynamicsState]:
        """Compute inverse dynamics with full state information.

        Note: The torque is computed using the joint-frame implementation
        for accuracy. The state contains intermediate values from the
        internal CoM-based computation (for backward compatibility).

        Args:
            q: Joint positions [rad] (n,).
            dq: Joint velocities [rad/s] (n,).
            ddq: Joint accelerations [rad/s²] (n,).
            F_tip: External wrench at end-effector in tool frame (6,).

        Returns:
            Tuple of:
                - tau: Joint torques [N·m] (n,).
                - state: DynamicsState with all intermediate values.
        """
        # Use joint-frame implementation for accurate torque
        tau = self.inverse_dynamics(q, dq, ddq, F_tip)

        # Compute internal state for backward compatibility
        q = np.asarray(q, dtype=np.float64).flatten()
        dq = np.asarray(dq, dtype=np.float64).flatten()
        ddq = np.asarray(ddq, dtype=np.float64).flatten()

        state = self._forward_iterations(q, dq, ddq)
        _, state = self._backward_iterations(state, F_tip)

        return tau, state

    def gravity_torques(self, q: np.ndarray) -> np.ndarray:
        """Compute gravity compensation torques.

        Equivalent to inverse_dynamics with zero velocities and accelerations.

        Args:
            q: Joint positions [rad] (n,).

        Returns:
            tau_g: Gravity compensation torques [N·m] (n,).
        """
        from dynamics.newton_euler_joint_frame import NewtonEulerJointFrame

        jf_dynamics = NewtonEulerJointFrame(gravity=self.gravity)
        return jf_dynamics.gravity_torques(q)

    def get_end_effector_twist(
        self,
        q: np.ndarray,
        dq: np.ndarray,
    ) -> np.ndarray:
        """Compute end-effector twist (velocity) in internal CoM frame.

        WARNING: This returns the twist in the internal CoM-based coordinate
        system, which does NOT match the standard DH tool0 frame.
        Use get_tool0_twist() for the correct tool0 velocity.

        Args:
            q: Joint positions [rad] (n,).
            dq: Joint velocities [rad/s] (n,).

        Returns:
            V_ee: End-effector twist [ω, v] in internal CoM frame (6,).
        """
        q = np.asarray(q, dtype=np.float64).flatten()
        dq = np.asarray(dq, dtype=np.float64).flatten()
        ddq = np.zeros(self.params.n_joints)

        state = self._forward_iterations(q, dq, ddq)

        # Get twist of last link
        V_n = state.twists[-1]

        # Transform to end-effector frame
        Ad_T_ee = SE3.adjoint(self._ee_frame)
        V_ee = Ad_T_ee @ V_n

        return V_ee

    def get_tool0_twist(
        self,
        q: np.ndarray,
        dq: np.ndarray,
    ) -> np.ndarray:
        """Compute tool0 twist in standard DH tool0 frame.

        Uses the geometric Jacobian for accurate computation.
        The internal CoM-based frame chain differs from standard DH,
        making direct transformation unreliable for velocity.

        Args:
            q: Joint positions [rad] (n,).
            dq: Joint velocities [rad/s] (n,).

        Returns:
            V_tool0: Tool0 twist [ω, v] in standard DH tool0 frame (6,).
        """
        from dynamics.forward_kinematics import compute_tool0_twist_tool
        return compute_tool0_twist_tool(q, dq)

    def get_end_effector_acceleration(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        ddq: np.ndarray,
    ) -> np.ndarray:
        """Compute end-effector acceleration in internal CoM frame.

        WARNING: This returns the acceleration in the internal CoM-based
        coordinate system, which does NOT match the standard DH tool0 frame.
        Use get_tool0_acceleration() for the correct tool0 acceleration.

        Note: This returns the acceleration with gravity included as
        fictitious acceleration.

        Args:
            q: Joint positions [rad] (n,).
            dq: Joint velocities [rad/s] (n,).
            ddq: Joint accelerations [rad/s²] (n,).

        Returns:
            Vdot_ee: End-effector acceleration [α, a] in internal CoM frame (6,).
        """
        q = np.asarray(q, dtype=np.float64).flatten()
        dq = np.asarray(dq, dtype=np.float64).flatten()
        ddq = np.asarray(ddq, dtype=np.float64).flatten()

        state = self._forward_iterations(q, dq, ddq)

        # Get acceleration of last link
        Vdot_n = state.twist_dots[-1]

        # Transform to end-effector frame
        Ad_T_ee = SE3.adjoint(self._ee_frame)
        Vdot_ee = Ad_T_ee @ Vdot_n

        return Vdot_ee

    def get_tool0_acceleration(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        ddq: np.ndarray,
    ) -> np.ndarray:
        """Compute tool0 acceleration in standard DH tool0 frame.

        Uses the joint-frame Newton-Euler for accurate computation.
        The internal CoM-based frame chain differs from standard DH.

        Note: This returns the acceleration with gravity included as
        fictitious acceleration.

        Args:
            q: Joint positions [rad] (n,).
            dq: Joint velocities [rad/s] (n,).
            ddq: Joint accelerations [rad/s²] (n,).

        Returns:
            Vdot_tool0: Tool0 acceleration [α, a] in standard DH tool0 frame (6,).
        """
        from dynamics.newton_euler_joint_frame import NewtonEulerJointFrame
        jf_dynamics = NewtonEulerJointFrame(gravity=self.gravity)
        return jf_dynamics.get_tool0_acceleration(q, dq, ddq)

    def mass_matrix(self, q: np.ndarray) -> np.ndarray:
        """Compute the mass (inertia) matrix M(q).

        Uses the relation: τ = M(q)θ̈ + c(q, θ̇) + g(q)
        Computed by calling inverse_dynamics with unit accelerations.

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


def create_ur5e_dynamics(
    gravity: Optional[np.ndarray] = None,
) -> NewtonEulerDynamics:
    """Factory function to create UR5e dynamics instance.

    Args:
        gravity: Gravity vector in base frame. Default is [0, 0, -9.81].

    Returns:
        NewtonEulerDynamics instance for UR5e.
    """
    return NewtonEulerDynamics(params=UR5eParameters(), gravity=gravity)
