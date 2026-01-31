"""Newton-Euler inverse dynamics using joint frames (standard DH).

This implementation places link frames at joint locations rather than
centers of mass, matching standard DH conventions.

Based on Lynch and Park (2017), Section 8.3.2, but adapted for joint frames.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
from pymlg import SE3, SO3

from dynamics.forward_kinematics import (
    UR5E_DH_PARAMS,
    dh_transform_standard,
)
from dynamics.ur5e_parameters import create_ur5e_parameters



@dataclass
class JointFrameDynamicsState:
    """State variables computed during Newton-Euler algorithm.

    All quantities are expressed in joint frames (standard DH).

    Attributes:
        twists: List of body twists V_i for each joint frame (6,).
        twist_dots: List of body twist derivatives V̇_i for each joint frame (6,).
        wrenches: List of body wrenches F_i for each joint frame (6,).
        transforms: List of transforms T_{i,i-1} from frame i-1 to frame i (4, 4).
    """

    twists: List[np.ndarray] = field(default_factory=list)
    twist_dots: List[np.ndarray] = field(default_factory=list)
    wrenches: List[np.ndarray] = field(default_factory=list)
    transforms: List[np.ndarray] = field(default_factory=list)


# Singleton instance of UR5e parameters (loaded once)
_ur5e_params = None


def _get_ur5e_params():
    """Get cached UR5e parameters instance."""
    global _ur5e_params
    if _ur5e_params is None:
        _ur5e_params = create_ur5e_parameters()
    return _ur5e_params


def compute_spatial_inertia_at_joint(
    mass: float,
    com_position: np.ndarray,
    inertia_at_com: np.ndarray,
) -> np.ndarray:
    """Compute spatial inertia matrix expressed at joint frame.

    Uses the parallel axis theorem to transform inertia from CoM to joint.

    The spatial inertia matrix G_j at joint frame is related to G_c at CoM by:
        G_j = [Ad_{T_{c,j}}]^T G_c [Ad_{T_{c,j}}]

    where T_{c,j} is the transform from joint to CoM frame.

    For pure translation p from joint to CoM:
        G_j = [[I_c + m*[p]×[p]×^T, m*[p]×],
               [m*[p]×^T,           m*I_3]]

    Args:
        mass: Link mass [kg].
        com_position: CoM position relative to joint frame [m] (3,).
        inertia_at_com: Inertia tensor at CoM (3, 3).

    Returns:
        G: (6, 6) spatial inertia matrix at joint frame.
    """
    p = com_position
    p_skew = SO3.wedge(p)

    # Parallel axis theorem for inertia
    # I_j = I_c + m * ([p]× @ [p]×^T)  (note: [p]×^T = -[p]×)
    I_j = inertia_at_com + mass * (p_skew @ p_skew.T)

    # Spatial inertia matrix in Lynch & Park convention
    # G = [[I, m*[p]×], [m*[p]×^T, m*I_3]]
    G = np.zeros((6, 6))
    G[:3, :3] = I_j
    G[:3, 3:] = mass * p_skew
    G[3:, :3] = mass * p_skew.T
    G[3:, 3:] = mass * np.eye(3)

    return G


class NewtonEulerJointFrame:
    """Newton-Euler inverse dynamics using joint frames.

    This implementation uses standard DH convention with frames placed
    at joint locations. All velocities and accelerations are computed
    in joint frames.

    Attributes:
        gravity: Gravity vector in base frame [m/s²].
        spatial_inertias: List of (6, 6) spatial inertia matrices at joint frames.
    """

    def __init__(self, gravity: Optional[np.ndarray] = None) -> None:
        """Initialize Newton-Euler dynamics.

        Args:
            gravity: Gravity vector in base frame. Default is [0, 0, -9.81].
        """
        if gravity is None:
            self.gravity = np.array([0.0, 0.0, -9.81])
        else:
            self.gravity = np.asarray(gravity, dtype=np.float64).flatten()

        # Pre-compute spatial inertia matrices at joint frames
        params = _get_ur5e_params()
        self.spatial_inertias = []
        for i in range(6):
            G = compute_spatial_inertia_at_joint(
                mass=params.link_masses[i],
                com_position=params.link_com_positions[i],
                inertia_at_com=params.link_inertias[i],
            )
            self.spatial_inertias.append(G)

        # Pre-compute screw axes for each joint
        # In DH convention, joint i rotates about z-axis of frame i-1
        # We need to express this axis in frame i coordinates
        self._screw_axes = self._compute_screw_axes()

    def _compute_screw_axes(self) -> List[np.ndarray]:
        """Compute screw axes A_i for each joint in frame i coordinates.

        In standard DH convention:
        - Joint i rotates about the z-axis of frame i-1
        - The screw axis A_i must be expressed in frame i

        For a revolute joint with axis passing through point r (not origin):
            A_i = [ω_i, v_i] where:
            - ω_i = R_{i-1,i}^T @ [0, 0, 1]  (axis direction in frame i)
            - v_i = -ω_i × r_i  (where r_i is a point on the axis in frame i)

        The z-axis of frame i-1 passes through the origin of frame i-1.
        In frame i coordinates, this origin is at -R^T @ p (translation part).

        Returns:
            List of (6,) screw axes for joints 1 to 6.
        """
        axes = []
        z_axis = np.array([0.0, 0.0, 1.0])

        for i in range(6):
            # Get DH parameters
            d = UR5E_DH_PARAMS[i, 0]
            a = UR5E_DH_PARAMS[i, 1]
            alpha = UR5E_DH_PARAMS[i, 2]

            # At theta=0, compute T_{i-1,i}
            T_im1_i = dh_transform_standard(d, a, alpha, 0.0)
            R_im1_i = T_im1_i[:3, :3]
            p_im1_i = T_im1_i[:3, 3]

            # Joint axis direction in frame i = R^T @ z
            omega_i = R_im1_i.T @ z_axis

            # Point on joint axis in frame i = origin of frame i-1 in frame i
            # r_i = -R^T @ p (inverse transform)
            r_i = -R_im1_i.T @ p_im1_i

            # Linear velocity part: v = -ω × r
            v_i = -np.cross(omega_i, r_i)

            A_i = np.concatenate([omega_i, v_i])
            axes.append(A_i)

        return axes

    def _compute_transform(self, i: int, theta: float) -> np.ndarray:
        """Compute T_{i,i-1}: transform from frame i-1 to frame i.

        Uses standard DH convention.

        Args:
            i: Joint index (0 to 5).
            theta: Joint angle [rad].

        Returns:
            T_{i,i-1}: (4, 4) transformation matrix.
        """
        d = UR5E_DH_PARAMS[i, 0]
        a = UR5E_DH_PARAMS[i, 1]
        alpha = UR5E_DH_PARAMS[i, 2]

        # Standard DH gives T_{i-1,i} (from i to i-1)
        T_im1_i = dh_transform_standard(d, a, alpha, theta)

        # We need T_{i,i-1} (from i-1 to i)
        T_i_im1 = SE3.inverse(T_im1_i)

        return T_i_im1

    def _forward_iterations(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        ddq: np.ndarray,
    ) -> JointFrameDynamicsState:
        """Perform forward iterations of Newton-Euler algorithm.

        Computes transforms, twists, and twist derivatives for each joint frame.

        Equations (Lynch & Park 8.50-8.52):
            T_{i,i-1} = from DH parameters
            V_i = [Ad_{T_{i,i-1}}] * V_{i-1} + A_i * θ̇_i
            V̇_i = [Ad_{T_{i,i-1}}] * V̇_{i-1} + [ad_{V_i}] * A_i * θ̇_i + A_i * θ̈_i

        Args:
            q: Joint positions [rad] (6,).
            dq: Joint velocities [rad/s] (6,).
            ddq: Joint accelerations [rad/s²] (6,).

        Returns:
            JointFrameDynamicsState with twists, twist_dots, and transforms.
        """
        state = JointFrameDynamicsState()

        # Initial conditions (base frame)
        # V_0 = 0 (base is stationary)
        # V̇_0 = -[0, g] (accounts for gravity as fictitious acceleration)
        V_prev = np.zeros(6)
        Vdot_prev = np.concatenate([np.zeros(3), -self.gravity])

        for i in range(6):
            theta_i = q[i]
            dtheta_i = dq[i]
            ddtheta_i = ddq[i]
            A_i = self._screw_axes[i]

            # Compute T_{i,i-1}
            T_i_im1 = self._compute_transform(i, theta_i)
            state.transforms.append(T_i_im1)

            # Adjoint of transform
            Ad_T = SE3.adjoint(T_i_im1)

            # V_i = [Ad_{T_{i,i-1}}] * V_{i-1} + A_i * θ̇_i
            V_i = Ad_T @ V_prev + A_i * dtheta_i
            state.twists.append(V_i)

            # V̇_i = [Ad_{T_{i,i-1}}] * V̇_{i-1} + [ad_{V_i}] * A_i * θ̇_i + A_i * θ̈_i
            ad_V_i = SE3.adjoint_algebra(SE3.wedge(V_i))
            Vdot_i = Ad_T @ Vdot_prev + ad_V_i @ A_i * dtheta_i + A_i * ddtheta_i
            state.twist_dots.append(Vdot_i)

            # Update for next iteration
            V_prev = V_i
            Vdot_prev = Vdot_i

        return state

    def _backward_iterations(
        self,
        state: JointFrameDynamicsState,
        F_tip: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, JointFrameDynamicsState]:
        """Perform backward iterations of Newton-Euler algorithm.

        Equations (Lynch & Park 8.53-8.54):
            F_i = [Ad_{T_{i+1,i}}]^T * F_{i+1} + G_i * V̇_i - [ad_{V_i}]^T * G_i * V_i
            τ_i = F_i^T * A_i

        Args:
            state: JointFrameDynamicsState from forward iterations.
            F_tip: External wrench at tool0 in tool0 frame (6,).

        Returns:
            Tuple of (tau, state).
        """
        tau = np.zeros(6)

        # Initial wrench at tool0
        if F_tip is None:
            F_next = np.zeros(6)
        else:
            F_next = np.asarray(F_tip, dtype=np.float64).flatten()

        state.wrenches = [None] * 6

        for i in range(5, -1, -1):
            G_i = self.spatial_inertias[i]
            V_i = state.twists[i]
            Vdot_i = state.twist_dots[i]
            A_i = self._screw_axes[i]

            # F_propagated from child link
            if i < 5:
                # T_{i+1,i} from current state (which has T_{i,i-1} for each i)
                # state.transforms[i+1] = T_{i+1,i}
                T_ip1_i = state.transforms[i + 1]
                Ad_T_ip1_i = SE3.adjoint(T_ip1_i)
                F_propagated = Ad_T_ip1_i.T @ F_next
            else:
                # For link 6, F_tip is already in tool0 = frame 6 coordinates
                # In standard DH, frame 6 IS tool0 (d6 is already applied)
                F_propagated = F_next

            # Inertial wrench
            F_inertia = G_i @ Vdot_i

            # Coriolis/centrifugal wrench
            ad_V_i_T = SE3.adjoint_algebra(SE3.wedge(V_i)).T
            F_coriolis = ad_V_i_T @ (G_i @ V_i)

            F_i = F_propagated + F_inertia - F_coriolis
            state.wrenches[i] = F_i

            # τ_i = F_i^T * A_i
            tau[i] = F_i @ A_i

            # Update for next iteration
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

        Args:
            q: Joint positions [rad] (6,).
            dq: Joint velocities [rad/s] (6,).
            ddq: Joint accelerations [rad/s²] (6,).
            F_tip: External wrench at tool0 in tool0 frame (6,).

        Returns:
            tau: Joint torques [N·m] (6,).
        """
        q = np.asarray(q, dtype=np.float64).flatten()
        dq = np.asarray(dq, dtype=np.float64).flatten()
        ddq = np.asarray(ddq, dtype=np.float64).flatten()

        state = self._forward_iterations(q, dq, ddq)
        tau, _ = self._backward_iterations(state, F_tip)

        return tau

    def get_tool0_twist(
        self,
        q: np.ndarray,
        dq: np.ndarray,
    ) -> np.ndarray:
        """Compute tool0 twist in tool0 frame.

        This returns the same result as geometric Jacobian since we use
        standard DH frames.

        Args:
            q: Joint positions [rad] (6,).
            dq: Joint velocities [rad/s] (6,).

        Returns:
            V_tool0: Twist [ω, v] in tool0 frame (6,).
        """
        q = np.asarray(q, dtype=np.float64).flatten()
        dq = np.asarray(dq, dtype=np.float64).flatten()

        state = self._forward_iterations(q, dq, np.zeros(6))

        # V_6 is the twist of frame 6 (joint 6 frame) in frame 6 coordinates
        V_6 = state.twists[-1]

        # tool0 is at offset d6 along z from joint 6 frame
        # V_tool0 = [Ad_{T_{tool0,6}}] V_6
        d6 = UR5E_DH_PARAMS[5, 1]  # 0.0823 but this is already included in frame 6
        # Actually, in standard DH, frame 6 already has d6 applied
        # So tool0 = frame 6 position

        # Wait, let me check this more carefully...
        # In forward_kinematics.py, UR5E_DH_PARAMS[5] = [0.0823, 0, 0]
        # This means d6 = 0.0823, a6 = 0, alpha6 = 0
        # So the final frame (frame 6) is already at tool0 position

        # Therefore V_6 = V_tool0 in frame 6 = tool0 coordinates
        return V_6

    def get_tool0_acceleration(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        ddq: np.ndarray,
    ) -> np.ndarray:
        """Compute tool0 acceleration in tool0 frame.

        Note: This includes gravity as fictitious acceleration.
        To get true acceleration, add gravity to the linear part.

        Args:
            q: Joint positions [rad] (6,).
            dq: Joint velocities [rad/s] (6,).
            ddq: Joint accelerations [rad/s²] (6,).

        Returns:
            Vdot_tool0: Acceleration [α, a] in tool0 frame (6,).
        """
        q = np.asarray(q, dtype=np.float64).flatten()
        dq = np.asarray(dq, dtype=np.float64).flatten()
        ddq = np.asarray(ddq, dtype=np.float64).flatten()

        state = self._forward_iterations(q, dq, ddq)

        # Same reasoning as get_tool0_twist
        return state.twist_dots[-1]

    def gravity_torques(self, q: np.ndarray) -> np.ndarray:
        """Compute gravity compensation torques.

        Args:
            q: Joint positions [rad] (6,).

        Returns:
            tau_g: Gravity compensation torques [N·m] (6,).
        """
        return self.inverse_dynamics(q, np.zeros(6), np.zeros(6))

    def mass_matrix(self, q: np.ndarray) -> np.ndarray:
        """Compute the mass matrix M(q).

        Args:
            q: Joint positions [rad] (6,).

        Returns:
            M: Mass matrix (6, 6).
        """
        q = np.asarray(q, dtype=np.float64).flatten()

        # Temporarily disable gravity
        original_gravity = self.gravity.copy()
        self.gravity = np.zeros(3)

        M = np.zeros((6, 6))
        for i in range(6):
            ddq = np.zeros(6)
            ddq[i] = 1.0
            M[:, i] = self.inverse_dynamics(q, np.zeros(6), ddq)

        self.gravity = original_gravity
        return M

    def coriolis_vector(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """Compute the Coriolis/centrifugal vector c(q, θ̇).

        Args:
            q: Joint positions [rad] (6,).
            dq: Joint velocities [rad/s] (6,).

        Returns:
            c: Coriolis/centrifugal torques (6,).
        """
        original_gravity = self.gravity.copy()
        self.gravity = np.zeros(3)

        c = self.inverse_dynamics(q, dq, np.zeros(6))

        self.gravity = original_gravity
        return c


def create_ur5e_joint_frame_dynamics(
    gravity: Optional[np.ndarray] = None,
) -> NewtonEulerJointFrame:
    """Factory function to create UR5e joint-frame dynamics instance.

    Args:
        gravity: Gravity vector in base frame. Default is [0, 0, -9.81].

    Returns:
        NewtonEulerJointFrame instance for UR5e.
    """
    return NewtonEulerJointFrame(gravity=gravity)
