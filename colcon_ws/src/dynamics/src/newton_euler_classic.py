"""Classic Newton-Euler inverse dynamics implementation.

Traditional formulation using separate rotation matrices, position vectors,
angular velocities, and forces. Based on Craig's "Introduction to Robotics"
and Siciliano's "Robotics: Modelling, Planning and Control".

This implementation does NOT use the twist-wrench formulation.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from dynamics.ur5e_parameters import UR5eParameters


@dataclass
class ClassicDynamicsState:
    """State variables computed during classic Newton-Euler algorithm.

    Attributes:
        omega: List of angular velocities ω_i in frame {i} (3,).
        omega_dot: List of angular accelerations ω̇_i in frame {i} (3,).
        v_dot: List of linear accelerations v̇_i at frame origin in frame {i} (3,).
        v_c_dot: List of linear accelerations at CoM in frame {i} (3,).
        F: List of forces at CoM in frame {i} (3,).
        N: List of moments at CoM in frame {i} (3,).
        R: List of rotation matrices R_{i}^{i-1} (3, 3).
        p: List of position vectors p_{i}^{i-1} in frame {i} (3,).
    """

    omega: List[np.ndarray] = field(default_factory=list)
    omega_dot: List[np.ndarray] = field(default_factory=list)
    v_dot: List[np.ndarray] = field(default_factory=list)
    v_c_dot: List[np.ndarray] = field(default_factory=list)
    F: List[np.ndarray] = field(default_factory=list)
    N: List[np.ndarray] = field(default_factory=list)
    R: List[np.ndarray] = field(default_factory=list)
    p: List[np.ndarray] = field(default_factory=list)


def dh_transform(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    """Compute homogeneous transform using Modified DH convention (Craig).

    T = Rx(alpha_{i-1}) * Dx(a_{i-1}) * Rz(theta_i) * Dz(d_i)

    For Modified DH (Craig convention):
        T_{i}^{i-1} = [[cos(θ), -sin(θ), 0, a],
                       [sin(θ)cos(α), cos(θ)cos(α), -sin(α), -d*sin(α)],
                       [sin(θ)sin(α), cos(θ)sin(α), cos(α), d*cos(α)],
                       [0, 0, 0, 1]]

    Args:
        a: Link length a_{i-1}.
        d: Link offset d_i.
        alpha: Link twist alpha_{i-1}.
        theta: Joint angle theta_i.

    Returns:
        (4, 4) homogeneous transformation matrix T_{i}^{i-1}.
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    T = np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -d * sa],
        [st * sa, ct * sa, ca, d * ca],
        [0, 0, 0, 1]
    ], dtype=np.float64)

    return T


class NewtonEulerClassic:
    """Classic Newton-Euler inverse dynamics for UR5e robot.

    Implements the traditional Newton-Euler algorithm using separate
    rotation matrices, position vectors, angular velocities, and forces.

    This formulation expresses quantities in individual link frames and
    uses the recursive outward-inward propagation scheme.

    The equations are (for revolute joints, joint axis z):

    Forward (outward) iteration (i = 1 to n):
        ω_i = R_{i}^{i-1} * ω_{i-1} + θ̇_i * z_i
        ω̇_i = R_{i}^{i-1} * ω̇_{i-1} + R_{i}^{i-1} * ω_{i-1} × θ̇_i * z_i + θ̈_i * z_i
        v̇_i = R_{i}^{i-1} * (ω̇_{i-1} × p_{i}^{i-1} + ω_{i-1} × (ω_{i-1} × p_{i}^{i-1}) + v̇_{i-1})
        v̇_{c,i} = ω̇_i × p_{c,i} + ω_i × (ω_i × p_{c,i}) + v̇_i
        F_i = m_i * v̇_{c,i}
        N_i = I_i * ω̇_i + ω_i × (I_i * ω_i)

    Backward (inward) iteration (i = n to 1):
        f_i = R_{i+1}^{i}^T * f_{i+1} + F_i
        n_i = N_i + R_{i+1}^{i}^T * n_{i+1} + p_{c,i} × F_i + p_{i+1}^{i} × R_{i+1}^{i}^T * f_{i+1}
        τ_i = n_i^T * z_i

    Attributes:
        params: UR5e kinematic and dynamic parameters.
        gravity: Gravity vector in base frame [m/s²].
    """

    def __init__(
        self,
        params: Optional[UR5eParameters] = None,
        gravity: Optional[np.ndarray] = None,
    ) -> None:
        """Initialize classic Newton-Euler dynamics.

        Args:
            params: UR5e parameters. If None, uses default parameters.
            gravity: Gravity vector in base frame. Default is [0, 0, -9.81].
        """
        self.params = params if params is not None else UR5eParameters()

        if gravity is None:
            self.gravity = np.array([0.0, 0.0, -9.81])
        else:
            self.gravity = np.asarray(gravity, dtype=np.float64).flatten()

        # Joint axis in local frame (z-axis for revolute joints)
        self.z_axis = np.array([0.0, 0.0, 1.0])

    def _get_dh_transform(self, i: int, theta_i: float) -> np.ndarray:
        """Get DH transform for joint i.

        Args:
            i: Joint index (0 to n-1).
            theta_i: Joint angle [rad].

        Returns:
            (4, 4) transformation matrix T_{i}^{i-1}.
        """
        if i == 0:
            # For first joint, use its own DH parameters
            a = 0.0
            d = self.params.dh_params[0, 1]
            alpha = self.params.dh_params[0, 2]
        else:
            # Use previous link's a and alpha, current link's d
            a = self.params.dh_params[i - 1, 0]
            alpha = self.params.dh_params[i - 1, 2]
            d = self.params.dh_params[i, 1]

        return dh_transform(a, d, alpha, theta_i)

    def _forward_iterations(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        ddq: np.ndarray,
    ) -> ClassicDynamicsState:
        """Perform forward (outward) iterations.

        Computes angular velocities, angular accelerations, and linear
        accelerations for each link frame.

        Args:
            q: Joint positions [rad] (n,).
            dq: Joint velocities [rad/s] (n,).
            ddq: Joint accelerations [rad/s²] (n,).

        Returns:
            ClassicDynamicsState with velocities and accelerations.
        """
        n = self.params.n_joints
        state = ClassicDynamicsState()

        # Initial conditions (base frame)
        omega_prev = np.zeros(3)
        omega_dot_prev = np.zeros(3)
        v_dot_prev = -self.gravity  # Gravity as fictitious acceleration

        for i in range(n):
            theta_i = q[i]
            dtheta_i = dq[i]
            ddtheta_i = ddq[i]

            # Get transformation from frame {i-1} to frame {i}
            T = self._get_dh_transform(i, theta_i)
            R_i_im1 = T[:3, :3]  # R_{i}^{i-1}
            p_i_im1 = T[:3, 3]   # p_{i}^{i-1} in frame {i}

            state.R.append(R_i_im1)
            state.p.append(p_i_im1)

            # Angular velocity: ω_i = R * ω_{i-1} + θ̇ * z
            omega_i = R_i_im1 @ omega_prev + dtheta_i * self.z_axis
            state.omega.append(omega_i)

            # Angular acceleration: ω̇_i = R * ω̇_{i-1} + R * ω_{i-1} × θ̇ * z + θ̈ * z
            omega_dot_i = (
                R_i_im1 @ omega_dot_prev +
                np.cross(R_i_im1 @ omega_prev, dtheta_i * self.z_axis) +
                ddtheta_i * self.z_axis
            )
            state.omega_dot.append(omega_dot_i)

            # Linear acceleration at frame origin:
            # v̇_i = R * (ω̇_{i-1} × p + ω_{i-1} × (ω_{i-1} × p) + v̇_{i-1})
            # Note: p_{i}^{i-1} expressed in frame {i-1}
            p_in_im1 = R_i_im1.T @ p_i_im1  # Transform to frame {i-1}
            v_dot_i = R_i_im1 @ (
                np.cross(omega_dot_prev, p_in_im1) +
                np.cross(omega_prev, np.cross(omega_prev, p_in_im1)) +
                v_dot_prev
            )
            state.v_dot.append(v_dot_i)

            # Linear acceleration at CoM:
            # v̇_{c,i} = ω̇_i × p_c + ω_i × (ω_i × p_c) + v̇_i
            p_c = self.params.link_com_positions[i]
            v_c_dot_i = (
                np.cross(omega_dot_i, p_c) +
                np.cross(omega_i, np.cross(omega_i, p_c)) +
                v_dot_i
            )
            state.v_c_dot.append(v_c_dot_i)

            # Force at CoM: F_i = m * v̇_c
            m_i = self.params.link_masses[i]
            F_i = m_i * v_c_dot_i
            state.F.append(F_i)

            # Moment at CoM: N_i = I * ω̇ + ω × (I * ω)
            I_i = self.params.link_inertias[i]
            N_i = I_i @ omega_dot_i + np.cross(omega_i, I_i @ omega_i)
            state.N.append(N_i)

            # Update for next iteration
            omega_prev = omega_i
            omega_dot_prev = omega_dot_i
            v_dot_prev = v_dot_i

        return state

    def _backward_iterations(
        self,
        state: ClassicDynamicsState,
        f_tip: Optional[np.ndarray] = None,
        n_tip: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Perform backward (inward) iterations.

        Computes joint torques from forces and moments.

        Args:
            state: ClassicDynamicsState from forward iterations.
            f_tip: External force at end-effector in tool frame (3,).
            n_tip: External moment at end-effector in tool frame (3,).

        Returns:
            tau: Joint torques [N·m] (n,).
        """
        n = self.params.n_joints
        tau = np.zeros(n)

        # Initial conditions (end-effector)
        if f_tip is None:
            f_next = np.zeros(3)
        else:
            f_next = np.asarray(f_tip, dtype=np.float64).flatten()

        if n_tip is None:
            n_next = np.zeros(3)
        else:
            n_next = np.asarray(n_tip, dtype=np.float64).flatten()

        for i in range(n - 1, -1, -1):
            F_i = state.F[i]
            N_i = state.N[i]
            p_c = self.params.link_com_positions[i]

            if i < n - 1:
                # Get rotation from frame {i+1} to frame {i}
                R_ip1_i = state.R[i + 1]  # R_{i+1}^{i}
                R_i_ip1 = R_ip1_i.T        # R_{i}^{i+1}

                # Position from frame {i} origin to frame {i+1} origin
                p_ip1_i = state.p[i + 1]  # In frame {i+1}
                p_ip1_in_i = R_i_ip1 @ p_ip1_i  # Transform to frame {i}

                # Force propagation: f_i = R^T * f_{i+1} + F_i
                f_i = R_i_ip1 @ f_next + F_i

                # Moment propagation:
                # n_i = N_i + R^T * n_{i+1} + p_c × F_i + p_{i+1} × R^T * f_{i+1}
                n_i = (
                    N_i +
                    R_i_ip1 @ n_next +
                    np.cross(p_c, F_i) +
                    np.cross(p_ip1_in_i, R_i_ip1 @ f_next)
                )
            else:
                # Last link: f_i = F_i + f_tip, n_i = N_i + n_tip + p_c × F_i
                # Need to transform f_tip and n_tip from tool frame to link 6 frame
                # For simplicity, assume tool frame = link 6 frame offset
                d6 = self.params.dh_params[5, 1]
                p_ee = np.array([0, 0, d6]) - self.params.link_com_positions[5]

                f_i = F_i + f_next
                n_i = N_i + n_next + np.cross(p_c, F_i) + np.cross(p_ee, f_next)

            # Joint torque: τ_i = n_i^T * z
            tau[i] = np.dot(n_i, self.z_axis)

            # Update for next iteration
            f_next = f_i
            n_next = n_i

        return tau

    def inverse_dynamics(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        ddq: np.ndarray,
        f_tip: Optional[np.ndarray] = None,
        n_tip: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Compute inverse dynamics: joint torques given motion.

        Args:
            q: Joint positions [rad] (n,).
            dq: Joint velocities [rad/s] (n,).
            ddq: Joint accelerations [rad/s²] (n,).
            f_tip: External force at end-effector in tool frame (3,).
            n_tip: External moment at end-effector in tool frame (3,).

        Returns:
            tau: Joint torques [N·m] (n,).
        """
        q = np.asarray(q, dtype=np.float64).flatten()
        dq = np.asarray(dq, dtype=np.float64).flatten()
        ddq = np.asarray(ddq, dtype=np.float64).flatten()

        n = self.params.n_joints
        if q.shape[0] != n or dq.shape[0] != n or ddq.shape[0] != n:
            raise ValueError(f"Expected arrays of length {n}")

        state = self._forward_iterations(q, dq, ddq)
        tau = self._backward_iterations(state, f_tip, n_tip)

        return tau

    def gravity_torques(self, q: np.ndarray) -> np.ndarray:
        """Compute gravity compensation torques.

        Args:
            q: Joint positions [rad] (n,).

        Returns:
            tau_g: Gravity compensation torques [N·m] (n,).
        """
        n = self.params.n_joints
        zero = np.zeros(n)
        return self.inverse_dynamics(q, zero, zero)

    def mass_matrix(self, q: np.ndarray) -> np.ndarray:
        """Compute the mass (inertia) matrix M(q).

        Args:
            q: Joint positions [rad] (n,).

        Returns:
            M: Mass matrix (n, n).
        """
        n = self.params.n_joints
        q = np.asarray(q, dtype=np.float64).flatten()
        zero_dq = np.zeros(n)

        # Temporarily disable gravity
        original_gravity = self.gravity.copy()
        self.gravity = np.zeros(3)

        M = np.zeros((n, n))
        for i in range(n):
            ddq = np.zeros(n)
            ddq[i] = 1.0
            M[:, i] = self.inverse_dynamics(q, zero_dq, ddq)

        # Restore gravity
        self.gravity = original_gravity

        return M

    def coriolis_vector(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """Compute the Coriolis/centrifugal vector c(q, θ̇).

        Args:
            q: Joint positions [rad] (n,).
            dq: Joint velocities [rad/s] (n,).

        Returns:
            c: Coriolis/centrifugal torques (n,).
        """
        n = self.params.n_joints
        zero_ddq = np.zeros(n)

        # Temporarily disable gravity
        original_gravity = self.gravity.copy()
        self.gravity = np.zeros(3)

        c = self.inverse_dynamics(q, dq, zero_ddq)

        # Restore gravity
        self.gravity = original_gravity

        return c


def create_ur5e_classic_dynamics(
    gravity: Optional[np.ndarray] = None,
) -> NewtonEulerClassic:
    """Factory function to create UR5e classic dynamics instance.

    Args:
        gravity: Gravity vector in base frame. Default is [0, 0, -9.81].

    Returns:
        NewtonEulerClassic instance for UR5e.
    """
    return NewtonEulerClassic(params=UR5eParameters(), gravity=gravity)
