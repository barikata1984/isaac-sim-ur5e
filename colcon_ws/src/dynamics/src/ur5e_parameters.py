"""UR5e robot kinematic and dynamic parameters.

Based on:
- UR5e DH parameters (Modified DH, Craig convention)
- Inertial properties from URDF specifications

Note: Link frames {i} are placed at the centers of mass of each link.
"""

from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np
from pymlg import SE3

from dynamics.spatial_inertia import SpatialInertia


@dataclass
class UR5eParameters:
    """UR5e robot kinematic and dynamic parameters.

    The Newton-Euler algorithm requires:
    - M_{i,i-1}: Configuration of frame {i-1} in frame {i} when theta_i = 0
    - A_i: Screw axis of joint i expressed in frame {i}
    - G_i: Spatial inertia matrix of link i

    Coordinate convention:
    - Frames are placed at link centers of mass
    - All quantities expressed in SI units (meters, kg, rad)
    """

    # Number of joints
    n_joints: int = 6

    # UR5e DH parameters (Modified DH, Craig convention)
    # [a_i, d_i, alpha_i] for i = 1 to 6
    # These relate consecutive joint frames, not CoM frames
    dh_params: np.ndarray = field(default_factory=lambda: np.array([
        [0.0,       0.089159,   np.pi / 2],   # Joint 1
        [-0.425,    0.0,        0.0],          # Joint 2
        [-0.392,    0.0,        0.0],          # Joint 3
        [0.0,       0.10915,    np.pi / 2],    # Joint 4
        [0.0,       0.09465,   -np.pi / 2],    # Joint 5
        [0.0,       0.0823,     0.0],          # Joint 6
    ]))

    # Link masses [kg] - approximate values for UR5e
    link_masses: np.ndarray = field(default_factory=lambda: np.array([
        3.7,    # Link 1 (shoulder)
        8.393,  # Link 2 (upper arm)
        2.275,  # Link 3 (forearm)
        1.219,  # Link 4 (wrist 1)
        1.219,  # Link 5 (wrist 2)
        0.1879, # Link 6 (wrist 3)
    ]))

    # Center of mass positions relative to link frame [m]
    # [x, y, z] in link frame when theta_i = 0
    link_com_positions: np.ndarray = field(default_factory=lambda: np.array([
        [0.0, -0.02561, 0.00193],      # Link 1
        [-0.2125, 0.0, 0.11336],       # Link 2
        [-0.15, 0.0, 0.0265],          # Link 3
        [0.0, -0.0018, 0.01634],       # Link 4
        [0.0, 0.0018, 0.01634],        # Link 5
        [0.0, 0.0, -0.001159],         # Link 6
    ]))

    # Link inertia tensors [kg*m^2] at center of mass, aligned with link frame
    # [[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]]
    link_inertias: np.ndarray = field(default_factory=lambda: np.array([
        # Link 1
        [[0.010267, 0.0, 0.0],
         [0.0, 0.010267, 0.0],
         [0.0, 0.0, 0.00666]],
        # Link 2
        [[0.22689, 0.0, 0.0],
         [0.0, 0.22689, 0.0],
         [0.0, 0.0, 0.0151074]],
        # Link 3
        [[0.049443, 0.0, 0.0],
         [0.0, 0.049443, 0.0],
         [0.0, 0.0, 0.004095]],
        # Link 4
        [[0.111172, 0.0, 0.0],
         [0.0, 0.111172, 0.0],
         [0.0, 0.0, 0.21942]],
        # Link 5
        [[0.111172, 0.0, 0.0],
         [0.0, 0.111172, 0.0],
         [0.0, 0.0, 0.21942]],
        # Link 6
        [[0.0171364, 0.0, 0.0],
         [0.0, 0.0171364, 0.0],
         [0.0, 0.0, 0.033822]],
    ]))

    def get_spatial_inertia(self, link_index: int) -> SpatialInertia:
        """Get spatial inertia matrix for a link.

        Args:
            link_index: 0-indexed link number (0 to 5).

        Returns:
            SpatialInertia object for the link.
        """
        if not 0 <= link_index < self.n_joints:
            raise ValueError(f"link_index must be 0-5, got {link_index}")
        return SpatialInertia(
            I_b=self.link_inertias[link_index].copy(),
            mass=self.link_masses[link_index],
        )

    def get_spatial_inertia_matrices(self) -> List[np.ndarray]:
        """Get all spatial inertia matrices G_i.

        Returns:
            List of (6, 6) spatial inertia matrices for links 1 to n.
        """
        return [self.get_spatial_inertia(i).to_matrix() for i in range(self.n_joints)]

    def get_screw_axes(self) -> List[np.ndarray]:
        """Get screw axes A_i expressed in link frame {i}.

        For revolute joints, the screw axis is:
            A_i = [omega_i, -omega_i x r_i]

        where omega_i is the joint axis direction and r_i is the position
        of the joint origin relative to the link CoM.

        Returns:
            List of (6,) screw axes for joints 1 to n.
        """
        axes = []
        for i in range(self.n_joints):
            # All UR5e joints are revolute about z-axis in local frame
            omega = np.array([0.0, 0.0, 1.0])

            # Position of joint origin relative to link CoM (negative of CoM position)
            # This gives us r_i in the link frame
            r = -self.link_com_positions[i]

            # For revolute joint: v = -omega x r
            v = -np.cross(omega, r)

            A_i = np.concatenate([omega, v])
            axes.append(A_i)
        return axes

    def get_home_configurations(self) -> List[np.ndarray]:
        """Get M_{i,i-1}: configuration of {i-1} in {i} when theta_i = 0.

        This transforms coordinates from frame {i-1} (parent link CoM frame)
        to frame {i} (current link CoM frame).

        Returns:
            List of (4, 4) transformation matrices M_{i,i-1} for i = 1 to n.
        """
        M_list = []

        # First, compute joint-to-joint transforms using DH parameters
        # Then adjust for CoM frame positions

        for i in range(self.n_joints):
            if i == 0:
                # M_{1,0}: Base frame to link 1 CoM frame
                # Link 1 joint is at origin, CoM is offset
                # We need T_1,0 = T_joint1_base * T_com1_joint1
                d1 = self.dh_params[0, 1]  # d = 0.089159
                alpha1 = self.dh_params[0, 2]  # alpha = pi/2

                # Joint 1 frame has z-axis along joint axis
                # Apply DH transform from base to joint 1 frame
                # Then offset to CoM
                R_x_alpha = np.array([
                    [1, 0, 0],
                    [0, np.cos(alpha1), -np.sin(alpha1)],
                    [0, np.sin(alpha1), np.cos(alpha1)],
                ])
                p_joint = np.array([0, 0, d1])
                T_joint = SE3.from_components(R_x_alpha, p_joint)

                com_offset = self.link_com_positions[i]
                T_com = SE3.from_components(np.eye(3), com_offset)

                # M_{1,0} = T_com^{-1} @ T_joint (transform from base to CoM1)
                # But we want {i-1} in {i}, so T_{i,i-1}
                M = SE3.inverse(T_com @ SE3.inverse(T_joint))
            else:
                # For subsequent links, use DH parameters to compute transforms
                a_im1 = self.dh_params[i - 1, 0]
                d_im1 = self.dh_params[i - 1, 1]
                alpha_im1 = self.dh_params[i - 1, 2]

                a_i = self.dh_params[i, 0]
                d_i = self.dh_params[i, 1]
                alpha_i = self.dh_params[i, 2]

                # DH transform from joint i-1 to joint i (at theta=0)
                # T_{j_i, j_{i-1}} using Modified DH convention
                ca = np.cos(alpha_im1)
                sa = np.sin(alpha_im1)

                # Modified DH: T = Rx(alpha_{i-1}) * Dx(a_{i-1}) * Rz(theta_i) * Dz(d_i)
                # At theta_i = 0:
                R_x = np.array([
                    [1, 0, 0],
                    [0, ca, -sa],
                    [0, sa, ca],
                ])

                p = np.array([a_im1, -sa * d_i, ca * d_i])
                T_joint_to_joint = SE3.from_components(R_x, p)

                # CoM offsets
                com_prev = self.link_com_positions[i - 1]
                com_curr = self.link_com_positions[i]

                # T from previous CoM to current CoM
                # T_{com_i, com_{i-1}} = T_{com_i, j_i} @ T_{j_i, j_{i-1}} @ T_{j_{i-1}, com_{i-1}}
                T_com_prev_to_joint_prev = SE3.from_components(np.eye(3), -com_prev)
                T_joint_curr_to_com_curr = SE3.from_components(np.eye(3), com_curr)

                T_com_curr_from_com_prev = T_joint_curr_to_com_curr @ T_joint_to_joint @ T_com_prev_to_joint_prev

                # M_{i,i-1} transforms from {i-1} to {i}
                M = SE3.inverse(T_com_curr_from_com_prev)

            M_list.append(M)

        return M_list

    def get_end_effector_frame(self) -> np.ndarray:
        """Get M_{n+1,n}: end-effector frame relative to last link CoM.

        Returns:
            (4, 4) transformation matrix from link n CoM to end-effector.
        """
        # End-effector frame is at tool0, which is at the flange
        # Offset from link 6 CoM to flange
        com6 = self.link_com_positions[5]
        d6 = self.dh_params[5, 1]  # 0.0823

        # The flange is at distance d6 along z from joint 6
        # CoM is offset from joint
        p_ee = np.array([0, 0, d6]) - com6

        return SE3.from_components(np.eye(3), p_ee)


def create_ur5e_parameters() -> UR5eParameters:
    """Factory function to create UR5e parameters with default values.

    Returns:
        UR5eParameters instance.
    """
    return UR5eParameters()
