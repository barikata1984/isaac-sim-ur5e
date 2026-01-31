"""Isaac Sim interface for dynamics verification.

Provides functions to query robot state (positions, velocities, accelerations)
from Isaac Sim for comparison with computed dynamics.
"""

from dataclasses import dataclass
from typing import Optional, Tuple, List
import numpy as np


@dataclass
class RobotState:
    """Complete robot state from Isaac Sim.

    Attributes:
        joint_positions: Joint positions [rad] (6,).
        joint_velocities: Joint velocities [rad/s] (6,).
        tool0_position: tool0 position in world frame [m] (3,).
        tool0_orientation: tool0 orientation quaternion [w, x, y, z] (4,).
        tool0_linear_velocity: tool0 linear velocity in world frame [m/s] (3,).
        tool0_angular_velocity: tool0 angular velocity in world frame [rad/s] (3,).
        timestamp: Simulation timestamp [s].
    """

    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    tool0_position: np.ndarray
    tool0_orientation: np.ndarray
    tool0_linear_velocity: np.ndarray
    tool0_angular_velocity: np.ndarray
    timestamp: float


class IsaacSimRobotInterface:
    """Interface to query robot state from Isaac Sim.

    This class wraps the Isaac Sim Robot API to provide convenient access
    to robot state information for dynamics verification.
    """

    def __init__(self, robot, world):
        """Initialize the interface.

        Args:
            robot: Isaac Sim Robot (Articulation) object.
            world: Isaac Sim World object.
        """
        self.robot = robot
        self.world = world
        self._tool0_link_index = self._find_tool0_link_index()

    def _find_tool0_link_index(self) -> int:
        """Find the index of the tool0 link.

        Returns:
            Index of tool0 link in the articulation.
        """
        # UR5e link names typically include tool0
        # This may need adjustment based on actual USD structure
        try:
            link_names = self.robot.get_link_names()
            for i, name in enumerate(link_names):
                if 'tool0' in name.lower():
                    return i
            # If tool0 not found, use the last link (wrist_3_link or similar)
            return len(link_names) - 1
        except Exception:
            return -1  # Will handle in get_state

    def get_state(self) -> RobotState:
        """Get current robot state from Isaac Sim.

        Returns:
            RobotState containing all relevant state information.
        """
        # Joint state
        joint_positions = self.robot.get_joint_positions()
        joint_velocities = self.robot.get_joint_velocities()

        # tool0 pose and velocity
        # Note: These are in world (base) frame
        try:
            tool0_position, tool0_orientation = self.robot.get_world_pose()
            tool0_linear_velocity = self.robot.get_linear_velocity()
            tool0_angular_velocity = self.robot.get_angular_velocity()
        except Exception:
            # Fallback if link-specific queries fail
            tool0_position = np.zeros(3)
            tool0_orientation = np.array([1.0, 0.0, 0.0, 0.0])
            tool0_linear_velocity = np.zeros(3)
            tool0_angular_velocity = np.zeros(3)

        # Get simulation time
        timestamp = self.world.current_time

        return RobotState(
            joint_positions=np.array(joint_positions),
            joint_velocities=np.array(joint_velocities),
            tool0_position=np.array(tool0_position),
            tool0_orientation=np.array(tool0_orientation),
            tool0_linear_velocity=np.array(tool0_linear_velocity),
            tool0_angular_velocity=np.array(tool0_angular_velocity),
            timestamp=timestamp,
        )

    def get_link_velocity(self, link_name: str) -> Tuple[np.ndarray, np.ndarray]:
        """Get velocity of a specific link.

        Args:
            link_name: Name of the link.

        Returns:
            Tuple of (linear_velocity, angular_velocity) in world frame.
        """
        # This requires accessing the specific rigid body prim
        # Implementation depends on Isaac Sim version
        raise NotImplementedError("Link-specific velocity query not yet implemented")


def compute_numerical_acceleration(
    velocities: List[np.ndarray],
    timestamps: List[float],
) -> np.ndarray:
    """Compute acceleration from velocity history using finite differences.

    Args:
        velocities: List of velocity arrays (at least 2).
        timestamps: List of corresponding timestamps.

    Returns:
        Estimated acceleration at the last timestamp.
    """
    if len(velocities) < 2:
        return np.zeros_like(velocities[-1])

    dt = timestamps[-1] - timestamps[-2]
    if dt < 1e-10:
        return np.zeros_like(velocities[-1])

    return (velocities[-1] - velocities[-2]) / dt


@dataclass
class DynamicsVerificationResult:
    """Result of comparing computed dynamics with Isaac Sim.

    Attributes:
        timestamp: Simulation timestamp.
        joint_positions: Joint positions used.
        joint_velocities: Joint velocities used.
        computed_angular_velocity: Angular velocity from dynamics calculation.
        computed_linear_velocity: Linear velocity from dynamics calculation.
        isaac_angular_velocity: Angular velocity from Isaac Sim.
        isaac_linear_velocity: Linear velocity from Isaac Sim.
        angular_velocity_error: Error in angular velocity.
        linear_velocity_error: Error in linear velocity.
    """

    timestamp: float
    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    computed_angular_velocity: np.ndarray
    computed_linear_velocity: np.ndarray
    isaac_angular_velocity: np.ndarray
    isaac_linear_velocity: np.ndarray
    angular_velocity_error: np.ndarray
    linear_velocity_error: np.ndarray

    @property
    def angular_velocity_norm_error(self) -> float:
        """Norm of angular velocity error."""
        return np.linalg.norm(self.angular_velocity_error)

    @property
    def linear_velocity_norm_error(self) -> float:
        """Norm of linear velocity error."""
        return np.linalg.norm(self.linear_velocity_error)
