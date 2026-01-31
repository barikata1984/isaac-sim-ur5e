"""ROS 2 node for verifying dynamics calculations against Isaac Sim.

This node subscribes to robot state from Isaac Sim and compares
the computed tool0 velocity/acceleration with direct measurements.
"""

import json
from pathlib import Path
from typing import List, Optional
from dataclasses import dataclass, field, asdict

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from dynamics.newton_euler import NewtonEulerDynamics, create_ur5e_dynamics
from dynamics.lie_algebra import rotation_from_transform, adjoint


@dataclass
class VerificationRecord:
    """Single record for verification."""

    timestamp: float
    q: List[float]
    dq: List[float]
    ddq: List[float]
    computed_twist: List[float]  # [omega, v] in tool0 frame
    isaac_linear_vel_base: List[float]  # From Isaac Sim (base frame)
    isaac_angular_vel_base: List[float]  # From Isaac Sim (base frame)


@dataclass
class VerificationSession:
    """Collection of verification records."""

    records: List[VerificationRecord] = field(default_factory=list)

    def to_json(self, filepath: str) -> None:
        """Save session to JSON file."""
        data = {
            "records": [asdict(r) for r in self.records]
        }
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

    @classmethod
    def from_json(cls, filepath: str) -> "VerificationSession":
        """Load session from JSON file."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        records = [VerificationRecord(**r) for r in data["records"]]
        return cls(records=records)


class DynamicsVerificationNode(Node):
    """ROS 2 node for dynamics verification.

    This node:
    1. Receives robot state (joint positions, velocities)
    2. Computes tool0 twist using Newton-Euler dynamics
    3. Receives tool0 velocity from Isaac Sim (via custom topic)
    4. Logs comparison for later analysis
    """

    def __init__(self):
        super().__init__('dynamics_verification_node')

        # Parameters
        self.declare_parameter('output_file', 'verification_results.json')
        self.declare_parameter('record_interval', 0.01)  # seconds

        # Dynamics calculator
        self.dynamics = create_ur5e_dynamics()

        # State storage
        self._q: Optional[np.ndarray] = None
        self._dq: Optional[np.ndarray] = None
        self._dq_prev: Optional[np.ndarray] = None
        self._timestamp_prev: float = 0.0
        self._isaac_linear_vel: Optional[np.ndarray] = None
        self._isaac_angular_vel: Optional[np.ndarray] = None

        # Verification session
        self.session = VerificationSession()

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            Float64MultiArray,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.isaac_velocity_sub = self.create_subscription(
            Float64MultiArray,
            'tool0_velocity',  # [vx, vy, vz, wx, wy, wz] in base frame
            self.isaac_velocity_callback,
            10
        )

        # Timer for periodic verification
        interval = self.get_parameter('record_interval').value
        self.timer = self.create_timer(interval, self.verification_callback)

        self.get_logger().info('Dynamics verification node started')

    def joint_state_callback(self, msg: Float64MultiArray):
        """Receive joint state [q0..q5, dq0..dq5]."""
        if len(msg.data) >= 12:
            self._q = np.array(msg.data[:6])
            self._dq = np.array(msg.data[6:12])
        elif len(msg.data) == 6:
            # Only positions, estimate velocity numerically
            self._q = np.array(msg.data)

    def isaac_velocity_callback(self, msg: Float64MultiArray):
        """Receive tool0 velocity from Isaac Sim [vx,vy,vz,wx,wy,wz]."""
        if len(msg.data) >= 6:
            self._isaac_linear_vel = np.array(msg.data[:3])
            self._isaac_angular_vel = np.array(msg.data[3:6])

    def verification_callback(self):
        """Periodic verification comparison."""
        if self._q is None or self._dq is None:
            return

        timestamp = self.get_clock().now().nanoseconds / 1e9

        # Estimate acceleration from velocity change
        if self._dq_prev is not None:
            dt = timestamp - self._timestamp_prev
            if dt > 1e-6:
                ddq = (self._dq - self._dq_prev) / dt
            else:
                ddq = np.zeros(6)
        else:
            ddq = np.zeros(6)

        # Compute tool0 twist using dynamics
        computed_twist = self.dynamics.get_end_effector_twist(self._q, self._dq)

        # Create record
        record = VerificationRecord(
            timestamp=timestamp,
            q=self._q.tolist(),
            dq=self._dq.tolist(),
            ddq=ddq.tolist(),
            computed_twist=computed_twist.tolist(),
            isaac_linear_vel_base=(
                self._isaac_linear_vel.tolist()
                if self._isaac_linear_vel is not None
                else [0.0] * 3
            ),
            isaac_angular_vel_base=(
                self._isaac_angular_vel.tolist()
                if self._isaac_angular_vel is not None
                else [0.0] * 3
            ),
        )
        self.session.records.append(record)

        # Update previous state
        self._dq_prev = self._dq.copy()
        self._timestamp_prev = timestamp

    def save_results(self):
        """Save verification results to file."""
        output_file = self.get_parameter('output_file').value
        self.session.to_json(output_file)
        self.get_logger().info(f'Saved {len(self.session.records)} records to {output_file}')


def main(args=None):
    rclpy.init(args=args)
    node = DynamicsVerificationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
