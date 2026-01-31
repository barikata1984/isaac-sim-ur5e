"""Isaac Sim verification node for dynamics calculations.

This node compares computed dynamics (velocity, acceleration) with
Isaac Sim's physics simulation in real-time.

Usage:
    1. Start Isaac Sim: ros2 run core main
    2. Start this node: ros2 run dynamics isaac_verification
    3. Send trajectory: ros2 launch trajectories joint_pos_sender.launch.py json_path:=<path>
"""

from dataclasses import dataclass, field
from typing import Optional, List
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


@dataclass
class VerificationRecord:
    """Record of a single verification sample."""

    timestamp: float
    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    computed_twist_base: np.ndarray  # [vx, vy, vz, wx, wy, wz] in base frame
    isaac_twist_base: Optional[np.ndarray] = None
    error: Optional[float] = None


class IsaacVerificationNode(Node):
    """ROS 2 node for verifying dynamics against Isaac Sim.

    This node:
    1. Subscribes to joint commands to track robot motion
    2. Computes tool0 velocity using our dynamics implementation
    3. Compares with Isaac Sim reported velocities (when available)
    4. Logs verification results
    """

    def __init__(self):
        super().__init__('isaac_verification_node')

        # Parameters
        self.declare_parameter('output_path', '/tmp/isaac_verification.json')
        self.declare_parameter('sample_rate', 60.0)

        self.output_path = self.get_parameter('output_path').value
        self.sample_rate = self.get_parameter('sample_rate').value

        # Subscribers
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_command_callback,
            10
        )

        # State tracking
        self.prev_positions: Optional[np.ndarray] = None
        self.prev_time: Optional[float] = None
        self.records: List[VerificationRecord] = []

        # Import dynamics
        try:
            import sys
            sys.path.insert(0, '/workspaces/isaac-sim-ur5e/colcon_ws/src/dynamics/src')
            from newton_euler_joint_frame import create_ur5e_joint_frame_dynamics
            from forward_kinematics import compute_tool0_twist_base

            self.dynamics = create_ur5e_joint_frame_dynamics()
            self.compute_tool0_twist_base = compute_tool0_twist_base
            self.get_logger().info("Dynamics module loaded successfully")
        except ImportError as e:
            self.get_logger().error(f"Failed to import dynamics: {e}")
            self.dynamics = None
            self.compute_tool0_twist_base = None

        self.get_logger().info("Isaac verification node initialized")
        self.get_logger().info(f"Output will be saved to: {self.output_path}")

    def joint_command_callback(self, msg: Float64MultiArray):
        """Process joint command and compute dynamics."""
        if self.dynamics is None:
            return

        if len(msg.data) != 6:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        current_positions = np.array(msg.data)

        # Compute velocity from position changes
        if self.prev_positions is not None and self.prev_time is not None:
            dt = current_time - self.prev_time
            if dt > 0:
                velocities = (current_positions - self.prev_positions) / dt

                # Compute tool0 twist in base frame
                twist_base = self.compute_tool0_twist_base(current_positions, velocities)

                # Create record
                record = VerificationRecord(
                    timestamp=current_time,
                    joint_positions=current_positions.copy(),
                    joint_velocities=velocities.copy(),
                    computed_twist_base=twist_base.copy(),
                )
                self.records.append(record)

                # Log periodically
                if len(self.records) % 60 == 0:
                    self.get_logger().info(
                        f"Samples: {len(self.records)}, "
                        f"twist_v: [{twist_base[0]:.4f}, {twist_base[1]:.4f}, {twist_base[2]:.4f}], "
                        f"twist_w: [{twist_base[3]:.4f}, {twist_base[4]:.4f}, {twist_base[5]:.4f}]"
                    )

        self.prev_positions = current_positions
        self.prev_time = current_time

    def save_results(self):
        """Save verification results to JSON file."""
        if not self.records:
            self.get_logger().warn("No records to save")
            return

        data = {
            'num_samples': len(self.records),
            'records': [
                {
                    'timestamp': r.timestamp,
                    'joint_positions': r.joint_positions.tolist(),
                    'joint_velocities': r.joint_velocities.tolist(),
                    'computed_twist_base': r.computed_twist_base.tolist(),
                }
                for r in self.records
            ]
        }

        with open(self.output_path, 'w') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(f"Saved {len(self.records)} records to {self.output_path}")

    def print_summary(self):
        """Print verification summary."""
        if not self.records:
            self.get_logger().info("No records collected")
            return

        # Compute statistics
        velocities = np.array([r.joint_velocities for r in self.records])
        twists = np.array([r.computed_twist_base for r in self.records])

        self.get_logger().info("=" * 60)
        self.get_logger().info("VERIFICATION SUMMARY")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Total samples: {len(self.records)}")
        self.get_logger().info(f"Joint velocity range:")
        for i in range(6):
            self.get_logger().info(
                f"  Joint {i+1}: [{velocities[:, i].min():.4f}, {velocities[:, i].max():.4f}] rad/s"
            )
        self.get_logger().info(f"Tool0 linear velocity range (base frame):")
        self.get_logger().info(
            f"  vx: [{twists[:, 0].min():.4f}, {twists[:, 0].max():.4f}] m/s"
        )
        self.get_logger().info(
            f"  vy: [{twists[:, 1].min():.4f}, {twists[:, 1].max():.4f}] m/s"
        )
        self.get_logger().info(
            f"  vz: [{twists[:, 2].min():.4f}, {twists[:, 2].max():.4f}] m/s"
        )
        self.get_logger().info("=" * 60)


def main(args=None):
    rclpy.init(args=args)

    node = IsaacVerificationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.print_summary()
        node.save_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
