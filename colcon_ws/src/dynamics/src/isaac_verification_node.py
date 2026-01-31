"""Isaac Sim verification node for dynamics calculations.

This node compares computed dynamics (velocity, acceleration) with
Isaac Sim's physics simulation in real-time.

Coordinate Frame Convention:
    - Our DH implementation computes twist in UR internal frame (X+ backward)
    - Isaac Sim reports twist in world frame (REP-103: X+ forward)
    - Transformation: π rotation about Z axis
        vx_world = -vx_internal, vy_world = -vy_internal, vz_world = vz_internal
        ωx_world = -ωx_internal, ωy_world = -ωy_internal, ωz_world = ωz_internal

Usage:
    1. Start Isaac Sim: ros2 launch core bring_up.launch.py headless:=true
    2. Start this node: ros2 run dynamics isaac_verification
    3. Send trajectory: ros2 run trajectories trajectory_follower --ros-args -p json_path:=<path> -p auto_start:=true
"""

from dataclasses import dataclass
from typing import Optional, List
import json

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool


def transform_ur_internal_to_world(twist: np.ndarray) -> np.ndarray:
    """Transform twist from UR internal frame to REP-103 world frame.

    UR internal frame has X+ pointing backward, while REP-103 has X+ forward.
    The transformation is a π rotation about the Z axis.

    Args:
        twist: [vx, vy, vz, ωx, ωy, ωz] in UR internal frame.

    Returns:
        Twist in world frame (REP-103 convention).
    """
    transformed = twist.copy()
    # Apply π rotation about Z: negate x and y components
    transformed[0] = -twist[0]  # vx
    transformed[1] = -twist[1]  # vy
    # vz unchanged
    transformed[3] = -twist[3]  # ωx
    transformed[4] = -twist[4]  # ωy
    # ωz unchanged
    return transformed


@dataclass
class VerificationRecord:
    """Record of a single verification sample."""

    timestamp: float
    joint_positions: np.ndarray  # Actual positions from Isaac Sim
    joint_velocities: np.ndarray  # Actual velocities from Isaac Sim
    computed_twist: np.ndarray  # [vx, vy, vz, wx, wy, wz] computed from FK (world frame)
    isaac_twist: np.ndarray  # [vx, vy, vz, wx, wy, wz] from Isaac Sim (world frame)


class IsaacVerificationNode(Node):
    """ROS 2 node for verifying dynamics against Isaac Sim.

    This node:
    1. Subscribes to actual joint state from Isaac Sim
    2. Computes tool0 velocity using our forward kinematics
    3. Compares with Isaac Sim's reported tool0 twist
    4. When trajectory completes, runs verification and saves results
    """

    def __init__(self):
        super().__init__('isaac_verification_node')

        # Parameters
        self.declare_parameter('output_path', '/tmp/isaac_verification.json')

        self.output_path = self.get_parameter('output_path').value

        # Subscriber for combined synchronized data
        # [q1..q6, dq1..dq6, vx, vy, vz, wx, wy, wz] = 18 values
        self.combined_sub = self.create_subscription(
            Float64MultiArray,
            'robot_state_combined',
            self.combined_state_callback,
            10
        )

        self.complete_sub = self.create_subscription(
            Bool,
            'trajectory_complete',
            self.trajectory_complete_callback,
            10
        )

        # State tracking
        self.records: List[VerificationRecord] = []
        self.collecting = True

        # Import dynamics
        try:
            from dynamics.forward_kinematics import compute_tool0_twist_base

            self.compute_tool0_twist_base = compute_tool0_twist_base
            self.get_logger().info("Dynamics module loaded successfully")
        except ImportError as e:
            self.get_logger().error(f"Failed to import dynamics: {e}")
            self.compute_tool0_twist_base = None

        self.get_logger().info("Isaac verification node initialized")
        self.get_logger().info(f"Output will be saved to: {self.output_path}")
        self.get_logger().info("Waiting for robot_state_combined data from Isaac Sim...")

    def combined_state_callback(self, msg: Float64MultiArray):
        """Process synchronized robot state from Isaac Sim.

        Message format: [q1..q6, dq1..dq6, vx, vy, vz, wx, wy, wz] = 18 values
        All data is from the same simulation step, ensuring synchronization.
        """
        if not self.collecting or self.compute_tool0_twist_base is None:
            return

        # Expect 18 values: 6 positions + 6 velocities + 6 twist
        if len(msg.data) != 18:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        # Extract synchronized data
        joint_positions = np.array(msg.data[:6])
        joint_velocities = np.array(msg.data[6:12])
        isaac_twist = np.array(msg.data[12:18])

        # Compute tool0 twist using our forward kinematics (UR internal frame)
        computed_twist_internal = self.compute_tool0_twist_base(joint_positions, joint_velocities)

        # Transform to world frame (REP-103) for comparison with Isaac Sim
        computed_twist = transform_ur_internal_to_world(computed_twist_internal)

        # Create record with synchronized data
        record = VerificationRecord(
            timestamp=current_time,
            joint_positions=joint_positions.copy(),
            joint_velocities=joint_velocities.copy(),
            computed_twist=computed_twist.copy(),
            isaac_twist=isaac_twist.copy(),
        )
        self.records.append(record)

        # Log periodically
        if len(self.records) % 60 == 0:
            self.get_logger().info(
                f"Samples: {len(self.records)}, "
                f"computed_v: [{computed_twist[0]:.4f}, {computed_twist[1]:.4f}, {computed_twist[2]:.4f}]"
            )

    def trajectory_complete_callback(self, msg: Bool):
        """Handle trajectory completion."""
        if msg.data and self.collecting:
            self.get_logger().info("Trajectory complete signal received")
            self.collecting = False
            self.run_verification()

    def run_verification(self):
        """Run verification and save results."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("RUNNING VERIFICATION")
        self.get_logger().info("=" * 60)

        if not self.records:
            self.get_logger().warn("No records collected")
            return

        self.get_logger().info(f"Total samples: {len(self.records)}")

        # Compute statistics
        computed_twists = np.array([r.computed_twist for r in self.records])
        isaac_twists = np.array([r.isaac_twist for r in self.records])

        self.get_logger().info("Computed tool0 twist range (world frame, after transformation):")
        self.get_logger().info(
            f"  vx: [{computed_twists[:, 0].min():.4f}, {computed_twists[:, 0].max():.4f}] m/s"
        )
        self.get_logger().info(
            f"  vy: [{computed_twists[:, 1].min():.4f}, {computed_twists[:, 1].max():.4f}] m/s"
        )
        self.get_logger().info(
            f"  vz: [{computed_twists[:, 2].min():.4f}, {computed_twists[:, 2].max():.4f}] m/s"
        )
        self.get_logger().info(
            f"  wx: [{computed_twists[:, 3].min():.4f}, {computed_twists[:, 3].max():.4f}] rad/s"
        )
        self.get_logger().info(
            f"  wy: [{computed_twists[:, 4].min():.4f}, {computed_twists[:, 4].max():.4f}] rad/s"
        )
        self.get_logger().info(
            f"  wz: [{computed_twists[:, 5].min():.4f}, {computed_twists[:, 5].max():.4f}] rad/s"
        )

        self.get_logger().info("")
        self.get_logger().info("Isaac Sim tool0 twist range (world frame):")
        self.get_logger().info(
            f"  vx: [{isaac_twists[:, 0].min():.4f}, {isaac_twists[:, 0].max():.4f}] m/s"
        )
        self.get_logger().info(
            f"  vy: [{isaac_twists[:, 1].min():.4f}, {isaac_twists[:, 1].max():.4f}] m/s"
        )
        self.get_logger().info(
            f"  vz: [{isaac_twists[:, 2].min():.4f}, {isaac_twists[:, 2].max():.4f}] m/s"
        )
        self.get_logger().info(
            f"  wx: [{isaac_twists[:, 3].min():.4f}, {isaac_twists[:, 3].max():.4f}] rad/s"
        )
        self.get_logger().info(
            f"  wy: [{isaac_twists[:, 4].min():.4f}, {isaac_twists[:, 4].max():.4f}] rad/s"
        )
        self.get_logger().info(
            f"  wz: [{isaac_twists[:, 5].min():.4f}, {isaac_twists[:, 5].max():.4f}] rad/s"
        )

        # Compute comparison
        errors = isaac_twists - computed_twists
        rmse = np.sqrt(np.mean(errors ** 2, axis=0))
        max_error = np.max(np.abs(errors), axis=0)

        self.get_logger().info("")
        self.get_logger().info("COMPARISON (Isaac Sim - Computed):")
        self.get_logger().info(f"  RMSE (linear):  [{rmse[0]:.6f}, {rmse[1]:.6f}, {rmse[2]:.6f}] m/s")
        self.get_logger().info(f"  RMSE (angular): [{rmse[3]:.6f}, {rmse[4]:.6f}, {rmse[5]:.6f}] rad/s")
        self.get_logger().info(f"  Max error (linear):  [{max_error[0]:.6f}, {max_error[1]:.6f}, {max_error[2]:.6f}] m/s")
        self.get_logger().info(f"  Max error (angular): [{max_error[3]:.6f}, {max_error[4]:.6f}, {max_error[5]:.6f}] rad/s")

        # Verification pass/fail
        linear_rmse = np.linalg.norm(rmse[:3])
        angular_rmse = np.linalg.norm(rmse[3:])
        self.get_logger().info("")
        if linear_rmse < 0.01 and angular_rmse < 0.01:
            self.get_logger().info("VERIFICATION PASSED: Errors within tolerance (0.01)")
        else:
            self.get_logger().info(f"VERIFICATION RESULT: Linear RMSE={linear_rmse:.6f}, Angular RMSE={angular_rmse:.6f}")

        self.get_logger().info("=" * 60)

        # Save results
        self.save_results()

    def save_results(self):
        """Save verification results to JSON file."""
        if not self.records:
            return

        data = {
            'num_samples': len(self.records),
            'records': [
                {
                    'timestamp': r.timestamp,
                    'joint_positions': r.joint_positions.tolist(),
                    'joint_velocities': r.joint_velocities.tolist(),
                    'computed_twist': r.computed_twist.tolist(),
                    'isaac_twist': r.isaac_twist.tolist(),
                }
                for r in self.records
            ]
        }

        with open(self.output_path, 'w') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(f"Saved {len(self.records)} records to {self.output_path}")


def main(args=None):
    rclpy.init(args=args)

    node = IsaacVerificationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
        if node.collecting and node.records:
            node.run_verification()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
