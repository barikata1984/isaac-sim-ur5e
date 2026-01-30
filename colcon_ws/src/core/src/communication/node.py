"""Isaac Sim ROS 2 node with parameterized robot spawning."""

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
import numpy as np


class IsaacSimNode(Node):
    """Isaac Sim ROS 2 node with parameterized robot spawning."""

    def __init__(self, name: str = 'isaac_sim_node'):
        """Initialize Isaac Sim node.

        Args:
            name: Name of the ROS 2 node.
        """
        super().__init__(name)

        # Declare parameters
        self.declare_parameter('robot_type', 'ur5e')
        self.declare_parameter('prim_path', '/World/UR')
        self.declare_parameter('robot_name', 'ur_robot')
        self.declare_parameter('headless', False)

        # Setup ROS 2 communication
        self.status_pub = self.create_publisher(String, 'sim_status', 10)
        self.joint_sub = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_command_callback,
            10
        )

        # World setup
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        # Robot reference
        self.robot: Optional[object] = None
        self._target_joint_positions: Optional[np.ndarray] = None

        # Log parameters
        robot_type = self.get_parameter('robot_type').value
        self.get_logger().info(f'Robot type: {robot_type}')

    def setup_robot(self):
        """Setup robot in the simulation using ur package."""
        robot_type = self.get_parameter('robot_type').value
        prim_path = self.get_parameter('prim_path').value
        robot_name = self.get_parameter('robot_name').value

        try:
            # Import ur package spawn function
            from ur.spawning import spawn_ur_robot

            self.get_logger().info(
                f'Spawning {robot_type} at {prim_path} '
                f'with name {robot_name}'
            )

            self.robot = spawn_ur_robot(
                self.world,
                robot_type=robot_type,
                prim_path=prim_path,
                name=robot_name
            )

            self.world.reset()
            self.get_logger().info(f'Robot {robot_type} spawned successfully')
            self.publish_status(f"Robot {robot_type} spawned and ready")

        except ImportError as e:
            self.get_logger().error(
                f'Failed to import ur package: {e}. '
                'Make sure robots/ur package is built and sourced.'
            )
            raise
        except Exception as e:
            self.get_logger().error(f'Failed to spawn robot: {e}')
            raise

    def joint_command_callback(self, msg: Float64MultiArray):
        """Callback for joint commands in radians.

        Args:
            msg: Float64MultiArray containing 6 joint angles.
        """
        if self.robot is None:
            self.get_logger().warn("Robot not initialized yet")
            return

        if len(msg.data) != 6:
            self.get_logger().error(
                f"Expected 6 joint angles, got {len(msg.data)}"
            )
            return

        self.get_logger().info(f"Received joint commands: {msg.data}")
        # Store target positions for the step loop
        self._target_joint_positions = np.array(msg.data)
        self.publish_status(f"Target set to: {msg.data}")

    def publish_status(self, message: str):
        """Publish a status message to ROS 2.

        Args:
            message: Status message to publish.
        """
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)

    def step(self, render: bool = True):
        """Advance the simulation by one step.

        Args:
            render: Whether to render the simulation.
        """
        if self.robot and self._target_joint_positions is not None:
            # Apply physics-based control to reach target positions
            self.robot.apply_action(
                ArticulationAction(
                    joint_positions=self._target_joint_positions
                )
            )

        self.world.step(render=render)

