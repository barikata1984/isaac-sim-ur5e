import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from omni.isaac.core import World
from omni.isaac.core.utils.types import ArticulationAction
from isaacsim_core.spawning.ur5e import spawn_ur5e
import numpy as np

class IsaacSimNode(Node):
    """ROS 2 Node that manages Isaac Sim world and communication."""

    def __init__(self, name='isaac_sim_core'):
        super().__init__(name)
        self.status_pub = self.create_publisher(String, 'sim_status', 10)
        self.joint_sub = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_command_callback,
            10
        )
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        self.robot = None
        self._target_joint_positions = None
        
    def setup_robot(self):
        """Initializes the robot in the simulation."""
        self.robot = spawn_ur5e(self.world)
        self.world.reset()
        self.get_logger().info('Robot spawned and world reset')
        self.publish_status("Robot spawned and ready")

    def joint_command_callback(self, msg):
        """Callback for joint commands in radians."""
        if self.robot is None:
            self.get_logger().warn("Robot not initialized yet")
            return
            
        if len(msg.data) != 6:
            self.get_logger().error(f"Expected 6 joint angles, got {len(msg.data)}")
            return
            
        self.get_logger().info(f"Received joint commands: {msg.data}")
        # Store target positions for the step loop
        self._target_joint_positions = np.array(msg.data)
        self.publish_status(f"Target set to: {msg.data}")

    def publish_status(self, message):
        """Publishes a status message to ROS 2."""
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)

    def step(self, render=True):
        """Advanced the simulation by one step."""
        if self.robot and self._target_joint_positions is not None:
            # Apply physics-based control to reach target positions
            self.robot.apply_action(ArticulationAction(joint_positions=self._target_joint_positions))
        
        self.world.step(render=render)
