import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from omni.isaac.core import World
from isaacsim_core.spawning.ur5e import spawn_ur5e

class IsaacSimNode(Node):
    """ROS 2 Node that manages Isaac Sim world and communication."""

    def __init__(self, name='isaac_sim_core'):
        super().__init__(name)
        self.status_pub = self.create_publisher(String, 'sim_status', 10)
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        self.robot = None
        
    def setup_robot(self):
        """Initializes the robot in the simulation."""
        self.robot = spawn_ur5e(self.world)
        self.world.reset()
        self.get_logger().info('Robot spawned and world reset')
        self.publish_status("Robot spawned and ready")

    def publish_status(self, message):
        """Publishes a status message to ROS 2."""
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)

    def step(self, render=True):
        """Advanced the simulation by one step."""
        self.world.step(render=render)
