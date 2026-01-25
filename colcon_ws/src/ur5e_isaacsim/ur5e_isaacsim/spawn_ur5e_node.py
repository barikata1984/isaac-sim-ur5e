"""ROS2 Node for spawning UR5e in Isaac Sim GUI."""
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage


class UR5eSpawnerNode(Node):
    """Node that spawns UR5e in Isaac Sim and runs simulation loop."""

    def __init__(self):
        super().__init__('ur5e_spawner')
        self.status_pub = self.create_publisher(String, 'ur5e_status', 10)
        self._setup_simulation()
        self.get_logger().info('UR5e spawned successfully in Isaac Sim')
        
        # Publish initial status
        msg = String()
        msg.data = "UR5e spawned and simulation running"
        self.status_pub.publish(msg)

    def _setup_simulation(self):
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        assets_root_path = get_assets_root_path()
        if not assets_root_path:
            self.get_logger().error("Could not find Isaac Sim assets root path")
            return

        asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"

        add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR5e")
        self.ur5e_robot = Robot(prim_path="/World/UR5e", name="ur5e")
        self.world.scene.add(self.ur5e_robot)
        self.world.reset()

    def spin_simulation(self):
        self.get_logger().info("Starting simulation loop...")
        while simulation_app.is_running() and rclpy.ok():
            self.world.step(render=True)
            # Run ROS2 callbacks
            rclpy.spin_once(self, timeout_sec=0)
        
        self.get_logger().info("Simulation stopped.")
        simulation_app.close()


def main(args=None):
    rclpy.init(args=args)
    node = UR5eSpawnerNode()
    try:
        node.spin_simulation()
    except Exception as e:
        print(f"Error in simulation loop: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
