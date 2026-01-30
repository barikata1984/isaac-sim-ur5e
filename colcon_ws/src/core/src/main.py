import sys
import os
from isaacsim.simulation_app import SimulationApp


def parse_headless_flag():
    """Parse headless flag from environment variable or command line args.

    Returns:
        bool: True if headless mode is requested, False otherwise.
    """
    # First check environment variable
    env_headless = os.environ.get('ISAAC_HEADLESS', '').lower()
    if env_headless in ('true', '1', 'yes'):
        return True

    # Then check command line arguments for ROS 2 parameter format
    for i, arg in enumerate(sys.argv):
        if arg == '-p' and i + 1 < len(sys.argv):
            param = sys.argv[i + 1]
            if param.startswith('headless:='):
                value = param.split(':=')[1].lower()
                return value in ('true', '1', 'yes')

    return False


# SimulationApp must be initialized before importing other omni modules
# and before rclpy if it's used in the same process.
headless = parse_headless_flag()
simulation_app = SimulationApp({"headless": headless})

import rclpy
from core.communication.node import IsaacSimNode

def main(args=None):
    rclpy.init(args=args)
    node = IsaacSimNode()

    try:
        node.setup_robot()
        node.get_logger().info("Starting simulation loop...")

        # Render only if not in headless mode
        render = not headless

        while simulation_app.is_running() and rclpy.ok():
            node.step(render=render)
            # Run ROS 2 callbacks
            rclpy.spin_once(node, timeout_sec=0)
            
    except Exception as e:
        if rclpy.ok():
            node.get_logger().error(f"Error in simulation loop: {e}")
        else:
            print(f"Error after rclpy shutdown: {e}")
    finally:
        if rclpy.ok():
            node.get_logger().info("Shutting down...")
            node.destroy_node()
            rclpy.shutdown()
        simulation_app.close()

if __name__ == '__main__':
    main()
