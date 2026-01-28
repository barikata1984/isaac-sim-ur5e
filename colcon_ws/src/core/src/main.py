import sys
from omni.isaac.kit import SimulationApp

# SimulationApp must be initialized before importing other omni modules
# and before rclpy if it's used in the same process.
simulation_app = SimulationApp({"headless": False})

import rclpy
from core.communication.node import IsaacSimNode

def main(args=None):
    rclpy.init(args=args)
    node = IsaacSimNode()
    
    try:
        node.setup_robot()
        node.get_logger().info("Starting simulation loop...")
        
        while simulation_app.is_running() and rclpy.ok():
            node.step(render=True)
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
