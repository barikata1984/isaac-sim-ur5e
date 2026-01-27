import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import math

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        
    def publish_joints(self, degrees):
        if len(degrees) != 6:
            self.get_logger().error(f"Expected 6 joint angles, got {len(degrees)}")
            return
            
        # Convert degrees to radians
        radians = [d * math.pi / 180.0 for d in degrees]
        
        msg = Float64MultiArray()
        msg.data = radians
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published joint commands (deg): {degrees}")

def main(args=None):
    rclpy.init(args=args)
    
    # Simple CLI argument parsing
    try:
        if len(sys.argv) < 7:
            print("Usage: ros2 run isaacsim_teleop joint_publisher deg1 deg2 deg3 deg4 deg5 deg6")
            return
            
        joint_angles = [float(sys.argv[i]) for i in range(1, 7)]
    except ValueError:
        print("Error: All arguments must be numbers")
        return

    node = JointPublisher()
    
    # We need to wait a tiny bit for discovery, or just publish a few times
    # For a simple script, we'll publish once and exit.
    # In a real scenario, we might want to wait for subscription.
    
    import time
    time.sleep(1.0) # Wait for discovery
    node.publish_joints(joint_angles)
    time.sleep(0.5) # Ensure it's sent
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
