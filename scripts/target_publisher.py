#!/usr/bin/env python3
"""
Simple target position publisher for manipulator control.
Run this in a separate terminal to send target positions.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.publisher = self.create_publisher(Point, '/target_position', 10)
        self.get_logger().info('Target position publisher ready')
    
    def publish_target(self, x, y, z):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published target: ({x}, {y}, {z})')


def main():
    rclpy.init()
    node = TargetPublisher()
    
    print("\n" + "="*50)
    print("  Target Position Publisher")
    print("="*50)
    print("Enter target position as: x y z")
    print("Example: 0.4 0.4 0.5")
    print("Type 'q' to quit")
    print("="*50 + "\n")
    
    try:
        while rclpy.ok():
            user_input = input(">> ")
            
            if user_input.lower() == 'q':
                break
            
            try:
                parts = user_input.split()
                if len(parts) != 3:
                    print("Invalid format. Enter 3 numbers: x y z")
                    continue
                
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                node.publish_target(x, y, z)
                
            except ValueError:
                print("Invalid numbers. Try again.")
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
