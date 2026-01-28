import json
import time 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
import os

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        
        # Declare parameters
        self.declare_parameter('json_path', '')
        self.declare_parameter('loop', False)
        
        # Publisher
        self.publisher_ = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        
        # Load JSON
        json_path = self.get_parameter('json_path').get_parameter_value().string_value
        if not json_path:
            self.get_logger().error('No JSON path provided')
            return
            
        try:
            with open(json_path, 'r') as f:
                self.data = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load JSON: {e}")
            return
            
        self.frames = self.data.get('frames', [])
        self.fps = self.data.get('fps', 60.0)
        self.duration = self.data.get('duration', 0.0)
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value
        
        self.current_frame = 0
        self.timer_period = 1.0 / self.fps
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.active = False
        
        # Start a thread to wait for input
        import threading
        self.input_thread = threading.Thread(target=self.wait_for_start)
        self.input_thread.daemon = True
        self.input_thread.start()

    def wait_for_start(self):
        try:
            self.get_logger().info("Press Enter in the terminal to start the trajectory...")
            input()
            self.get_logger().info("Starting trajectory...")
            self.active = True
        except EOFError:
            self.get_logger().warn("Stdin closed, starting immediately after 3 seconds...")
            time.sleep(3)
            self.active = True

    def timer_callback(self):
        if not self.active:
            return

        if self.current_frame < len(self.frames):
            frame_data = self.frames[self.current_frame]
            
            # Extract qpos (joint positions) - it's the first element in the frame array
            # Structure matches: [qpos, qvel, qacc]
            if isinstance(frame_data, list) and len(frame_data) > 0:
                qpos = frame_data[0]
                
                msg = Float64MultiArray()
                msg.data = qpos
                self.publisher_.publish(msg)
                
                if self.current_frame % int(self.fps) == 0:
                     self.get_logger().info(f"Publishing frame {self.current_frame}/{len(self.frames)}")
                
                self.current_frame += 1
            else:
                self.get_logger().warn(f"Invalid frame format at index {self.current_frame}")
                self.current_frame += 1
        elif self.loop:
            self.current_frame = 0
            self.get_logger().info("Looping trajectory")
        else:
            self.get_logger().info("Trajectory finished")
            self.timer.cancel()
            self.active = False

def main(args=None):
    rclpy.init(args=args)
    
    node = TrajectoryFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
