#!/usr/bin/env python3
"""
ROS2-based manipulator control for Isaac Sim.
Subscribes to /target_position topic and moves manipulator using IK.
"""
import argparse
import sys

# Parse arguments BEFORE SimulationApp initialization
def parse_args():
    parser = argparse.ArgumentParser(description="ROS2-based manipulator control")
    parser.add_argument(
        "--manipulator", "-m",
        type=str,
        default="ur5e",
        help="Manipulator to use"
    )
    parser.add_argument(
        "--headless",
        type=str,
        default="false",
        help="Run in headless mode (true/false)"
    )
    args, _ = parser.parse_known_args()
    return args

args = parse_args()
headless = args.headless.lower() == "true"

from omni.isaac.kit import SimulationApp

# Launch the simulator with explicit window configuration
config = {
    "headless": headless,
    "open_usd": None,
    "width": 1280,
    "height": 720,
    "window_width": 1920,
    "window_height": 1080,
    "renderer": "RayTracedLighting",
    "anti_aliasing": 3,  # FXAA
    "active_gpu": 0,
    "physics_gpu": 0,
}
simulation_app = SimulationApp(config)

import numpy as np

sys.path.insert(0, "/workspaces/isaac-sim-ur5e")
from src.env_loader import create_manipulator_env, get_available_manipulators
from omni.isaac.motion_generation import LulaKinematicsSolver
from omni.isaac.motion_generation import interface_config_loader
from omni.isaac.core.utils.types import ArticulationAction

# ============================================
# ROS2 Bridge Setup
# ============================================
# This script must be run via run_isaac_ros2.sh to avoid conflicts
# between container's ROS2 and Isaac Sim's ROS2 bridge.

# Enable ROS2 bridge extension
import omni.ext
manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class ManipulatorController(Node):
    """ROS2 node that controls a manipulator in Isaac Sim."""
    
    def __init__(self, robot, ik_solver, end_effector_frame):
        super().__init__('manipulator_controller')
        self.robot = robot
        self.ik_solver = ik_solver
        self.end_effector_frame = end_effector_frame
        self.target_position = None
        self.move_requested = False
        
        # Subscribe to target position
        self.subscription = self.create_subscription(
            Point,
            '/target_position',
            self.target_callback,
            10
        )
        self.get_logger().info('Subscribed to /target_position')
        self.get_logger().info(f'Controlling frame: {end_effector_frame}')
    
    def target_callback(self, msg):
        """Callback when new target position is received."""
        self.target_position = np.array([msg.x, msg.y, msg.z])
        self.move_requested = True
        self.get_logger().info(f'Received target: {self.target_position}')
    
    def process_move_request(self):
        """Process pending move request. Returns True if movement was initiated."""
        if not self.move_requested or self.target_position is None:
            return False
        
        self.move_requested = False
        target_pos = self.target_position
        
        # Get current orientation
        current_pose = self.robot.get_world_pose()
        target_orientation = current_pose[1]
        
        # Compute IK
        joint_positions, ik_converged = self.ik_solver.compute_inverse_kinematics(
            frame_name=self.end_effector_frame,
            target_position=target_pos,
            target_orientation=target_orientation
        )
        
        if ik_converged:
            self.get_logger().info('Target REACHABLE. Moving...')
            action = ArticulationAction(joint_positions=joint_positions)
            self.robot.get_articulation_controller().apply_action(action)
            return True
        else:
            self.get_logger().warn('Target OUT OF REACH!')
            return False


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="ROS2-based manipulator control")
    parser.add_argument(
        "--manipulator", "-m",
        type=str,
        default="ur5e",
        help=f"Manipulator to use. Available: {get_available_manipulators()}"
    )
    args, unknown = parser.parse_known_args()
    
    # Create Isaac Sim environment
    world, robot, end_effector_frame, ik_name = create_manipulator_env(simulation_app, args.manipulator)
    if world is None:
        simulation_app.close()
        return

    # Initialize IK Solver
    try:
        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(ik_name)
        ik_solver = LulaKinematicsSolver(**kinematics_config)
        print(f"IK solver loaded for: {ik_name}")
    except Exception as e:
        print(f"Error: IK solver not available for {args.manipulator}: {e}")
        simulation_app.close()
        return

    world.reset()
    
    # Warm up simulation
    for _ in range(20):
        world.step(render=True)

    # Initialize ROS2
    rclpy.init()
    controller = ManipulatorController(robot, ik_solver, end_effector_frame)
    
    print("\n" + "="*50)
    print(f"  {args.manipulator.upper()} ROS2 Control Ready")
    print("="*50)
    print("Listening for target positions on /target_position")
    print("Send from another terminal:")
    print("  ros2 topic pub /target_position geometry_msgs/Point \"{x: 0.4, y: 0.4, z: 0.5}\"")
    print("="*50 + "\n")

    move_steps_remaining = 0

    try:
        while simulation_app.is_running():
            # Spin ROS2 once (non-blocking)
            rclpy.spin_once(controller, timeout_sec=0)
            
            # Step simulation
            world.step(render=True)
            
            # Handle movement animation
            if move_steps_remaining > 0:
                move_steps_remaining -= 1
                continue
            
            # Process any pending move requests
            if controller.process_move_request():
                move_steps_remaining = 60  # ~1 second of movement
                
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
