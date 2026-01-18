from omni.isaac.kit import SimulationApp

# Launch the simulator in the script
simulation_app = SimulationApp({"headless": False})

import sys
import numpy as np
import threading
import queue
import argparse

sys.path.insert(0, "/workspaces/isaac-sim-ur5e")
from src.env_loader import create_manipulator_env, get_available_manipulators
from omni.isaac.motion_generation import LulaKinematicsSolver
from omni.isaac.motion_generation import interface_config_loader
from omni.isaac.core.utils.types import ArticulationAction


class InputThread(threading.Thread):
    """Thread to handle user input without blocking the main loop."""
    def __init__(self, input_queue):
        super().__init__(daemon=True)
        self.input_queue = input_queue
        self.running = True

    def run(self):
        while self.running:
            try:
                print("\nEnter target position (x y z) in meters (e.g., 0.4 0.4 0.5):")
                print("Type 'q' to quit.")
                user_input = input(">> ")
                self.input_queue.put(user_input)
            except EOFError:
                break

    def stop(self):
        self.running = False


def parse_input(input_str):
    """Parse space-separated x y z input."""
    try:
        parts = [float(x.strip()) for x in input_str.split()]
        if len(parts) != 3:
            return None
        return np.array(parts)
    except ValueError:
        return None


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Simple manipulator control with IK")
    parser.add_argument(
        "--manipulator", "-m",
        type=str,
        default="ur5e",
        help=f"Manipulator to use. Available: {get_available_manipulators()}"
    )
    args = parser.parse_args()
    
    world, robot, end_effector_frame, ik_name = create_manipulator_env(simulation_app, args.manipulator)
    if world is None:
        simulation_app.close()
        return

    # Initialize IK Solver using the correct IK name from registry
    try:
        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(ik_name)
        ik_solver = LulaKinematicsSolver(**kinematics_config)
        ik_enabled = True
        print(f"IK solver loaded for: {ik_name}")
    except Exception as e:
        print(f"Warning: IK solver not available for {args.manipulator}: {e}")
        print("Running in visualization-only mode (no IK control).")
        ik_solver = None
        ik_enabled = False

    world.reset()
    
    # Warm up simulation
    for _ in range(20):
        world.step(render=True)

    print(f"\n--- {args.manipulator.upper()} Control Initialized ---")
    print(f"Controlling frame: {end_effector_frame}")
    print("You can now interact with the GUI (rotate camera, zoom, etc.)")
    
    if not ik_enabled:
        print("\n[Visualization mode - use GUI to interact]")
        while simulation_app.is_running():
            world.step(render=True)
        simulation_app.close()
        return

    # Create input queue and start input thread
    input_queue = queue.Queue()
    input_thread = InputThread(input_queue)
    input_thread.start()

    move_steps_remaining = 0

    while simulation_app.is_running():
        # Always step the simulation to keep GUI responsive
        world.step(render=True)
        
        # Handle movement animation
        if move_steps_remaining > 0:
            move_steps_remaining -= 1
            continue
        
        # Check for new input (non-blocking)
        try:
            user_input = input_queue.get_nowait()
            
            if user_input.lower() == 'q':
                break
            
            target_pos = parse_input(user_input)
            if target_pos is None:
                print("Invalid input format. Please try again.")
                continue
                
            print(f"Attempting to reach: {target_pos}")
            
            current_pose = robot.get_world_pose()
            target_orientation = current_pose[1] 
            
            # compute_inverse_kinematics returns (joint_positions, success_flag)
            joint_positions, ik_converged = ik_solver.compute_inverse_kinematics(
                frame_name=end_effector_frame,
                target_position=target_pos,
                target_orientation=target_orientation
            )
            
            if ik_converged:
                print("Target is REACHABLE. Moving...")
                action = ArticulationAction(joint_positions=joint_positions)
                robot.get_articulation_controller().apply_action(action)
                move_steps_remaining = 60  # Let the robot move for ~1 second
            else:
                print("!!! WARNING: Target likely OUT OF REACH or invalid configuration !!!")
                print("Please enter a new target position.")
                
        except queue.Empty:
            # No input available, continue simulation
            pass

    input_thread.stop()
    simulation_app.close()

if __name__ == "__main__":
    main()
