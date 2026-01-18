from omni.isaac.kit import SimulationApp

# Launch the simulator in the script
simulation_app = SimulationApp({"headless": False})

import sys
import numpy as np
sys.path.insert(0, "/workspaces/isaac-sim-ur5e")
from src.env_loader import create_ur5e_env
from omni.isaac.motion_generation import LulaKinematicsSolver
from omni.isaac.motion_generation import interface_config_loader
from omni.isaac.core.utils.types import ArticulationAction

def main():
    world, ur5e_robot = create_ur5e_env(simulation_app)
    if world is None:
        simulation_app.close()
        return

    # Initialize IK Solver
    try:
        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config("UR5e")
        ik_solver = LulaKinematicsSolver(**kinematics_config)
    except Exception as e:
        print(f"Error initializing IK solver: {e}")
        print("Available robots:", interface_config_loader.get_supported_robots())
        simulation_app.close()
        return

    end_effector_frame = "tool0"

    world.reset()
    
    # Warm up simulation
    for _ in range(20):
        world.step(render=True)

    print("\n--- UR5e Control Initialized ---")
    print(f"Controlling frame: {end_effector_frame}")

    # Helper to parse input
    def parse_input(input_str):
        try:
            parts = [float(x.strip()) for x in input_str.split()]
            if len(parts) != 3:
                return None
            return np.array(parts)
        except ValueError:
            return None

    while simulation_app.is_running():
        world.step(render=True)
        
        print("\nEnter target position (x y z) in meters (e.g., 0.4 0.4 0.5):")
        print("Type 'q' to quit.")
        
        user_input = input(">> ")
        if user_input.lower() == 'q':
            break
        
        target_pos = parse_input(user_input)
        if target_pos is None:
            print("Invalid input format. Please try again.")
            continue
            
        print(f"Attempting to reach: {target_pos}")
        
        current_pose = ur5e_robot.get_world_pose()
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
            ur5e_robot.get_articulation_controller().apply_action(action)
            
            for _ in range(60):
                world.step(render=True)
                
        else:
            print("!!! WARNING: Target likely OUT OF REACH or invalid configuration !!!")
            print("Please enter a new target position.")

    simulation_app.close()

if __name__ == "__main__":
    main()
