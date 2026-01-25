"""
env_loader.py - Common environment setup for manipulator simulation.
"""
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage


# Registry of available manipulators: name -> (relative_path, end_effector_frame, ik_name)
# ik_name is the name used by interface_config_loader (None if IK not supported)
MANIPULATOR_REGISTRY = {
    # Universal Robots - All tested and working
    "ur3": ("/Isaac/Robots/UniversalRobots/ur3/ur3.usd", "tool0", "UR3"),
    "ur3e": ("/Isaac/Robots/UniversalRobots/ur3e/ur3e.usd", "tool0", "UR3e"),
    "ur5": ("/Isaac/Robots/UniversalRobots/ur5/ur5.usd", "tool0", "UR5"),
    "ur5e": ("/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd", "tool0", "UR5e"),
    "ur10": ("/Isaac/Robots/UniversalRobots/ur10/ur10.usd", "tool0", "UR10"),
    "ur10e": ("/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd", "tool0", "UR10e"),
    "ur16e": ("/Isaac/Robots/UniversalRobots/ur16e/ur16e.usd", "tool0", "UR16e"),
}


def get_available_manipulators():
    """Return list of available manipulator names."""
    return list(MANIPULATOR_REGISTRY.keys())


def get_manipulator_info(manipulator: str):
    """
    Get manipulator asset path and end effector frame name.
    
    Args:
        manipulator: Name of the manipulator (e.g., "ur5e")
    
    Returns:
        tuple: (relative_usd_path, end_effector_frame, ik_name) or None if not found
    """
    return MANIPULATOR_REGISTRY.get(manipulator.lower())


def create_manipulator_env(simulation_app, manipulator: str = "ur5e"):
    """
    Create the Isaac Sim environment with a specified manipulator.
    
    Args:
        simulation_app: The SimulationApp instance.
        manipulator: Name of the manipulator to load (default: "ur5e").
                    Use get_available_manipulators() to see all options.
    
    Returns:
        tuple: (world, robot, end_effector_frame, ik_name) or (None, None, None, None) if setup fails.
    """
    manipulator_info = get_manipulator_info(manipulator)
    if manipulator_info is None:
        print(f"Error: Unknown manipulator '{manipulator}'")
        print(f"Available manipulators: {get_available_manipulators()}")
        return None, None, None, None
    
    relative_path, end_effector_frame, ik_name = manipulator_info
    
    # Initialize the World
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Get assets path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        print("Could not find Isaac Sim assets folder.")
        return None, None, None, None

    # Full path to manipulator USD
    manipulator_usd_path = assets_root_path + relative_path
    prim_path = f"/World/{manipulator.upper()}"
    
    # Add robot to stage
    add_reference_to_stage(usd_path=manipulator_usd_path, prim_path=prim_path)
    robot = Robot(prim_path=prim_path, name=manipulator)
    world.scene.add(robot)

    print(f"Loaded manipulator: {manipulator}")
    print(f"End effector frame: {end_effector_frame}")

    return world, robot, end_effector_frame, ik_name


# Backward compatibility alias
def create_ur5e_env(simulation_app):
    """Legacy function for UR5e. Use create_manipulator_env instead."""
    world, robot, _, _ = create_manipulator_env(simulation_app, "ur5e")
    return world, robot
