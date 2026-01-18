"""
env_loader.py - Common environment setup for UR5e simulation.
"""
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage


def create_ur5e_env(simulation_app, headless=False):
    """
    Create the Isaac Sim environment with UR5e robot.
    
    Args:
        simulation_app: The SimulationApp instance.
        headless: Whether running in headless mode (not used here, for future).
    
    Returns:
        tuple: (world, ur5e_robot) or (None, None) if setup fails.
    """
    # Initialize the World
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Get assets path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        print("Could not find Isaac Sim assets folder.")
        return None, None

    # Path to UR5e USD
    ur5e_usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
    
    # Add robot to stage
    add_reference_to_stage(usd_path=ur5e_usd_path, prim_path="/World/UR5e")
    ur5e_robot = Robot(prim_path="/World/UR5e", name="ur5e")
    world.scene.add(ur5e_robot)

    return world, ur5e_robot
