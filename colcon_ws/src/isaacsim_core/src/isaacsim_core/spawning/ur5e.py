from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from isaacsim_core.utilities.isaac_utils import get_robot_asset_path

def spawn_ur5e(world, prim_path="/World/UR5e", name="ur5e"):
    """Spawns a UR5e robot into the given world."""
    asset_path = get_robot_asset_path("ur5e")
    if not asset_path:
        raise RuntimeError("Could not find UR5e asset path")
    
    add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)
    robot = Robot(prim_path=prim_path, name=name)
    world.scene.add(robot)
    return robot
