from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage

def get_robot_asset_path(robot_name="ur5e"):
    """Returns the USD path for the specified robot."""
    assets_root_path = get_assets_root_path()
    if not assets_root_path:
        return None
    
    # Map robot names to their asset paths in Isaac Sim
    asset_paths = {
        "ur5e": "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd",
    }
    
    if robot_name in asset_paths:
        return assets_root_path + asset_paths[robot_name]
    
    return None
