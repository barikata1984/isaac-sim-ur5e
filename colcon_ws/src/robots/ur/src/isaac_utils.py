"""Isaac Sim asset path utilities for UR robots."""

from isaacsim.storage.native import get_assets_root_path

# Mapping of robot types to Isaac Sim asset paths
ROBOT_ASSET_MAPPING = {
    "ur5e": "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd",
    "ur3": "/Isaac/Robots/UniversalRobots/ur3/ur3.usd",
    "ur10": "/Isaac/Robots/UniversalRobots/ur10/ur10.usd",
    "ur10e": "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd",
}


def get_robot_asset_path(robot_type: str) -> str:
    """Get the full asset path for a given robot type.

    Args:
        robot_type: Type of robot (e.g., 'ur5e', 'ur3', 'ur10e').

    Returns:
        Full path to the robot USD asset, or empty string if not found.
    """
    if robot_type not in ROBOT_ASSET_MAPPING:
        return ""

    assets_root_path = get_assets_root_path()
    if not assets_root_path:
        return ""

    return assets_root_path + ROBOT_ASSET_MAPPING[robot_type]
