"""UR robot spawning functions for Isaac Sim."""

from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
from ur.isaac_utils import get_robot_asset_path


def spawn_ur_robot(
    world,
    robot_type: str = "ur5e",
    prim_path: str = "/World/UR",
    name: str = "ur_robot"
):
    """Spawn a UR robot into the given world.

    Args:
        world: Isaac Sim World object.
        robot_type: Type of UR robot (ur5e, ur3, ur10, ur10e, etc.).
        prim_path: USD prim path for the robot.
        name: Name identifier for the robot.

    Returns:
        Robot: Spawned robot instance.

    Raises:
        RuntimeError: If robot asset path cannot be found.
    """
    asset_path = get_robot_asset_path(robot_type)
    if not asset_path:
        raise RuntimeError(
            f"Could not find asset path for robot type: {robot_type}"
        )

    add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)
    robot = Robot(prim_path=prim_path, name=name)
    world.scene.add(robot)
    return robot


def spawn_ur5e(world, prim_path: str = "/World/UR5e", name: str = "ur5e"):
    """Spawn a UR5e robot (backward compatibility wrapper).

    Args:
        world: Isaac Sim World object.
        prim_path: USD prim path for the robot.
        name: Name identifier for the robot.

    Returns:
        Robot: Spawned robot instance.
    """
    return spawn_ur_robot(
        world,
        robot_type="ur5e",
        prim_path=prim_path,
        name=name
    )
