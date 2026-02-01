"""UR robot spawning functions for Isaac Sim."""

from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
from ur.isaac_utils import get_robot_asset_path


def enable_self_collision(prim_path: str) -> bool:
    """Enable self-collision detection for an articulation.

    The UR5e USD asset has the PhysxArticulationAPI on the 'root_joint' child prim,
    not on the robot root prim. This function finds the correct prim with the
    self-collision attribute and enables it.

    Args:
        prim_path: USD prim path of the robot root.

    Returns:
        True if self-collision was enabled successfully.
    """
    try:
        from pxr import PhysxSchema
        from omni.usd import get_context

        stage = get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)

        if not prim or not prim.IsValid():
            print(f"[Self-Collision] Prim not found: {prim_path}")
            return False

        # Find the prim with PhysxArticulationAPI that has self-collision attribute
        # For UR5e, this is typically at {prim_path}/root_joint
        prims_to_check = [prim_path, f"{prim_path}/root_joint"]

        for check_path in prims_to_check:
            check_prim = stage.GetPrimAtPath(check_path)
            if not check_prim or not check_prim.IsValid():
                continue

            physx_articulation = PhysxSchema.PhysxArticulationAPI.Get(stage, check_path)
            if physx_articulation:
                self_collision_attr = physx_articulation.GetEnabledSelfCollisionsAttr()
                if self_collision_attr:
                    current_value = self_collision_attr.Get()
                    print(f"[Self-Collision] Found at {check_path}, current value: {current_value}")
                    self_collision_attr.Set(True)
                    print(f"[Self-Collision] ENABLED at {check_path}")
                    return True

        print(f"[Self-Collision] No PhysxArticulationAPI found under {prim_path}")
        return False

    except Exception as e:
        print(f"[Self-Collision] Failed to enable: {e}")
        import traceback
        traceback.print_exc()
        return False


def spawn_ur_robot(
    world,
    robot_type: str = "ur5e",
    prim_path: str = "/World/UR",
    name: str = "ur_robot",
    enable_self_collisions: bool = True
):
    """Spawn a UR robot into the given world.

    Args:
        world: Isaac Sim World object.
        robot_type: Type of UR robot (ur5e, ur3, ur10, ur10e, etc.).
        prim_path: USD prim path for the robot.
        name: Name identifier for the robot.
        enable_self_collisions: Whether to enable self-collision detection.

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

    # Enable self-collision if requested
    if enable_self_collisions:
        enable_self_collision(prim_path)

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
