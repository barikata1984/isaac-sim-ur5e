# test_ur5e.py
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"

# Add UR5e robot to the stage
add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR5e")

# Create Robot object and add to scene
ur5e_robot = Robot(prim_path="/World/UR5e", name="ur5e")
world.scene.add(ur5e_robot)

world.reset()
while simulation_app.is_running():
    world.step(render=True)
simulation_app.close()