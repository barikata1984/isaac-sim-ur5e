"""Check the structure of robot USD files."""
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Usd, UsdGeom
import omni.usd

assets_root = get_assets_root_path()
print(f"Assets root: {assets_root}")

# Check Franka
franka_path = assets_root + "/Isaac/Robots/Franka/franka.usd"
print(f"\n--- Checking Franka: {franka_path} ---")

add_reference_to_stage(usd_path=franka_path, prim_path="/World/franka_test")

stage = omni.usd.get_context().get_stage()
for prim in stage.Traverse():
    if prim.GetPath().pathString.startswith("/World/franka"):
        print(f"  {prim.GetPath()} - Type: {prim.GetTypeName()}")

# Step a few times
from omni.isaac.core import World
world = World()
for _ in range(10):
    world.step(render=True)

simulation_app.close()
