"""List available grippers and end effectors in Isaac Sim assets."""
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core.utils.nucleus import get_assets_root_path

assets_root = get_assets_root_path()
print(f"Assets root: {assets_root}")

# Common gripper paths in Isaac Sim
gripper_paths = [
    "/Isaac/Robots/UniversalRobots/",
    "/Isaac/Robots/Robotiq/",
    "/Isaac/Props/",
    "/Isaac/EndEffectors/",
]

import omni.client

for path in gripper_paths:
    full_path = assets_root + path
    print(f"\n--- Checking: {full_path} ---")
    result, entries = omni.client.list(full_path)
    if result == omni.client.Result.OK:
        for entry in entries[:20]:  # Limit to first 20
            print(f"  {entry.relative_path}")
    else:
        print(f"  (not found or error: {result})")

simulation_app.close()
