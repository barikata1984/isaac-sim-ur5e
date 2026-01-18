# test_ur5e.py
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import sys
sys.path.insert(0, "/workspaces/isaac-sim-ur5e")
from src.env_loader import create_ur5e_env

world, ur5e_robot = create_ur5e_env(simulation_app)
if world is None:
    simulation_app.close()
    sys.exit(1)

world.reset()
while simulation_app.is_running():
    world.step(render=True)
simulation_app.close()