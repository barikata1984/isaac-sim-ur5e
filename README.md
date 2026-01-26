# Isaac Sim UR5e ROS 2 Project

This repository contains a ROS 2 package for simulating a UR5e robot in NVIDIA Isaac Sim.

## Packages

### isaacsim_core

The core package for Isaac Sim integration, providing robot spawning and simulation control logic.

- **Source Layout**: `src/isaacsim_core`
- **Subpackages**:
    - `communication`: ROS 2 Node logic.
    - `spawning`: Robot spawning logic (UR5e).
    - `utilities`: Isaac Sim helper methods.
    - `dynamics`: Physics-related logic.

## Getting Started

### Prerequisites

- NVIDIA Isaac Sim installed (standard path `/isaac-sim` assumed in container).
- ROS 2 Jazzy (or compatible).

### Installation

1. Clone the repository into your workspace:
   ```bash
   cd colcon_ws/src
   git clone <repo_url>
   ```

2. Build the package:
   ```bash
   cd colcon_ws
   colcon build --symlink-install --packages-select isaacsim_core
   ```

### Running the Simulation

To launch Isaac Sim with the GUI and spawn a UR5e robot:

1. Source the workspace:
   ```bash
   source colcon_ws/install/setup.bash
   ```

2. Run the launch file:
   ```bash
   ros2 launch isaacsim_core isaac_sim_gui.launch.py
   ```

This will:
- Set up the environment for Isaac Sim.
- Launch the Isaac Sim GUI.
- Initialize the `isaac_sim_gui` node.
- Spawn a UR5e robot and start the simulation loop.
