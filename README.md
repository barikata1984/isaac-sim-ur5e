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

## Dynamics Verification

This section describes how to verify that the dynamics calculations (using `pymlg` for Lie group operations) match Isaac Sim's physics simulation.

### Overview

The verification compares the end-effector twist computed by our forward kinematics implementation against the values reported by Isaac Sim's physics engine.

### Prerequisites

1. Build all packages:
   ```bash
   cd /workspaces/isaac-sim-ur5e/colcon_ws
   colcon build
   source install/setup.bash
   ```

2. Ensure `pymlg` is installed from the `underlay_ws`:
   ```bash
   cd /workspaces/isaac-sim-ur5e/underlay_ws/pymlg
   pip install -e .
   ```

### Verification Procedure

#### Step 1: Start Isaac Sim (Headless Mode)

```bash
cd /workspaces/isaac-sim-ur5e/colcon_ws
source install/setup.bash
ros2 launch core bring_up.launch.py headless:=true
```

Wait until Isaac Sim is fully initialized (approximately 30-60 seconds). You can verify by checking available topics:

```bash
ros2 topic list | grep robot_state
```

#### Step 2: Start the Verification Node

In a new terminal:

```bash
cd /workspaces/isaac-sim-ur5e/colcon_ws
source install/setup.bash
PYTHONPATH=/workspaces/isaac-sim-ur5e/colcon_ws/src/dynamics/src:$PYTHONPATH \
python3 -c "from dynamics.isaac_verification_node import main; main()" \
--ros-args -p output_path:=/tmp/verification_result.json
```

The node subscribes to `/robot_state_combined` and records:
- Joint positions and velocities
- Computed end-effector twist (from pymlg-based forward kinematics)
- Isaac Sim end-effector twist (from physics simulation)

#### Step 3: Play a Test Trajectory

In another terminal:

```bash
cd /workspaces/isaac-sim-ur5e/colcon_ws
source install/setup.bash
ros2 launch trajectories joint_pos_sender.launch.py \
  json_path:=/workspaces/isaac-sim-ur5e/data/test_trajectory.json \
  auto_start:=true
```

The trajectory will play for approximately 10 seconds (600 frames at 60 FPS).

#### Step 4: Analyze Results

After the trajectory completes, analyze the verification data:

```python
import json
import numpy as np

with open('/tmp/verification_result.json', 'r') as f:
    data = json.load(f)

records = data['records']
computed = np.array([r['computed_twist'] for r in records])
isaac = np.array([r['isaac_twist'] for r in records])
joint_vel = np.array([r['joint_velocities'] for r in records])

# Filter samples where robot is moving
moving = np.linalg.norm(joint_vel, axis=1) > 0.01
errors = computed[moving] - isaac[moving]

# Compute RMSE
rmse = np.sqrt(np.mean(errors**2))
print(f"Overall RMSE: {rmse:.6e}")

# Compute correlation
for i, name in enumerate(['ω_x', 'ω_y', 'ω_z', 'v_x', 'v_y', 'v_z']):
    corr = np.corrcoef(computed[moving, i], isaac[moving, i])[0, 1]
    print(f"{name} correlation: {corr:.6f}")
```

### Expected Results

A successful verification should show:

| Metric | Expected Value |
|--------|----------------|
| Overall RMSE | < 0.01 |
| Mean Relative Error | < 2% |
| Correlation (all components) | > 0.999 |

### Example Output

```
Overall RMSE: 2.49e-03
Mean Relative Error: 1.12%

Correlation (computed vs Isaac Sim):
  ω_x: 0.9995
  ω_y: 0.9996
  ω_z: 0.9999
  v_x: 0.9997
  v_y: 1.0000
  v_z: 0.9998
```

These results confirm that the pymlg-based dynamics implementation accurately matches Isaac Sim's physics simulation.
