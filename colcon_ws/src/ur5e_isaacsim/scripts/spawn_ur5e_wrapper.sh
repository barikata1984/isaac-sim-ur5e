#!/bin/bash
# Wrapper script for Isaac Sim ROS2 node

# Find the install directory
PREFIX=$(ros2 pkg prefix ur5e_isaacsim)
NODE_SCRIPT="$PREFIX/lib/python3.12/site-packages/ur5e_isaacsim/spawn_ur5e_node.py"

# Isaac Sim environment setup
export SCRIPT_DIR="/isaac-sim"
export PYTHONPATH=$PYTHONPATH:$SCRIPT_DIR/exts/isaacsim.ros2.bridge/jazzy/rclpy
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$SCRIPT_DIR/exts/isaacsim.ros2.bridge/jazzy/lib

# Use the python.sh wrapper from Isaac Sim
/isaac-sim/python.sh "$NODE_SCRIPT" "$@"
