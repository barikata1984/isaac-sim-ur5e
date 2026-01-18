#!/bin/bash
# Wrapper script to run Isaac Sim with ROS2 support using internal libraries

# Set ROS2 environment variables for Isaac Sim's internal ROS2 Bridge
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib:$LD_LIBRARY_PATH

# Add Python path for ROS2 modules from Isaac Sim bridge
export PYTHONPATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/rclpy:$PYTHONPATH

# Run Isaac Sim python with all arguments
exec /isaac-sim/python.sh "$@"
