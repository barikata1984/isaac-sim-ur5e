#!/bin/bash
# Wrapper script to run Isaac Sim Python scripts with ROS2 support
# This script isolates Isaac Sim's ROS2 bridge from container's ROS2

# Allow running as root user in container
export OMNI_KIT_ALLOW_ROOT=1

# Clear container's ROS2 environment to avoid conflicts
unset ROS_DISTRO
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset ROS_PYTHON_VERSION
unset PYTHONPATH

# Remove /opt/ros paths from LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')

# Set Isaac Sim ROS2 Bridge environment
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib:$LD_LIBRARY_PATH

# Run Isaac Sim python with all arguments
exec /isaac-sim/python.sh "$@"
