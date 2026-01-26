#!/bin/bash
# Wrapper script for Isaac Sim GUI with ROS 2 integration

# Find the install directory
# Find the main.py script path using python3 (which should be aware of the package path after sourcing setup.bash)
NODE_SCRIPT=$(python3 -c "import isaacsim_core; import os; print(os.path.join(os.path.dirname(isaacsim_core.__file__), 'main.py'))" 2>/dev/null)

if [ -z "$NODE_SCRIPT" ] || [ ! -f "$NODE_SCRIPT" ]; then
    # Fallback: search in typical ROS2 install paths
    PACKAGE_NAME="isaacsim_core"
    PREFIX=$(ros2 pkg prefix $PACKAGE_NAME 2>/dev/null)
    if [ -n "$PREFIX" ]; then
        NODE_SCRIPT=$(find -L "$PREFIX" -name "main.py" | grep "/isaacsim_core/main.py" | head -n 1)
    fi
fi

# Isaac Sim environment setup (standard paths in the container)
export SCRIPT_DIR="/isaac-sim"
# Prepend Isaac Sim's ROS2 bridge paths to avoid Python version mismatch with system ROS2
export PYTHONPATH=$SCRIPT_DIR/exts/isaacsim.ros2.bridge/jazzy/rclpy:$PYTHONPATH
export LD_LIBRARY_PATH=$SCRIPT_DIR/exts/isaacsim.ros2.bridge/jazzy/lib:$LD_LIBRARY_PATH

# Use the python.sh wrapper from Isaac Sim
/isaac-sim/python.sh "$NODE_SCRIPT" "$@"
