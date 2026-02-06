# ~/.profile: executed by Bourne-compatible login shells.

# ============================================================
# ROS 2 and Isaac Sim Environment Setup
# ============================================================
# These must be set before .bashrc is sourced to ensure
# non-interactive shells (like Claude Code's Bash tool) can
# access the ROS environment.

# Isaac Sim Python environment
if [ -f "/isaac-sim/setup_python_env.sh" ]; then
    . /isaac-sim/setup_python_env.sh
fi

# ROS 2 Jazzy (apt packages for launch)
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    . /opt/ros/jazzy/setup.bash 2>/dev/null || true
fi

# ROS 2 Jazzy (Python 3.11 build)
export ROS_DISTRO=jazzy
if [ -f "/opt/ros311/setup.bash" ]; then
    . /opt/ros311/setup.bash 2>/dev/null || true
fi

# Project underlay workspace
if [ -f "/workspaces/isaac-sim-ur5e/underlay_ws/install/setup.bash" ]; then
    . /workspaces/isaac-sim-ur5e/underlay_ws/install/setup.bash
fi

# Project overlay workspace (colcon_ws)
if [ -f "/workspaces/isaac-sim-ur5e/colcon_ws/install/setup.bash" ]; then
    . /workspaces/isaac-sim-ur5e/colcon_ws/install/setup.bash
fi

# Set working directory
cd /workspaces/isaac-sim-ur5e 2>/dev/null || true

# ============================================================
# Load interactive shell configuration
# ============================================================
if [ "$BASH" ]; then
  if [ -f ~/.bashrc ]; then
    . ~/.bashrc
  fi
fi

mesg n 2> /dev/null || true
