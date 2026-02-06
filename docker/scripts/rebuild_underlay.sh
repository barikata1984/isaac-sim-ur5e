#!/bin/bash
# ============================================================
# Rebuild Underlay Workspace
# ============================================================
# Manually rebuild underlay_ws (useful for development)
# ============================================================

set -e

PROJECT_ROOT="/workspaces/isaac-sim-ur5e"
BUILD_MARKER="$PROJECT_ROOT/.build_completed"

# Color codes
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

echo ""
echo "============================================================"
echo "  Rebuilding Underlay Workspace"
echo "============================================================"
echo ""

# Remove build marker to allow re-initialization
if [ -f "$BUILD_MARKER" ]; then
    log_info "Removing build marker..."
    rm -f "$BUILD_MARKER"
fi

# Clean underlay build artifacts
log_step "Cleaning underlay_ws..."
cd "$PROJECT_ROOT/underlay_ws"
rm -rf build install log

# Ensure src directory exists
mkdir -p src

# Check packages
if [ -z "$(ls -A src 2>/dev/null)" ]; then
    log_info "underlay_ws/src is empty. No packages found."
    echo ""
    echo "To add ur_description:"
    echo "  cd $PROJECT_ROOT/underlay_ws/src"
    echo "  git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git ur_description"
    echo ""
    exit 0
fi

log_info "Building underlay_ws..."
source /opt/ros311/setup.bash
colcon build --symlink-install

# Generate URDF cache if ur_description exists
if [ -d "install/ur_description" ]; then
    log_step "Generating URDF cache..."
    source install/setup.bash
    mkdir -p /tmp/ur_urdf_cache
    /isaac-sim/kit/python/bin/xacro \
        install/ur_description/share/ur_description/urdf/ur.urdf.xacro \
        ur_type:=ur5e name:=ur5e > /tmp/ur_urdf_cache/ur5e.urdf
    log_info "✓ URDF cache generated"
fi

log_info "✓ Underlay rebuild complete"

# Rebuild colcon_ws as well
log_step "Rebuilding colcon_ws..."
cd "$PROJECT_ROOT/colcon_ws"
source "$PROJECT_ROOT/underlay_ws/install/setup.bash"
colcon build --symlink-install --cmake-args \
    -DPython3_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11

log_info "✓ Overlay rebuild complete"

# Recreate marker
echo "$(date '+%Y-%m-%d %H:%M:%S')" > "$BUILD_MARKER"

echo ""
echo "============================================================"
echo "  ✓ Rebuild Complete!"
echo "============================================================"
echo ""
