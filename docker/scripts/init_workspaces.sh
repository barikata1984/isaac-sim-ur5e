#!/bin/bash
# ============================================================
# Workspace Initialization Script
# ============================================================
# Automatically builds underlay_ws and colcon_ws on first run
# ============================================================

set -e

PROJECT_ROOT="/workspaces/isaac-sim-ur5e"
BUILD_MARKER="$PROJECT_ROOT/.build_completed"

# Color codes for logging
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INIT]${NC} $1"
}

log_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# Check if already built
if [ -f "$BUILD_MARKER" ]; then
    log_info "Workspaces already initialized (marker found)."

    # Verify that underlay_ws is actually built
    if [ ! -f "$PROJECT_ROOT/underlay_ws/install/setup.bash" ]; then
        log_warn "Marker exists but underlay_ws not built. Re-running initialization..."
        rm -f "$BUILD_MARKER"
    else
        log_info "Skipping build."
        exit 0
    fi
fi

echo ""
echo "============================================================"
echo "  First Run: Initializing Workspaces"
echo "============================================================"
echo ""

# ==============================================
# Phase 1: Underlay Workspace
# ==============================================
log_step "Phase 1/3: Building underlay_ws..."

cd "$PROJECT_ROOT/underlay_ws"

# Ensure src directory exists
mkdir -p src

# Check if there are any packages in src
if [ -z "$(ls -A src 2>/dev/null)" ]; then
    log_warn "underlay_ws/src is empty. No packages to build."
    log_info "Note: If Dockerfile built a template, it should have been copied by entrypoint.sh"
    log_info "You can manually add packages to underlay_ws/src and run rebuild_underlay.sh"
    # Create empty install structure if no packages
    mkdir -p install
else
    log_info "Found packages in underlay_ws/src:"
    ls -1 src/

    # Build underlay
    log_info "Building underlay_ws..."
    source /opt/ros311/setup.bash
    colcon build --symlink-install \
        --cmake-args \
            -DPython3_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11 \
            -DPYTHON_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11
    log_info "✓ Underlay build complete"
fi

# ==============================================
# Phase 2: URDF Cache Generation
# ==============================================
log_step "Phase 2/3: Generating URDF cache..."

# Check if ur_description was built
if [ -d "install/ur_description" ]; then
    source install/setup.bash
    mkdir -p /tmp/ur_urdf_cache

    /isaac-sim/kit/python/bin/xacro \
        install/ur_description/share/ur_description/urdf/ur.urdf.xacro \
        ur_type:=ur5e name:=ur5e > /tmp/ur_urdf_cache/ur5e.urdf

    log_info "✓ URDF cache generated at /tmp/ur_urdf_cache/ur5e.urdf"
else
    log_warn "ur_description not found in underlay. Skipping URDF cache generation."
    log_info "URDF will be generated on-demand when needed."
fi

# ==============================================
# Phase 3: Overlay Workspace (colcon_ws)
# ==============================================
log_step "Phase 3/3: Building colcon_ws (overlay)..."

cd "$PROJECT_ROOT/colcon_ws"

# Source underlay
source /opt/ros311/setup.bash
source "$PROJECT_ROOT/underlay_ws/install/setup.bash"

# Build overlay
log_info "Building colcon_ws..."
colcon build --symlink-install --cmake-args \
    -DPython3_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11

log_info "✓ Overlay build complete"

# ==============================================
# Mark as completed
# ==============================================
echo "$(date '+%Y-%m-%d %H:%M:%S')" > "$BUILD_MARKER"

echo ""
echo "============================================================"
echo "  ✓ Workspace Initialization Complete!"
echo "============================================================"
echo ""
echo "To use the workspace, run:"
echo "  source /opt/ros311/setup.bash"
echo "  source $PROJECT_ROOT/underlay_ws/install/setup.bash"
echo "  source $PROJECT_ROOT/colcon_ws/install/setup.bash"
echo ""
