#!/bin/bash
# ============================================================
# Docker Entrypoint Script
# ============================================================
# Fixes SSH permissions and executes the main command
# ============================================================

set -e

# --------------------------------------------
# Fix SSH permissions
# --------------------------------------------
if [ -d "/root/.ssh" ]; then
    echo "Fixing SSH permissions..."

    # Fix directory permissions
    chmod 700 /root/.ssh

    # Fix file permissions
    if [ -f "/root/.ssh/config" ]; then
        chmod 600 /root/.ssh/config
        chown root:root /root/.ssh/config
    fi

    if [ -f "/root/.ssh/id_rsa" ]; then
        chmod 600 /root/.ssh/id_rsa
        chown root:root /root/.ssh/id_rsa
    fi

    if [ -f "/root/.ssh/id_ed25519" ]; then
        chmod 600 /root/.ssh/id_ed25519
        chown root:root /root/.ssh/id_ed25519
    fi

    if [ -f "/root/.ssh/known_hosts" ]; then
        chmod 644 /root/.ssh/known_hosts
        chown root:root /root/.ssh/known_hosts
    fi

    # Fix all private key files
    find /root/.ssh -type f -name "id_*" ! -name "*.pub" -exec chmod 600 {} \; -exec chown root:root {} \; 2>/dev/null || true

    # Fix all public key files
    find /root/.ssh -type f -name "*.pub" -exec chmod 644 {} \; -exec chown root:root {} \; 2>/dev/null || true

    echo "SSH permissions fixed."
fi

# --------------------------------------------
# Initialize workspaces on first run
# --------------------------------------------
echo "[Entrypoint] Checking workspace initialization..."

PROJECT_ROOT="/workspaces/isaac-sim-ur5e"
BUILD_MARKER="$PROJECT_ROOT/.build_completed"

# Check if workspace initialization is needed
if [ ! -f "$BUILD_MARKER" ]; then
    echo "[Entrypoint] First run detected. Initializing workspaces..."

    # Copy underlay_ws template if underlay is empty
    if [ -d "/opt/underlay_ws_template" ] && [ -z "$(ls -A $PROJECT_ROOT/underlay_ws/src 2>/dev/null)" ]; then
        echo "[Entrypoint] Copying underlay_ws template from image..."
        mkdir -p "$PROJECT_ROOT/underlay_ws"
        cp -r /opt/underlay_ws_template/src "$PROJECT_ROOT/underlay_ws/" || true
        echo "[Entrypoint] Template copied."
    fi

    # Run workspace initialization script
    if [ -f "$PROJECT_ROOT/docker/scripts/init_workspaces.sh" ]; then
        echo "[Entrypoint] Running init_workspaces.sh..."
        /bin/bash "$PROJECT_ROOT/docker/scripts/init_workspaces.sh"
    else
        echo "[Entrypoint] WARNING: init_workspaces.sh not found!"
    fi
else
    echo "[Entrypoint] Workspaces already initialized."
fi

# --------------------------------------------
# Execute the main command
# --------------------------------------------
exec "$@"
