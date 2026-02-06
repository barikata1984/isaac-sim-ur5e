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
if [ -f "/workspaces/isaac-sim-ur5e/docker/scripts/init_workspaces.sh" ]; then
    /bin/bash /workspaces/isaac-sim-ur5e/docker/scripts/init_workspaces.sh
fi

# --------------------------------------------
# Execute the main command
# --------------------------------------------
exec "$@"
