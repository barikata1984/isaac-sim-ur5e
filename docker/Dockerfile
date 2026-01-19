FROM nvcr.io/nvidia/isaac-sim:5.1.0

USER root

# Antigravity リモートサーバーのダウンロードに必要
RUN apt-get update \
    && apt-get install -y git wget curl gnupg2 lsb-release software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# ========================================
# ROS2 Jazzy Installation (Ubuntu 24.04)
# ========================================

# Add ROS2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy (base + geometry_msgs for target position)
RUN apt-get update \
    && apt-get install -y \
        ros-jazzy-ros-base \
        ros-jazzy-geometry-msgs \
        ros-jazzy-std-msgs \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Source ROS2 in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# ========================================
# Path and Workspace Configuration
# ========================================

# 標準的なコマンドが確実に使えるようにパスを設定
ENV PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${PATH}"

# Allow Isaac Sim to run as root user in container
ENV OMNI_KIT_ALLOW_ROOT=1

# python.sh へのラッパースクリプトを作成
# シンボリックリンクだとSCRIPT_DIRが正しく解決されないためラッパーを使用
RUN echo '#!/bin/bash' > /usr/local/bin/python \
    && echo 'exec /isaac-sim/python.sh "$@"' >> /usr/local/bin/python \
    && chmod +x /usr/local/bin/python

# ログイン時に自動でワークスペースに移動するように設定
RUN echo "cd /workspaces/isaac-sim-ur5e" >> /root/.bashrc

USER root
