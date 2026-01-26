FROM nvcr.io/nvidia/isaac-sim:5.1.0

USER root

# Antigravity リモートサーバーのダウンロードに必要
# 基本ツールのインストール
RUN apt-get update \
    && apt-get install -y \
        git \
        wget \
        curl \
        gnupg2 \
        lsb-release \
        locales \
        software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# ロケール設定
RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# 標準的なコマンドが確実に使えるようにパスを設定
ENV PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${PATH}"

# ========================================
# ROS2 Jazzy Installation
# ========================================

# ROS2 GPG key の追加
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS2 apt リポジトリの追加
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Jazzy パッケージのインストール
RUN apt-get update \
    && apt-get install -y \
        ros-jazzy-ros-base \
        ros-dev-tools \
        python3-colcon-common-extensions \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# rosdep の初期化とアップデート
RUN rosdep init || true \
    && rosdep update --rosdistro jazzy

# ROS2 環境変数
ENV ROS_DISTRO=jazzy

# python.sh へのシンボリックリンクを作成
# これにより、どこからでも "python" と打つだけで Isaac Sim 環境が立ち上がります
RUN ln -s /isaac-sim/python.sh /usr/local/bin/python

# シェルログイン時に ROS2 環境とワークスペースをセットアップ
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "cd /workspaces/isaac-sim-ur5e" >> /root/.bashrc

USER root
