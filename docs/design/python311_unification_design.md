# Python 3.11 完全統一 詳細設計書

## 1. 概要

### 1.1 目的
Isaac Sim (Python 3.11) と ROS 2 Jazzy の Python 環境を完全に統一し、
以下の問題を解決する:
- xacro 実行時の Python バージョン競合
- Pinocchio 等のライブラリのバージョン不整合
- PYTHONPATH の複雑な管理

### 1.2 現状の問題
```
現在のDockerfile:
  ┌─────────────────────────────────┐
  │ Isaac Sim 5.1.0 (Python 3.11)  │
  │   ↓ apt install                │
  │ ROS 2 Jazzy (Python 3.12)      │ ← 競合発生
  └─────────────────────────────────┘
```

### 1.3 目標アーキテクチャ
```
新しいDockerfile:
  ┌─────────────────────────────────────────────────┐
  │ Stage 1: ros311-builder (Ubuntu 24.04)         │
  │   Python 3.11 + ROS 2 Jazzy ソースビルド       │
  │   → /workspace/ros311_ws/install を生成        │
  └─────────────────────────────────────────────────┘
                         ↓ COPY
  ┌─────────────────────────────────────────────────┐
  │ Stage 2: Isaac Sim 5.1.0 (Python 3.11)         │
  │   + /opt/ros311 (Python 3.11 版 ROS)           │
  │   + 共通ライブラリ (Pinocchio, etc.)           │
  │   → 完全な Python 3.11 統一環境                │
  └─────────────────────────────────────────────────┘
```

---

## 2. Multi-Stage Dockerfile 設計

### 2.1 Stage 1: ros311-builder

**目的**: Python 3.11 で ROS 2 Jazzy の必要最小限パッケージをソースビルド

```dockerfile
# ============================================================
# Stage 1: ROS 2 Jazzy Python 3.11 Builder
# ============================================================
FROM ubuntu:24.04 AS ros311-builder

ENV ROS_DISTRO=jazzy
ENV DEBIAN_FRONTEND=noninteractive

# Python 3.11 インストール (deadsnakes PPA)
RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository -y ppa:deadsnakes/ppa \
    && apt-get install -y \
        python3.11 python3.11-dev python3.11-distutils python3.11-venv \
    && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.11 1

# pip セットアップ
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.11

# ROS 2 ソースビルド依存関係
RUN apt-get install -y \
    build-essential cmake git wget curl gnupg2 lsb-release \
    pkg-config libbullet-dev libasio-dev libtinyxml2-dev \
    libcunit1-dev libacl1-dev liblttng-ust-dev libconsole-bridge-dev

# Python パッケージ
RUN python3.11 -m pip install \
    empy==3.3.4 catkin_pkg lark numpy pybind11 PyYAML setuptools==70.0.0

# ROS キーとリポジトリ
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    && apt-key add ros.asc
RUN echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# rosinstall_generator でパッケージリスト生成
RUN apt-get update && apt-get install -y python3-rosinstall-generator ros-dev-tools

WORKDIR /workspace/ros311_ws

# 必要なパッケージのみ選択的にビルド（詳細は 2.2 節）
RUN rosinstall_generator --deps --rosdistro jazzy \
    rcutils rcl rmw rclpy \
    std_msgs sensor_msgs geometry_msgs nav_msgs \
    tf2 tf2_ros tf2_msgs \
    ros2topic ros2node ros2run ros2pkg \
    ros_environment \
    > ros2.jazzy.rosinstall \
    && mkdir -p src && vcs import src < ros2.jazzy.rosinstall

# rosdep
RUN rosdep init || true && rosdep update

# Python 3.11 でビルド
RUN colcon build --merge-install --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3.11 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3.11 \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.11 \
    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.11.so

# 必要な共有ライブラリをコピー
RUN cp /usr/lib/x86_64-linux-gnu/libtinyxml2.so* install/lib/ || true
```

### 2.2 ビルド対象 ROS パッケージ

| カテゴリ | パッケージ | 必要性 |
|---------|-----------|--------|
| Core | rcutils, rcl, rmw, rclpy | 必須 |
| Messages | std_msgs, sensor_msgs, geometry_msgs, nav_msgs | 必須 |
| TF | tf2, tf2_ros, tf2_msgs | 必須 (kinematics で使用) |
| CLI | ros2topic, ros2node, ros2run, ros2pkg | 推奨 |
| Environment | ros_environment | 必須 |

**除外パッケージ** (apt版を使用):
- rviz2 (Python依存なし、バイナリで動作)
- rosbag2 (apt版で十分)
- ros2launch (apt版で十分)

### 2.3 Stage 2: Final Image

```dockerfile
# ============================================================
# Stage 2: Final Isaac Sim + ROS Python 3.11
# ============================================================
FROM nvcr.io/nvidia/isaac-sim:5.1.0

USER root

# 基本ツール
RUN apt-get update && apt-get install -y \
    git wget curl gnupg2 lsb-release locales software-properties-common \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

# ROS 2 apt リポジトリ追加（launch, rosbag 等のバイナリ用）
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

# Python依存のない ROS パッケージ (apt)
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros2launch \
    ros-jazzy-launch-xml \
    ros-jazzy-launch-yaml \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Python 3.11版 ROS をコピー
COPY --from=ros311-builder /workspace/ros311_ws/install /opt/ros311

# Isaac Sim Python に追加パッケージインストール
RUN /isaac-sim/python.sh -m pip install \
    pin==2.7.0 \
    pymlg \
    scipy \
    matplotlib

# 共有ライブラリパス設定
ENV LD_LIBRARY_PATH=/opt/ros311/lib:${LD_LIBRARY_PATH}

# Python コマンドを Isaac Sim Python にリンク
RUN ln -sf /isaac-sim/python.sh /usr/local/bin/python

# 環境設定
ENV ROS_DISTRO=jazzy
ENV PYTHONPATH=/opt/ros311/lib/python3.11/site-packages:${PYTHONPATH}

# シェル設定
RUN echo 'source /opt/ros311/setup.bash' >> /root/.bashrc \
    && echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo 'if [ -f "/workspaces/isaac-sim-ur5e/colcon_ws/install/setup.bash" ]; then \
    source /workspaces/isaac-sim-ur5e/colcon_ws/install/setup.bash; fi' >> /root/.bashrc \
    && echo 'cd /workspaces/isaac-sim-ur5e' >> /root/.bashrc
```

---

## 3. docker-compose.yml 設計

### 3.1 変更点

```yaml
services:
  isaac-sim:
    build:
      context: ..
      dockerfile: docker/Dockerfile
      # ビルドキャッシュ活用のため target 指定可能
      args:
        - BUILDKIT_INLINE_CACHE=1
    container_name: isaac-sim-ur5e
    network_mode: host
    shm_size: "2gb"  # ビルド時メモリ増加
    environment:
      - ACCEPT_EULA=Y
      - PRIVACY_CONSENT=Y
      - DISPLAY=${DISPLAY}
      # Python 3.11 統一環境変数
      - PYTHONPATH=/opt/ros311/lib/python3.11/site-packages
      - ROS_PYTHON_VERSION=3
    volumes:
      # X11 socket for GUI
      - /tmp/.X11-unix:/tmp/.X11-unix
      # Isaac Sim caches
      - ${HOME}/docker/isaac-sim/cache/main:/root/.cache/isaac-sim
      - ${HOME}/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache
      - ${HOME}/docker/isaac-sim/data:/root/.local/share/ov/data
      # SSH and Git config from host
      - ${HOME}/.ssh:/root/.ssh:ro
      - ${HOME}/.gitconfig:/root/.gitconfig:ro
      # Workspace
      - ..:/workspaces/isaac-sim-ur5e
      # URDF キャッシュ永続化
      - ur_urdf_cache:/tmp/ur_urdf_cache
    devices:
      - /dev/dri:/dev/dri
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [ gpu ]
    stdin_open: true
    tty: true
    command: sleep infinity

# Named volume for URDF cache persistence
volumes:
  ur_urdf_cache:
```

### 3.2 主な変更点

| 項目 | 旧 | 新 |
|------|-----|-----|
| shm_size | 1gb | 2gb (ビルド安定化) |
| PYTHONPATH | なし | /opt/ros311/lib/python3.11/site-packages |
| volumes | - | ur_urdf_cache 追加 |

---

## 4. devcontainer.json 設計

### 4.1 変更後の内容

```json
{
    "name": "Isaac Sim UR5e (Python 3.11 Unified)",
    "dockerComposeFile": "../docker/docker-compose.yml",
    "service": "isaac-sim",
    "workspaceFolder": "/workspaces/isaac-sim-ur5e",
    "remoteUser": "root",
    "overrideCommand": true,

    "customizations": {
        "vscode": {
            "extensions": [
                "ms-python.python",
                "ms-python.vscode-pylance",
                "ms-python.debugpy",
                "ms-vscode.cmake-tools",
                "ms-iot.vscode-ros"
            ],
            "settings": {
                "python.defaultInterpreterPath": "/isaac-sim/python.sh",
                "python.analysis.extraPaths": [
                    "/opt/ros311/lib/python3.11/site-packages",
                    "/workspaces/isaac-sim-ur5e/colcon_ws/install/lib/python3.11/site-packages"
                ],
                "python.envFile": "${workspaceFolder}/.env",
                "ros.distro": "jazzy",
                "terminal.integrated.env.linux": {
                    "PYTHONPATH": "/opt/ros311/lib/python3.11/site-packages:${env:PYTHONPATH}"
                }
            }
        }
    },

    "postCreateCommand": "cd /workspaces/isaac-sim-ur5e/colcon_ws && colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/isaac-sim/python.sh",

    "postStartCommand": "source /opt/ros311/setup.bash && source /opt/ros/jazzy/setup.bash",

    "remoteEnv": {
        "ROS_DISTRO": "jazzy",
        "PYTHONPATH": "/opt/ros311/lib/python3.11/site-packages"
    }
}
```

### 4.2 主な変更点

| 項目 | 説明 |
|------|------|
| name | Python 3.11 統一を明示 |
| extensions | ROS extension 追加 |
| python.defaultInterpreterPath | Isaac Sim Python 明示 |
| python.analysis.extraPaths | ros311 パス追加 |
| postCreateCommand | Python 3.11 で colcon ビルド |
| remoteEnv | 環境変数設定 |

---

## 5. colcon_ws ビルド設定

### 5.1 Python 3.11 統一ビルドコマンド

```bash
cd /workspaces/isaac-sim-ur5e/colcon_ws

# ROS Python 3.11 環境を source
source /opt/ros311/setup.bash

# colcon ビルド (Python 3.11 明示)
colcon build --symlink-install --cmake-args \
    -DPython3_EXECUTABLE=/isaac-sim/python.sh \
    -DPYTHON_EXECUTABLE=/isaac-sim/python.sh
```

### 5.2 パッケージごとの考慮事項

| パッケージ | Python依存 | 対応 |
|-----------|-----------|------|
| kinematics | Pinocchio, xacro | URDF キャッシュ使用を維持 |
| trajectory | numpy, scipy | Isaac Sim Python で動作 |
| iparam_identification | numpy, scipy, pinocchio | Isaac Sim Python で動作 |
| traj_follower | rclpy | ros311 経由で import |

---

## 6. 環境変数設計

### 6.1 最終的な環境変数

```bash
# Python 関連
PYTHONPATH=/opt/ros311/lib/python3.11/site-packages:/workspaces/isaac-sim-ur5e/colcon_ws/install/lib/python3.11/site-packages

# ROS 関連
ROS_DISTRO=jazzy
AMENT_PREFIX_PATH=/opt/ros311:/opt/ros/jazzy

# ライブラリパス
LD_LIBRARY_PATH=/opt/ros311/lib:/opt/ros/jazzy/lib

# Python 実行
# /usr/local/bin/python -> /isaac-sim/python.sh
```

### 6.2 .env ファイル (オプション)

```bash
# /workspaces/isaac-sim-ur5e/.env
PYTHONPATH=/opt/ros311/lib/python3.11/site-packages
ROS_DISTRO=jazzy
```

---

## 7. 移行計画

### Phase 1: 準備 (影響なし)
1. ros311-builder ステージを既存 Dockerfile の前に追加
2. 既存機能に影響なくビルドテスト

### Phase 2: 統合
1. Stage 2 で ros311 をコピー・設定
2. docker-compose.yml 更新
3. devcontainer.json 更新

### Phase 3: 検証
1. `ros2 topic list` 動作確認
2. `python -c "import rclpy"` 確認
3. kinematics パッケージテスト
4. iparam_identification 全体テスト

### Phase 4: クリーンアップ
1. kinematics.py の PYTHONPATH クリーニング処理を簡略化
2. 不要な workaround 削除

---

## 8. リスクと対策

| リスク | 影響 | 対策 |
|--------|------|------|
| ros311 ビルド失敗 | ビルド不可 | 最小パッケージセットで開始、段階的追加 |
| Isaac Sim Python との互換性 | import エラー | site-packages 順序調整 |
| ビルド時間増加 | CI遅延 | Docker layer キャッシュ活用 |
| apt ROS との競合 | 不安定動作 | apt ROS は Python非依存パッケージのみ |

---

## 9. テスト項目

### 9.1 基本テスト
```bash
# Python バージョン確認
python --version  # 3.11.x

# ROS 2 基本動作
ros2 topic list
ros2 node list

# rclpy import
python -c "import rclpy; print('rclpy OK')"

# Pinocchio import
python -c "import pinocchio; print('pinocchio OK')"
```

### 9.2 統合テスト
```bash
# kinematics
ros2 run kinematics kinematics_node

# iparam_identification
cd /workspaces/isaac-sim-ur5e/colcon_ws
python src/iparam_identification/scripts/run_identification_test.py
```

---

## 10. ファイル変更サマリ

| ファイル | 変更内容 |
|---------|---------|
| docker/Dockerfile | Multi-stage ビルドに全面改修 |
| docker/docker-compose.yml | shm_size, volumes, env 追加 |
| .devcontainer/devcontainer.json | Python/ROS 設定追加 |
| colcon_ws/ | ビルドコマンド変更 (CMake args) |

---

## 付録 A: 完全な Dockerfile

詳細は [docker/Dockerfile.python311](../docker/Dockerfile.python311) を参照
（実装時に作成）
