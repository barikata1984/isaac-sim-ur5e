# Python 3.11 統一 - アプローチB アクションプラン

## Isaac Sim Python を直接システム Python として使用

---

## 概要

Isaac Sim に同梱されている Python 3.11 を直接使用して ROS 2 Jazzy をビルドし、
真の単一 Python 環境を実現する。

### 確認済みの前提条件

| 項目 | パス | 状態 |
|------|------|------|
| Python バイナリ | `/isaac-sim/kit/python/bin/python3.11` | ✓ 存在 |
| Python ヘッダー | `/isaac-sim/kit/python/include/python3.11/Python.h` | ✓ 存在 |
| Python ライブラリ | `/isaac-sim/kit/python/lib/libpython3.11.so` | ✓ 存在 |
| pip | pip 24.3.1 | ✓ 動作確認済 |
| site-packages | `/isaac-sim/kit/python/lib/python3.11/site-packages` | ✓ 確認済 |

---

## フェーズ構成

```
Phase 1: 環境変数・パス設定
    ↓
Phase 2: ROS 2 ビルド依存関係インストール
    ↓
Phase 3: ROS 2 ソースビルド
    ↓
Phase 4: colcon_ws ビルド設定
    ↓
Phase 5: Docker/devcontainer 更新
    ↓
Phase 6: 検証・テスト
```

---

## Phase 1: 環境変数・パス設定

### 1.1 目的
Isaac Sim Python をシステム全体で使用可能にする

### 1.2 タスク

| # | タスク | コマンド/設定 |
|---|--------|--------------|
| 1.1 | Python シンボリックリンク作成 | `ln -sf /isaac-sim/kit/python/bin/python3.11 /usr/local/bin/python3` |
| 1.2 | python コマンドリンク | `ln -sf /isaac-sim/kit/python/bin/python3.11 /usr/local/bin/python` |
| 1.3 | update-alternatives 設定 | `update-alternatives --install /usr/bin/python3 python3 /isaac-sim/kit/python/bin/python3.11 100` |
| 1.4 | 環境変数設定 | 下記参照 |

### 1.3 環境変数

```bash
# /etc/environment または Dockerfile ENV
ISAAC_PYTHON=/isaac-sim/kit/python/bin/python3.11
ISAAC_PYTHON_INCLUDE=/isaac-sim/kit/python/include/python3.11
ISAAC_PYTHON_LIB=/isaac-sim/kit/python/lib
PYTHONPATH=/isaac-sim/kit/python/lib/python3.11/site-packages

# CMake 用
Python3_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11
PYTHON_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11
PYTHON_INCLUDE_DIR=/isaac-sim/kit/python/include/python3.11
PYTHON_LIBRARY=/isaac-sim/kit/python/lib/libpython3.11.so
```

### 1.4 成果物
- シンボリックリンク設定済み
- 環境変数定義済み

---

## Phase 2: ROS 2 ビルド依存関係インストール

### 2.1 目的
ROS 2 ソースビルドに必要なシステムパッケージと Python パッケージをインストール

### 2.2 システムパッケージ

```bash
apt-get update && apt-get install -y \
    # ビルドツール
    build-essential \
    cmake \
    git \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    pkg-config \
    # ROS 2 依存ライブラリ
    libbullet-dev \
    libasio-dev \
    libtinyxml2-dev \
    libcunit1-dev \
    libacl1-dev \
    liblttng-ust-dev \
    libconsole-bridge-dev \
    libyaml-cpp-dev \
    # ロケール
    locales
```

### 2.3 Python パッケージ (Isaac Sim Python にインストール)

```bash
/isaac-sim/kit/python/bin/python3.11 -m pip install \
    empy==3.3.4 \
    catkin_pkg \
    lark \
    colcon-common-extensions \
    rosdep \
    rosinstall-generator \
    vcstool \
    "pybind11[global]"
```

### 2.4 ROS リポジトリ設定

```bash
# GPG キー
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc
apt-key add ros.asc

# リポジトリ追加
echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list
```

### 2.5 成果物
- システム依存関係インストール済み
- Python ビルドツールインストール済み

---

## Phase 3: ROS 2 ソースビルド

### 3.1 目的
Isaac Sim Python で ROS 2 最小パッケージセットをビルド

### 3.2 ビルド対象パッケージ

| カテゴリ | パッケージ | 必要性 |
|---------|-----------|--------|
| Core | rcutils, rcl, rmw, rclpy | 必須 |
| Messages | std_msgs, sensor_msgs, geometry_msgs, nav_msgs | 必須 |
| TF | tf2, tf2_ros, tf2_msgs | 必須 |
| CLI | ros2topic, ros2node, ros2run, ros2pkg | 推奨 |
| Common | common_interfaces, rosgraph_msgs | 必須 |

### 3.3 ソース取得

```bash
mkdir -p /opt/ros2_jazzy_ws/src
cd /opt/ros2_jazzy_ws

rosinstall_generator --deps --rosdistro jazzy \
    rcutils rcl rmw rclpy \
    std_msgs sensor_msgs geometry_msgs nav_msgs \
    tf2 tf2_ros tf2_msgs \
    ros2topic ros2node ros2run ros2pkg \
    ros_environment common_interfaces rosgraph_msgs \
    > ros2.jazzy.rosinstall

vcs import src < ros2.jazzy.rosinstall
```

### 3.4 ビルドコマンド

```bash
cd /opt/ros2_jazzy_ws

source /isaac-sim/setup_python_env.sh

colcon build --merge-install \
    --install-base /isaac-sim/kit/python/lib/python3.11/site-packages/ros2 \
    --cmake-args \
        -DPython3_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11 \
        -DPYTHON_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11 \
        -DPYTHON_INCLUDE_DIR=/isaac-sim/kit/python/include/python3.11 \
        -DPYTHON_LIBRARY=/isaac-sim/kit/python/lib/libpython3.11.so \
        -DCMAKE_INSTALL_RPATH=/isaac-sim/kit/python/lib
```

**代替案**: site-packages 外にインストールして PYTHONPATH で追加

```bash
colcon build --merge-install \
    --install-base /opt/ros311 \
    --cmake-args ...
```

### 3.5 セットアップスクリプト生成

```bash
# /opt/ros311/setup.bash が生成される
# これを source することで ROS 2 環境が有効になる
```

### 3.6 成果物
- ROS 2 パッケージビルド完了
- setup.bash 生成

---

## Phase 4: colcon_ws ビルド設定

### 4.1 目的
プロジェクトの ROS パッケージを Isaac Sim Python でビルド

### 4.2 ビルドコマンド

```bash
cd /workspaces/isaac-sim-ur5e/colcon_ws

# ROS 2 環境を source
source /opt/ros311/setup.bash

# ビルド
colcon build --symlink-install \
    --cmake-args \
        -DPython3_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11 \
        -DPYTHON_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11
```

### 4.3 既存パッケージの調整

| パッケージ | 必要な変更 |
|-----------|-----------|
| kinematics | URDF キャッシュ処理を簡略化可能 |
| trajectory | 変更なし |
| iparam_identification | 変更なし |
| traj_follower | 変更なし |

### 4.4 成果物
- colcon_ws ビルド完了
- 全パッケージが Isaac Sim Python で動作

---

## Phase 5: Docker/devcontainer 更新

### 5.1 現行構成との比較

#### 現行ファイル

| ファイル | パス |
|---------|------|
| Dockerfile | `docker/Dockerfile` |
| docker-compose.yml | `docker/docker-compose.yml` |
| devcontainer.json | `.devcontainer/devcontainer.json` |

#### 変更サマリ

| 項目 | 現行 | アプローチB |
|------|------|------------|
| Dockerfile | apt で ROS インストール | Isaac Sim Python で ROS ソースビルド |
| Python リンク | `/isaac-sim/python.sh` → `/usr/local/bin/python` | `/isaac-sim/kit/python/bin/python3.11` → 直接使用 |
| shm_size | 1gb | 2gb（ビルド安定性向上） |
| volumes | 5 項目 | 6 項目（colcon build cache 追加） |
| environment | 3 項目 | 6 項目（Python/CMake 設定追加） |
| devcontainer extensions | 2 項目 | 6 項目（ROS, CMake, YAML 追加） |
| postCreateCommand | なし | colcon build 自動実行 |

---

### 5.2 Dockerfile (Single-stage)

**ファイル名**: `docker/Dockerfile.approach-b`

```dockerfile
# ============================================================
# Isaac Sim + ROS 2 Jazzy - Unified Python 3.11 Environment
# ============================================================
# アプローチB: Isaac Sim Python を直接使用して ROS 2 をビルド
# ============================================================

FROM nvcr.io/nvidia/isaac-sim:5.1.0

USER root

# ============================================================
# Isaac Sim Python をシステム Python として設定
# ============================================================
# Isaac Sim に同梱されている Python 3.11 を使用
# - バイナリ: /isaac-sim/kit/python/bin/python3.11
# - ヘッダー: /isaac-sim/kit/python/include/python3.11/
# - ライブラリ: /isaac-sim/kit/python/lib/libpython3.11.so
# ============================================================

ENV ISAAC_PYTHON=/isaac-sim/kit/python/bin/python3.11
ENV ISAAC_PYTHON_INCLUDE=/isaac-sim/kit/python/include/python3.11
ENV ISAAC_PYTHON_LIB=/isaac-sim/kit/python/lib

# Python コマンドを Isaac Sim Python にリンク
# これにより python, python3 コマンドが Isaac Sim Python を指す
RUN ln -sf ${ISAAC_PYTHON} /usr/local/bin/python3 \
    && ln -sf ${ISAAC_PYTHON} /usr/local/bin/python \
    && update-alternatives --install /usr/bin/python3 python3 ${ISAAC_PYTHON} 100

# CMake が Isaac Sim Python を検出するための環境変数
ENV Python3_EXECUTABLE=${ISAAC_PYTHON}
ENV PYTHON_EXECUTABLE=${ISAAC_PYTHON}
ENV PYTHON_INCLUDE_DIR=${ISAAC_PYTHON_INCLUDE}
ENV PYTHON_LIBRARY=${ISAAC_PYTHON_LIB}/libpython3.11.so

# ============================================================
# 基本ツールのインストール
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
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

# ============================================================
# ROS 2 ビルド依存関係（システムパッケージ）
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    # ROS 2 コアライブラリ依存
    libbullet-dev \
    libasio-dev \
    libtinyxml2-dev \
    libcunit1-dev \
    libacl1-dev \
    liblttng-ust-dev \
    libconsole-bridge-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# Python パッケージ (Isaac Sim Python に直接インストール)
# ============================================================
# empy 3.3.4 は ROS 2 Jazzy ビルドに必須
RUN ${ISAAC_PYTHON} -m pip install --no-cache-dir \
    empy==3.3.4 \
    catkin_pkg \
    lark \
    colcon-common-extensions \
    vcstool \
    "pybind11[global]" \
    netifaces

# ============================================================
# ROS 2 リポジトリ設定
# ============================================================
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    && apt-key add ros.asc && rm ros.asc

RUN echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# rosdep と rosinstall-generator (pip 版を使用)
RUN ${ISAAC_PYTHON} -m pip install --no-cache-dir rosdep rosinstall-generator \
    && rosdep init || true \
    && rosdep update --rosdistro jazzy

# ============================================================
# ROS 2 ソースビルド
# ============================================================
# 最小パッケージセット:
# - Core: rcutils, rcl, rmw, rclpy
# - Messages: std_msgs, sensor_msgs, geometry_msgs, nav_msgs
# - TF: tf2, tf2_ros, tf2_msgs
# - CLI: ros2topic, ros2node, ros2run, ros2pkg
# ============================================================
WORKDIR /opt/ros2_jazzy_ws

RUN rosinstall_generator --deps --rosdistro jazzy \
    rcutils rcl rmw rclpy \
    std_msgs sensor_msgs geometry_msgs nav_msgs \
    tf2 tf2_ros tf2_msgs \
    ros2topic ros2node ros2run ros2pkg \
    ros_environment common_interfaces rosgraph_msgs \
    > ros2.jazzy.rosinstall \
    && mkdir -p src && vcs import src < ros2.jazzy.rosinstall

# Isaac Sim 環境を source してビルド
# --install-base /opt/ros311 で Isaac Sim site-packages 外にインストール
RUN /bin/bash -c "source /isaac-sim/setup_python_env.sh && \
    colcon build --merge-install --install-base /opt/ros311 --cmake-args \
        -DPython3_EXECUTABLE=${ISAAC_PYTHON} \
        -DPYTHON_EXECUTABLE=${ISAAC_PYTHON} \
        -DPYTHON_INCLUDE_DIR=${ISAAC_PYTHON_INCLUDE} \
        -DPYTHON_LIBRARY=${ISAAC_PYTHON_LIB}/libpython3.11.so"

# 必要な共有ライブラリをコピー
RUN cp /usr/lib/x86_64-linux-gnu/libtinyxml2.so* /opt/ros311/lib/ 2>/dev/null || true \
    && cp /usr/lib/x86_64-linux-gnu/libconsole_bridge.so* /opt/ros311/lib/ 2>/dev/null || true

# ============================================================
# 追加 Python パッケージ（プロジェクト用）
# ============================================================
RUN ${ISAAC_PYTHON} -m pip install --no-cache-dir \
    pin==2.7.0 \
    pymlg \
    scipy \
    matplotlib

# ============================================================
# 環境設定
# ============================================================
ENV ROS_DISTRO=jazzy
ENV LD_LIBRARY_PATH=/opt/ros311/lib:${LD_LIBRARY_PATH}

# シェル設定
# source 順序: Isaac Sim → ROS 311 → colcon_ws
RUN echo '# Isaac Sim Python environment' >> /root/.bashrc \
    && echo 'source /isaac-sim/setup_python_env.sh' >> /root/.bashrc \
    && echo '' >> /root/.bashrc \
    && echo '# ROS 2 Jazzy (Python 3.11 build)' >> /root/.bashrc \
    && echo 'export ROS_DISTRO=jazzy' >> /root/.bashrc \
    && echo 'source /opt/ros311/setup.bash 2>/dev/null || true' >> /root/.bashrc \
    && echo '' >> /root/.bashrc \
    && echo '# Project workspace' >> /root/.bashrc \
    && echo 'if [ -f "/workspaces/isaac-sim-ur5e/colcon_ws/install/setup.bash" ]; then' >> /root/.bashrc \
    && echo '    source /workspaces/isaac-sim-ur5e/colcon_ws/install/setup.bash' >> /root/.bashrc \
    && echo 'fi' >> /root/.bashrc \
    && echo 'cd /workspaces/isaac-sim-ur5e' >> /root/.bashrc

WORKDIR /workspaces/isaac-sim-ur5e
USER root
```

---

### 5.3 docker-compose.yml

**ファイル名**: `docker/docker-compose.approach-b.yml`

```yaml
# ============================================================
# Docker Compose - Isaac Sim + ROS 2 Unified Python 3.11
# ============================================================
# アプローチB: Isaac Sim Python を直接使用した統一環境
# ============================================================

services:
  isaac-sim:
    build:
      context: ..
      dockerfile: docker/Dockerfile.approach-b
      # BuildKit キャッシュを有効化（ビルド高速化）
      args:
        - BUILDKIT_INLINE_CACHE=1

    image: isaac-sim-ur5e:unified-python311
    container_name: isaac-sim-ur5e

    # ホストネットワークを使用（ROS 2 通信用）
    network_mode: host

    # 共有メモリサイズ（ROS 2 ビルド安定性向上のため増加）
    shm_size: "2gb"

    environment:
      # Isaac Sim EULA 同意
      - ACCEPT_EULA=Y
      - PRIVACY_CONSENT=Y

      # X11 ディスプレイ
      - DISPLAY=${DISPLAY}

      # ROS 2 設定
      - ROS_DISTRO=jazzy
      - ROS_PYTHON_VERSION=3

      # Python 環境（CMake が検出できるように）
      - Python3_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11
      - PYTHON_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11

    volumes:
      # --------------------------------------------
      # X11 ソケット（GUI 表示用）
      # --------------------------------------------
      - /tmp/.X11-unix:/tmp/.X11-unix

      # --------------------------------------------
      # Isaac Sim キャッシュ（コンテナ再ビルド間で永続化）
      # --------------------------------------------
      - ${HOME}/docker/isaac-sim/cache/main:/root/.cache/isaac-sim
      - ${HOME}/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache
      - ${HOME}/docker/isaac-sim/data:/root/.local/share/ov/data

      # --------------------------------------------
      # 開発用設定（ホストから読み取り専用）
      # --------------------------------------------
      - ${HOME}/.ssh:/root/.ssh:ro
      - ${HOME}/.gitconfig:/root/.gitconfig:ro

      # --------------------------------------------
      # ワークスペース
      # --------------------------------------------
      - ..:/workspaces/isaac-sim-ur5e

      # --------------------------------------------
      # colcon ビルドキャッシュ（ビルド高速化）
      # Named volume で永続化
      # --------------------------------------------
      - colcon_build_cache:/workspaces/isaac-sim-ur5e/colcon_ws/build
      - colcon_install_cache:/workspaces/isaac-sim-ur5e/colcon_ws/install
      - colcon_log_cache:/workspaces/isaac-sim-ur5e/colcon_ws/log

    devices:
      # DRI デバイス（GPU レンダリング用）
      - /dev/dri:/dev/dri

    deploy:
      resources:
        reservations:
          devices:
            # NVIDIA GPU 全て使用
            - driver: nvidia
              count: all
              capabilities: [ gpu ]

    stdin_open: true
    tty: true
    command: sleep infinity

# ============================================================
# Named Volumes（永続化）
# ============================================================
volumes:
  # colcon ビルドキャッシュ
  # コンテナ再起動時もビルド成果物を保持
  colcon_build_cache:
    driver: local
  colcon_install_cache:
    driver: local
  colcon_log_cache:
    driver: local
```

#### docker-compose.yml 変更点詳細

| 項目 | 現行 | アプローチB | 理由 |
|------|------|------------|------|
| `shm_size` | 1gb | 2gb | ROS ソースビルド時の安定性向上 |
| `image` | なし | `isaac-sim-ur5e:unified-python311` | イメージタグ明示 |
| `environment` | 3項目 | 6項目 | Python/CMake 設定追加 |
| `volumes` | 5項目 | 8項目 | colcon キャッシュ追加 |
| `volumes (named)` | なし | 3項目 | ビルド成果物永続化 |

---

### 5.4 devcontainer.json

**ファイル名**: `.devcontainer/devcontainer.approach-b.json`

```json
{
    "name": "Isaac Sim UR5e (Unified Python 3.11)",

    "dockerComposeFile": "../docker/docker-compose.approach-b.yml",
    "service": "isaac-sim",
    "workspaceFolder": "/workspaces/isaac-sim-ur5e",

    "remoteUser": "root",
    "overrideCommand": true,

    "customizations": {
        "vscode": {
            "extensions": [
                // Python 開発
                "ms-python.python",
                "ms-python.vscode-pylance",
                "ms-python.debugpy",

                // ROS 開発
                "ms-iot.vscode-ros",

                // C++/CMake（ROS パッケージ用）
                "ms-vscode.cmake-tools",
                "ms-vscode.cpptools",

                // YAML/XML（ROS launch, package.xml）
                "redhat.vscode-yaml"
            ],
            "settings": {
                // ============================================
                // Python 設定
                // ============================================
                // Isaac Sim Python を使用
                "python.defaultInterpreterPath": "/isaac-sim/kit/python/bin/python3.11",

                // Pylance の解析パス
                "python.analysis.extraPaths": [
                    // ROS 2 Python 3.11 ビルド
                    "/opt/ros311/lib/python3.11/site-packages",
                    // プロジェクト colcon_ws
                    "/workspaces/isaac-sim-ur5e/colcon_ws/install/lib/python3.11/site-packages",
                    // Isaac Sim パッケージ
                    "/isaac-sim/exts/omni.isaac.core_archive/pip_prebundle",
                    "/isaac-sim/exts/omni.isaac.ml_archive/pip_prebundle",
                    "/isaac-sim/kit/python/lib/python3.11/site-packages"
                ],

                "python.analysis.typeCheckingMode": "basic",
                "python.languageServer": "Pylance",

                // 環境変数ファイル
                "python.envFile": "${workspaceFolder}/.env",

                // ============================================
                // ROS 設定
                // ============================================
                "ros.distro": "jazzy",

                // ============================================
                // ターミナル環境変数
                // ============================================
                "terminal.integrated.env.linux": {
                    "PYTHONPATH": "/opt/ros311/lib/python3.11/site-packages",
                    "ROS_DISTRO": "jazzy",
                    "Python3_EXECUTABLE": "/isaac-sim/kit/python/bin/python3.11"
                },

                // ============================================
                // エディタ設定
                // ============================================
                "editor.formatOnSave": true,
                "editor.rulers": [100],

                "[python]": {
                    "editor.defaultFormatter": "ms-python.python",
                    "editor.tabSize": 4
                },

                // ============================================
                // CMake 設定（ROS パッケージビルド用）
                // ============================================
                "cmake.configureArgs": [
                    "-DPython3_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11",
                    "-DPYTHON_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11"
                ]
            }
        }
    },

    // コンテナ作成後に実行（初回のみ）
    // ROS 環境を source して colcon ビルド
    "postCreateCommand": "/bin/bash -c 'source /opt/ros311/setup.bash && cd /workspaces/isaac-sim-ur5e/colcon_ws && colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11'",

    // コンテナ起動時に実行（毎回）
    "postStartCommand": "/bin/bash -c 'source /isaac-sim/setup_python_env.sh && source /opt/ros311/setup.bash'",

    // リモート環境変数
    "remoteEnv": {
        "ROS_DISTRO": "jazzy",
        "ROS_PYTHON_VERSION": "3",
        "PYTHONPATH": "/opt/ros311/lib/python3.11/site-packages",
        "LD_LIBRARY_PATH": "/opt/ros311/lib",
        "Python3_EXECUTABLE": "/isaac-sim/kit/python/bin/python3.11",
        "PYTHON_EXECUTABLE": "/isaac-sim/kit/python/bin/python3.11"
    },

    // ポートフォワーディング（必要に応じて追加）
    "forwardPorts": [],

    // 機能（追加機能があれば）
    "features": {}
}
```

#### devcontainer.json 変更点詳細

| 項目 | 現行 | アプローチB | 理由 |
|------|------|------------|------|
| `name` | "Isaac Sim UR5e RL" | "Isaac Sim UR5e (Unified Python 3.11)" | 環境を明示 |
| `extensions` | 2項目 | 7項目 | ROS, CMake, デバッグ追加 |
| `python.defaultInterpreterPath` | なし | Isaac Sim Python | 明示的に指定 |
| `python.analysis.extraPaths` | なし | 5パス | Pylance 解析用 |
| `terminal.integrated.env.linux` | なし | 3項目 | ターミナル環境設定 |
| `cmake.configureArgs` | なし | 2項目 | CMake Python 設定 |
| `postCreateCommand` | なし | colcon build | 初回自動ビルド |
| `postStartCommand` | なし | source 環境 | 毎回環境設定 |
| `remoteEnv` | なし | 6項目 | 環境変数設定 |

---

### 5.5 .env ファイル（オプション）

**ファイル名**: `.env.approach-b`

```bash
# ============================================================
# Python 3.11 Unified Environment Variables
# ============================================================
# このファイルを .env にコピーして使用

# ROS 2 設定
ROS_DISTRO=jazzy
ROS_PYTHON_VERSION=3

# Python パス
PYTHONPATH=/opt/ros311/lib/python3.11/site-packages

# CMake 設定
Python3_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11
PYTHON_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11
PYTHON_INCLUDE_DIR=/isaac-sim/kit/python/include/python3.11
PYTHON_LIBRARY=/isaac-sim/kit/python/lib/libpython3.11.so

# ライブラリパス
LD_LIBRARY_PATH=/opt/ros311/lib
```

---

### 5.6 成果物一覧

| ファイル | パス | 説明 |
|---------|------|------|
| Dockerfile | `docker/Dockerfile.approach-b` | Single-stage ビルド |
| docker-compose.yml | `docker/docker-compose.approach-b.yml` | 拡張構成 |
| devcontainer.json | `.devcontainer/devcontainer.approach-b.json` | VSCode 設定 |
| .env | `.env.approach-b` | 環境変数テンプレート |

### 5.7 移行手順

実装時の手順:

```bash
# 1. 現行ファイルをバックアップ
cp docker/Dockerfile docker/Dockerfile.backup
cp docker/docker-compose.yml docker/docker-compose.yml.backup
cp .devcontainer/devcontainer.json .devcontainer/devcontainer.json.backup

# 2. アプローチB ファイルを本番ファイルにコピー
cp docker/Dockerfile.approach-b docker/Dockerfile
cp docker/docker-compose.approach-b.yml docker/docker-compose.yml
cp .devcontainer/devcontainer.approach-b.json .devcontainer/devcontainer.json
cp .env.approach-b .env

# 3. Docker イメージ再ビルド
cd docker
docker-compose build --no-cache

# 4. コンテナ起動
docker-compose up -d

# 5. VSCode で Reopen in Container
```

---

## Phase 6: 検証・テスト

### 6.1 基本検証

```bash
# Python バージョン確認
python --version
# 期待: Python 3.11.x

# which python
which python
# 期待: /usr/local/bin/python -> /isaac-sim/kit/python/bin/python3.11

# ROS 2 CLI
ros2 topic list
ros2 node list

# Python import テスト
python -c "import rclpy; print('rclpy OK')"
python -c "import std_msgs; print('std_msgs OK')"
python -c "import pinocchio; print('pinocchio OK')"

# Isaac Sim import テスト
python -c "from isaacsim import SimulationApp; print('Isaac Sim OK')"
```

### 6.2 統合テスト

```bash
# kinematics パッケージ
ros2 run kinematics kinematics_node

# iparam_identification テスト
cd /workspaces/isaac-sim-ur5e/colcon_ws
python src/iparam_identification/scripts/run_identification_test.py
```

### 6.3 テストチェックリスト

| # | テスト項目 | 期待結果 | 状態 |
|---|-----------|---------|------|
| 1 | `python --version` | Python 3.11.x | [ ] |
| 2 | `import rclpy` | エラーなし | [ ] |
| 3 | `import pinocchio` | エラーなし | [ ] |
| 4 | `from isaacsim import SimulationApp` | エラーなし | [ ] |
| 5 | `ros2 topic list` | 正常動作 | [ ] |
| 6 | kinematics ノード起動 | 正常動作 | [ ] |
| 7 | iparam_identification テスト | 0.01% 以下の誤差 | [ ] |

---

## タイムライン

| フェーズ | 作業内容 | 依存関係 |
|---------|---------|---------|
| Phase 1 | 環境変数・パス設定 | なし |
| Phase 2 | 依存関係インストール | Phase 1 |
| Phase 3 | ROS 2 ソースビルド | Phase 2 |
| Phase 4 | colcon_ws ビルド | Phase 3 |
| Phase 5 | Docker/devcontainer 更新 | Phase 1-4 の知見 |
| Phase 6 | 検証・テスト | Phase 5 |

---

## リスクと対策

| リスク | 影響度 | 対策 |
|--------|-------|------|
| Isaac Sim パッケージとの競合 | 高 | ROS を別ディレクトリ (/opt/ros311) にインストール |
| ビルドエラー | 中 | 最小パッケージセットから開始、段階的に追加 |
| Isaac Sim アップデートで破損 | 中 | Dockerfile で再現可能な構成を維持 |
| LD_LIBRARY_PATH 競合 | 中 | /opt/ros311/lib を先頭に配置 |

---

## ロールバック計画

問題発生時:
1. 現行の Dockerfile (apt ROS) に戻す
2. URDF キャッシュ + PYTHONPATH クリーニングで対応（現在の workaround）

---

## 参考資料

- Isaac Sim Python 構成: `/isaac-sim/setup_python_env.sh`
- Python 開発ファイル: `/isaac-sim/kit/python/`
- 現行の workaround: `colcon_ws/src/kinematics/src/kinematics.py`
