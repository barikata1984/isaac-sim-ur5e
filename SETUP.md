# セットアップガイド

このドキュメントでは、Isaac Sim UR5e プロジェクトのセットアップ手順を説明します。

## 前提条件

- Docker と Docker Compose がインストールされていること
- NVIDIA GPU と nvidia-docker がインストールされていること
- VSCode と Dev Containers 拡張機能（推奨）

## セットアップ手順

### 1. リポジトリのクローン

```bash
git clone <repository-url>
cd isaac-sim-ur5e
```

### 2. Underlay Workspace の準備

underlay_ws には、ROS 2 の依存パッケージを配置します。

#### 2-1. ur_description の追加（必須）

```bash
cd underlay_ws/src
git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git ur_description
cd ../..
```

#### 2-2. その他のパッケージ（オプション）

必要に応じて、他のROS 2パッケージも `underlay_ws/src` に追加できます。

### 3. コンテナのビルドと起動

#### 方法A: VSCode Dev Containers（推奨）

1. VSCode でプロジェクトを開く
2. コマンドパレット（Ctrl+Shift+P）から "Dev Containers: Reopen in Container" を実行
3. 初回起動時、自動的に underlay_ws と colcon_ws がビルドされます（5-10分程度）

#### 方法B: Docker Compose

```bash
cd docker
docker-compose -f docker-compose.approach-b.yml up -d
docker exec -it isaac-sim-ur5e-approach-b /bin/bash
```

初回起動時は、コンテナ内で以下を実行してワークスペースをビルドします:

```bash
/bin/bash /workspaces/isaac-sim-ur5e/docker/scripts/init_workspaces.sh
```

### 4. ビルドの確認

コンテナ内で以下を実行して、ワークスペースがビルドされていることを確認します:

```bash
source /opt/ros311/setup.bash
source /workspaces/isaac-sim-ur5e/underlay_ws/install/setup.bash
source /workspaces/isaac-sim-ur5e/colcon_ws/install/setup.bash

# パッケージの確認
ros2 pkg list | grep ur_description
ros2 pkg list | grep core
```

## ワークスペース構成

```
isaac-sim-ur5e/
├── underlay_ws/          # 依存パッケージ（ur_description等）
│   ├── src/
│   ├── build/           # Docker volume でキャッシュ
│   └── install/         # Docker volume でキャッシュ
│
├── colcon_ws/           # プロジェクトパッケージ
│   ├── src/
│   │   ├── core/
│   │   ├── ur/
│   │   ├── trajectories/
│   │   ├── kinematics/
│   │   ├── iparam_identification/
│   │   └── ...
│   ├── build/           # Docker volume でキャッシュ
│   └── install/         # Docker volume でキャッシュ
│
└── docker/
    ├── scripts/
    │   ├── init_workspaces.sh     # 初回ビルドスクリプト
    │   └── rebuild_underlay.sh    # 再ビルドスクリプト
    └── ...
```

## トラブルシューティング

### underlay_ws がビルドされない

手動で再ビルドを実行:

```bash
/bin/bash /workspaces/isaac-sim-ur5e/docker/scripts/rebuild_underlay.sh
```

### ur_description が見つからない

1. underlay_ws/src に ur_description が存在するか確認:
   ```bash
   ls -la /workspaces/isaac-sim-ur5e/underlay_ws/src/
   ```

2. 存在しない場合は追加してビルド:
   ```bash
   cd /workspaces/isaac-sim-ur5e/underlay_ws/src
   git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git ur_description
   cd ..
   source /opt/ros311/setup.bash
   colcon build --symlink-install
   ```

### ビルドキャッシュをクリアしたい

Docker volume を削除してコンテナを再作成:

```bash
cd docker
docker-compose -f docker-compose.approach-b.yml down -v
docker-compose -f docker-compose.approach-b.yml up -d
```

## 開発フロー

### パッケージのビルド

```bash
cd /workspaces/isaac-sim-ur5e/colcon_ws
source /opt/ros311/setup.bash
source /workspaces/isaac-sim-ur5e/underlay_ws/install/setup.bash
colcon build --symlink-install
```

特定パッケージのみビルド:

```bash
colcon build --packages-select trajectory
```

### テストの実行

```bash
# ユニットテスト
cd /workspaces/isaac-sim-ur5e/colcon_ws
colcon test --packages-select iparam_identification
colcon test-result --verbose

# Isaac Sim 統合テスト
bash src/iparam_identification/scripts/run_test.sh
```

## 環境変数

コンテナ内では以下の環境変数が設定されています:

- `ISAAC_PYTHON`: `/isaac-sim/kit/python/bin/python3.11`
- `ROS_DISTRO`: `jazzy`
- `Python3_EXECUTABLE`: `/isaac-sim/kit/python/bin/python3.11`

## 参考

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Universal Robots ROS 2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
