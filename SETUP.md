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

### 2. コンテナのビルドと起動

**重要**: underlay_ws の準備は自動的に行われます。Dockerイメージビルド時に ur_description が含まれ、初回コンテナ起動時に自動的にセットアップされます。

#### 方法A: VSCode Dev Containers（推奨）

1. VSCode でプロジェクトを開く
2. コマンドパレット（Ctrl+Shift+P）から "Dev Containers: Reopen in Container" を実行
3. 初回起動時、entrypoint.sh により自動的に underlay_ws と colcon_ws がビルドされます（3-5分程度）

#### 方法B: Docker Compose

```bash
cd docker
docker-compose -f docker-compose.approach-b.yml build  # イメージビルド（初回のみ）
docker-compose -f docker-compose.approach-b.yml up -d   # コンテナ起動
docker exec -it isaac-sim-ur5e-approach-b /bin/bash
```

初回起動時、entrypoint.sh により自動的にワークスペースがビルドされます。ログを確認するには:

```bash
docker logs isaac-sim-ur5e-approach-b
```

### 3. ビルドの確認

コンテナ内で以下を実行して、ワークスペースがビルドされていることを確認します:

```bash
source /opt/ros311/setup.bash
source /workspaces/isaac-sim-ur5e/underlay_ws/install/setup.bash
source /workspaces/isaac-sim-ur5e/colcon_ws/install/setup.bash

# パッケージの確認
ros2 pkg list | grep ur_description
ros2 pkg list | grep core
```

## ワークスペース構成と自動ビルドフロー

### ディレクトリ構成

```
isaac-sim-ur5e/
├── underlay_ws/          # 依存パッケージ（ur_description等）
│   ├── src/             # ホストからマウント or イメージからコピー
│   ├── build/           # Docker volume でキャッシュ
│   └── install/         # Docker volume でキャッシュ
│
├── colcon_ws/           # プロジェクトパッケージ
│   ├── src/             # ホストからマウント
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
    ├── entrypoint.sh              # コンテナ起動時処理
    └── Dockerfile.approach-b      # イメージビルド定義
```

### 自動ビルドフロー

#### 1. イメージビルド時 (Dockerfile)

```
Dockerfile → /opt/underlay_ws_template/ を作成
  ├── ur_description をクローン
  ├── ビルド実行
  └── URDF キャッシュ生成
```

#### 2. 初回コンテナ起動時 (entrypoint.sh)

```
entrypoint.sh
  ├── ビルドマーカー確認
  ├── underlay_ws/src が空なら
  │   └── /opt/underlay_ws_template/src をコピー
  └── init_workspaces.sh を実行
      ├── Phase 1: underlay_ws ビルド
      ├── Phase 2: URDF キャッシュ生成
      ├── Phase 3: colcon_ws ビルド
      └── ビルドマーカー作成
```

#### 3. 2回目以降の起動

```
entrypoint.sh
  └── ビルドマーカー存在 → スキップ（高速起動）
```

この設計により:
- **devcontainer なしでも動作**: docker-compose だけで完全に動作
- **ビルドキャッシュ**: Docker volume により高速な再ビルド
- **冪等性**: 何度起動しても安全

## トラブルシューティング

### underlay_ws がビルドされない

手動で再ビルドを実行:

```bash
/bin/bash /workspaces/isaac-sim-ur5e/docker/scripts/rebuild_underlay.sh
```

### ur_description が見つからない

通常は自動的にセットアップされますが、問題がある場合:

1. ビルドマーカーを削除して再初期化:
   ```bash
   rm -f /workspaces/isaac-sim-ur5e/.build_completed
   # コンテナを再起動
   ```

2. 手動で再ビルド:
   ```bash
   /bin/bash /workspaces/isaac-sim-ur5e/docker/scripts/rebuild_underlay.sh
   ```

3. テンプレートから手動でコピー（イメージにテンプレートがある場合）:
   ```bash
   cp -r /opt/underlay_ws_template/src/* /workspaces/isaac-sim-ur5e/underlay_ws/src/
   cd /workspaces/isaac-sim-ur5e/underlay_ws
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
