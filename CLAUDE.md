# Isaac Sim UR5e プロジェクト - Claude Code 設定

## 言語設定

このプロジェクトでは**日本語**でやり取りを行います。コード内のコメントやdocstringは英語で記述してください。

## プロジェクト概要

NVIDIA Isaac Sim + ROS 2 Jazzy を使用した UR5e ロボットアームシミュレーションプロジェクトです。
詳細なプロジェクト構造とワークフローは `.agent/AGENTS.md` を参照してください。

### 現在の開発目標

1. **軌道生成の拡張** - 新しい軌道生成アルゴリズムの追加
2. **シミュレーション機能強化** - Isaac Simとの連携強化、センサー追加、可視化改善
3. **MoveIt2/モーションプランニング** - MoveIt2統合によるモーションプランニング機能

## 開発環境

### 実行環境

- **Docker** 内で実行（docker-compose使用）
- GPU対応のNVIDIA Containerランタイム

### ビルドコマンド

```bash
# コンテナ起動
cd docker && docker-compose up -d

# コンテナ内でビルド
cd /workspaces/isaac-sim-ur5e/colcon_ws
colcon build
source install/setup.bash

# 特定パッケージのみビルド
colcon build --packages-select trajectory
colcon build --symlink-install  # 開発時推奨
```

### 実行コマンド

```bash
# Isaac Sim起動
ros2 run core isaac_sim_node

# 軌道生成
ros2 run trajectory trajectory_generator --help

# 軌道再生
ros2 launch traj_follower play_trajectory.launch.py json_path:=<path>
```

## コーディング規約

### Python スタイル

- **型ヒント必須**: すべての関数の引数と戻り値に型アノテーションを付ける
- **docstring必須**: 公開関数・クラスにはGoogle styleのdocstringを記述
- **PEP 8準拠**: flake8でチェック

```python
def calculate_trajectory(
    start_pos: np.ndarray,
    end_pos: np.ndarray,
    duration: float,
) -> np.ndarray:
    """Calculate a smooth trajectory between two positions.

    Args:
        start_pos: Starting joint positions (6 DOF).
        end_pos: Target joint positions (6 DOF).
        duration: Total duration in seconds.

    Returns:
        Array of joint positions over time.
    """
    pass
```

### コミット規約

**Conventional Commits** 形式を使用:

```
feat: 新機能の追加
fix: バグ修正
docs: ドキュメントのみの変更
refactor: リファクタリング
test: テストの追加・修正
chore: ビルドプロセスやツールの変更
```

## テスト方針

### 静的解析

```bash
# lint
flake8 colcon_ws/src/

# 型チェック（推奨）
mypy colcon_ws/src/trajectory/
```

### ユニットテスト

新機能追加時は `test/` ディレクトリにpytestテストを追加してください。

```bash
# テスト実行
cd colcon_ws
colcon test --packages-select trajectory
colcon test-result --verbose
```

## 開発フロー

ブランチ運用を行います。詳細は `.agent/AGENTS.md` の「Coding Standards」セクションを参照。

1. feature branchを作成: `feature/ISSUE_ID-description`
2. 実装・テスト
3. PR作成・レビュー
4. mainにマージ

### ワークフローコマンド

`.agent/workflows/` に定義されたワークフローが使用可能です:

- `/start-task [説明]` - 新しいタスクを開始
- `/commit` - コミット操作
- `/finish-task` - タスク完了

## ROS 2 パッケージ構成

| パッケージ | 説明 |
|-----------|------|
| `core` | Isaac Sim起動・ロボットスポーン |
| `teleop` | テレオペレーション制御 |
| `trajectory` | 軌道生成CLI |
| `traj_follower` | 軌道再生ノード |

## 注意事項

- Isaac Simは大量のGPUメモリを使用します
- ROS 2ノード間の通信はトピック `joint_commands` (Float64MultiArray) を使用
- 軌道データはJSON形式で保存（`data/` ディレクトリ）
