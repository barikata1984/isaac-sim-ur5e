# アプローチB 移行ガイド

## 概要

Isaac Sim 同梱の Python 3.11 を直接使用して ROS 2 Jazzy をビルドする統一環境への移行手順です。

## 作成済みファイル

以下のファイルがすでに作成されています：

| ファイル | 説明 |
|---------|------|
| `docker/Dockerfile.approach-b` | Isaac Sim Python で ROS 2 をビルドする Dockerfile |
| `docker/docker-compose.approach-b.yml` | 拡張された docker-compose 設定 |
| `.devcontainer/devcontainer.approach-b.json` | VSCode devcontainer 設定 |
| `.env.approach-b` | 環境変数テンプレート |

## 移行手順

### ステップ 1: 現在の設定をバックアップ

```bash
# プロジェクトルートで実行
cd /home/atsushi/workspace/isaac-sim-ur5e

# 現在のファイルをバックアップ
cp docker/Dockerfile docker/Dockerfile.backup
cp docker/docker-compose.yml docker/docker-compose.yml.backup
cp .devcontainer/devcontainer.json .devcontainer/devcontainer.json.backup
```

### ステップ 2: アプローチB ファイルを本番環境にコピー

```bash
# アプローチB ファイルを本番ファイルに上書き
cp docker/Dockerfile.approach-b docker/Dockerfile
cp docker/docker-compose.approach-b.yml docker/docker-compose.yml
cp .devcontainer/devcontainer.approach-b.json .devcontainer/devcontainer.json
cp .env.approach-b .env
```

### ステップ 3: 既存コンテナとイメージをクリーンアップ

```bash
# 既存コンテナを停止・削除
cd docker
docker-compose down

# 既存イメージを削除（オプション、クリーンビルドのため推奨）
docker images | grep isaac-sim-ur5e
docker rmi <IMAGE_ID>  # 表示されたイメージIDを指定

# または、すべてクリーンアップ
docker system prune -a  # 注意：すべての未使用イメージが削除されます
```

### ステップ 4: Docker イメージをビルド

```bash
cd docker

# ビルド実行（初回は30分〜1時間程度かかります）
docker-compose build --no-cache

# ビルドログを確認
# エラーが発生した場合は、ログを確認して対処してください
```

### ステップ 5: コンテナを起動

```bash
# コンテナ起動
docker-compose up -d

# ログを確認
docker-compose logs -f
```

### ステップ 6: VSCode で Reopen in Container

1. VSCode でプロジェクトを開く
2. `F1` キーを押して Command Palette を開く
3. `Dev Containers: Reopen in Container` を選択
4. コンテナが起動し、`postCreateCommand` で colcon ビルドが自動実行されます

### ステップ 7: 環境検証

コンテナ内で以下のコマンドを実行して環境を確認：

```bash
# Python バージョン確認
python --version
# 期待: Python 3.11.x

# which python
which python
# 期待: /usr/local/bin/python -> /isaac-sim/kit/python/bin/python3.11

# ROS 2 CLI 動作確認
ros2 topic list
ros2 node list

# Python import テスト
python -c "import rclpy; print('rclpy OK')"
python -c "import std_msgs; print('std_msgs OK')"

# Isaac Sim import テスト
python -c "from isaacsim import SimulationApp; print('Isaac Sim OK')"
```

### ステップ 8: プロジェクトパッケージのビルド

```bash
# colcon_ws に移動
cd /workspaces/isaac-sim-ur5e/colcon_ws

# ROS 環境を source
source /opt/ros311/setup.bash

# ビルド実行
colcon build --symlink-install

# ビルド結果を source
source install/setup.bash
```

### ステップ 9: 統合テスト

```bash
# kinematics パッケージテスト
ros2 run kinematics kinematics_node

# trajectory パッケージテスト
ros2 run trajectory trajectory_generator --help

# iparam_identification テスト（該当する場合）
cd /workspaces/isaac-sim-ur5e/colcon_ws
python src/iparam_identification/scripts/run_identification_test.py
```

## トラブルシューティング

### ビルドエラーが発生した場合

#### エラー: `empy` バージョン不一致

```bash
# コンテナ内で確認
python -c "import em; print(em.__version__)"
# 期待: 3.3.4

# 修正（必要な場合）
python -m pip uninstall -y em empy
python -m pip install empy==3.3.4
```

#### エラー: Python ヘッダーが見つからない

```bash
# シンボリックリンクを確認
ls -l /usr/include/python3
# 期待: /usr/include/python3 -> /isaac-sim/kit/python/include/python3.11

# 修正（必要な場合）
ln -sf /isaac-sim/kit/python/include/python3.11 /usr/include/python3
```

#### エラー: `libtinyxml2.so` が見つからない

```bash
# ライブラリをコピー
cp /usr/lib/x86_64-linux-gnu/libtinyxml2.so* /opt/ros311/lib/
cp /usr/lib/x86_64-linux-gnu/libconsole_bridge.so* /opt/ros311/lib/

# LD_LIBRARY_PATH を確認
echo $LD_LIBRARY_PATH
# /opt/ros311/lib が含まれていることを確認
```

### `import rclpy` エラー

```bash
# PYTHONPATH を確認
echo $PYTHONPATH
# /opt/ros311/lib/python3.11/site-packages が含まれていることを確認

# ROS 環境を再 source
source /isaac-sim/setup_python_env.sh
source /opt/ros311/setup.bash
```

### colcon ビルドエラー

```bash
# クリーンビルド
cd /workspaces/isaac-sim-ur5e/colcon_ws
rm -rf build install log
colcon build --symlink-install --cmake-args \
    -DPython3_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11
```

## ロールバック手順

問題が発生して元に戻す必要がある場合：

```bash
# バックアップから復元
cp docker/Dockerfile.backup docker/Dockerfile
cp docker/docker-compose.yml.backup docker/docker-compose.yml
cp .devcontainer/devcontainer.json.backup .devcontainer/devcontainer.json

# コンテナを再ビルド
cd docker
docker-compose down
docker-compose build --no-cache
docker-compose up -d

# VSCode で Reopen in Container
```

## 検証チェックリスト

| # | テスト項目 | 期待結果 | 状態 |
|---|-----------|---------|------|
| 1 | `python --version` | Python 3.11.x | [ ] |
| 2 | `import rclpy` | エラーなし | [ ] |
| 3 | `import std_msgs` | エラーなし | [ ] |
| 4 | `from isaacsim import SimulationApp` | エラーなし | [ ] |
| 5 | `ros2 topic list` | 正常動作 | [ ] |
| 6 | kinematics ノード起動 | 正常動作 | [ ] |
| 7 | trajectory パッケージ | 正常動作 | [ ] |
| 8 | colcon ビルド | エラーなし | [ ] |

## 変更点まとめ

### Dockerfile の主な変更

| 項目 | 現行 | アプローチB |
|------|------|------------|
| Python | apt版 ROS 2 (Python 3.12) | Isaac Sim Python 3.11 |
| ROS 2 | apt パッケージ | ソースビルド (`/opt/ros311`) |
| ビルド時間 | 約5分 | 約30〜60分（初回のみ） |

### docker-compose.yml の主な変更

| 項目 | 現行 | アプローチB |
|------|------|------------|
| `shm_size` | 1gb | 2gb |
| `volumes` | 5項目 | 8項目（colcon キャッシュ追加） |
| `environment` | 3項目 | 6項目（Python/CMake 設定追加） |

### devcontainer.json の主な変更

| 項目 | 現行 | アプローチB |
|------|------|------------|
| `extensions` | 2項目 | 7項目（ROS, CMake, YAML 追加） |
| `python.defaultInterpreterPath` | なし | Isaac Sim Python 明示 |
| `postCreateCommand` | なし | colcon ビルド自動実行 |
| `remoteEnv` | なし | 6項目の環境変数設定 |

## 期待される効果

- ✅ Python バージョン競合の完全解決
- ✅ xacro 実行時のエラー解消
- ✅ Pinocchio 等のライブラリバージョン統一
- ✅ PYTHONPATH の複雑な管理が不要に
- ✅ kinematics.py の PYTHONPATH クリーニング処理を簡略化可能

## 参考資料

- [python311_approach_b_action_plan.md](docs/design/python311_approach_b_action_plan.md) - 詳細設計
- [python311_unification_design.md](docs/design/python311_unification_design.md) - アプローチA（参考）
- Isaac Sim Python 構成: `/isaac-sim/setup_python_env.sh`
- Python 開発ファイル: `/isaac-sim/kit/python/`
