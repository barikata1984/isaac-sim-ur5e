# アプローチB クイックスタート

## 既存環境を残したまま、アプローチBをテストする最短手順

### 前提条件
- 既存のコンテナ `isaac-sim-ur5e` はそのまま残します
- 新しいコンテナ `isaac-sim-ur5e-approach-b` を並行して作成します

---

## 手順

### 1. イメージビルド（30〜60分）

```bash
cd /home/atsushi/workspace/isaac-sim-ur5e/docker
docker-compose -f docker-compose.approach-b.yml build --no-cache
```

**コーヒーブレイク ☕** - ビルドが完了するまで待ちます。

### 2. コンテナ起動

```bash
docker-compose -f docker-compose.approach-b.yml up -d
```

### 3. 動作確認

```bash
docker exec -it isaac-sim-ur5e-approach-b /bin/bash

# Python バージョン確認
python --version  # Python 3.11.x が表示されることを確認

# ROS 環境を source
source /isaac-sim/setup_python_env.sh
source /opt/ros311/setup.bash

# ROS 2 動作確認
ros2 topic list

# Python import テスト
python -c "import rclpy; print('rclpy OK')"

# 確認できたら exit
exit
```

### 4. VSCode で接続

#### 方法A: Attach to Running Container（推奨）

1. VSCode で `F1` キーを押す
2. `Dev Containers: Attach to Running Container...` を選択
3. `isaac-sim-ur5e-approach-b` を選択
4. 新しいウィンドウが開く
5. `File` → `Open Folder` → `/workspaces/isaac-sim-ur5e` を開く

#### 方法B: devcontainer.json を切り替える

```bash
# バックアップ
cp .devcontainer/devcontainer.json .devcontainer/devcontainer.json.backup

# アプローチB に切り替え
cp .devcontainer/devcontainer.approach-b.json .devcontainer/devcontainer.json

# VSCode で F1 → Dev Containers: Reopen in Container
```

### 5. colcon ビルド

VSCode のターミナルで：

```bash
cd /workspaces/isaac-sim-ur5e/colcon_ws
source /opt/ros311/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 6. テスト実行

```bash
# パッケージが正常動作するか確認
ros2 run kinematics kinematics_node --help
ros2 run trajectory trajectory_generator --help
```

---

## 既存環境に戻る

```bash
# アプローチB コンテナを停止
docker-compose -f docker-compose.approach-b.yml stop

# 既存コンテナを起動（停止していた場合）
docker-compose -f docker-compose.yml up -d

# VSCode で既存コンテナにアタッチ
# F1 → Dev Containers: Attach to Running Container → isaac-sim-ur5e
```

---

## トラブル時

詳細は [APPROACH_B_SAFE_SETUP.md](APPROACH_B_SAFE_SETUP.md) を参照してください。

---

## コンテナの使い分け

| コンテナ名 | 用途 |
|-----------|------|
| `isaac-sim-ur5e` | 既存環境（保持） |
| `isaac-sim-ur5e-approach-b` | 新しい統一環境（テスト） |

両方を同時に起動可能ですが、ポート競合に注意してください。
