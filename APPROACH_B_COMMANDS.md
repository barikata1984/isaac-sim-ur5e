# アプローチB - ホストから実行するコマンド集

## 基本動作確認

### Python バージョン確認
```bash
docker exec isaac-sim-ur5e-approach-b python --version
# 出力: Python 3.11.13
```

### ROS 2 topic list
```bash
docker exec isaac-sim-ur5e-approach-b bash -c "source /opt/ros/jazzy/setup.bash && source /opt/ros311/setup.bash && ros2 topic list"
```

### rclpy import 確認
```bash
docker exec isaac-sim-ur5e-approach-b bash -c "source /opt/ros311/setup.bash && python -c 'import rclpy; print(\"rclpy OK\")'"
```

## colcon ワークスペース

### colcon ビルド
```bash
docker exec isaac-sim-ur5e-approach-b bash -c "cd /workspaces/isaac-sim-ur5e/colcon_ws && source /opt/ros311/setup.bash && colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11"
```

### colcon ビルド（クリーン）
```bash
docker exec isaac-sim-ur5e-approach-b bash -c "cd /workspaces/isaac-sim-ur5e/colcon_ws && rm -rf build install log && source /opt/ros311/setup.bash && colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11"
```

## bring_up.launch.py 実行

### launch ファイルの引数確認
```bash
docker exec isaac-sim-ur5e-approach-b bash -c "source /opt/ros/jazzy/setup.bash && source /opt/ros311/setup.bash && source /workspaces/isaac-sim-ur5e/colcon_ws/install/setup.bash && ros2 launch core bring_up.launch.py --show-args"
```

### headless モードで起動（デフォルト: ur5e）
```bash
docker exec isaac-sim-ur5e-approach-b bash -c "source /opt/ros/jazzy/setup.bash && source /opt/ros311/setup.bash && source /workspaces/isaac-sim-ur5e/colcon_ws/install/setup.bash && ros2 launch core bring_up.launch.py headless:=true"
```

### 異なるロボットタイプで起動
```bash
docker exec isaac-sim-ur5e-approach-b bash -c "source /opt/ros/jazzy/setup.bash && source /opt/ros311/setup.bash && source /workspaces/isaac-sim-ur5e/colcon_ws/install/setup.bash && ros2 launch core bring_up.launch.py robot_type:=ur10 headless:=true"
```

### バックグラウンドで起動
```bash
docker exec -d isaac-sim-ur5e-approach-b bash -c "source /opt/ros/jazzy/setup.bash && source /opt/ros311/setup.bash && source /workspaces/isaac-sim-ur5e/colcon_ws/install/setup.bash && ros2 launch core bring_up.launch.py headless:=true"
```

### ログ確認
```bash
docker logs -f isaac-sim-ur5e-approach-b
```

## コンテナ管理

### コンテナ起動
```bash
cd /home/atsushi/workspace/isaac-sim-ur5e/docker
docker compose -f docker-compose.approach-b.yml up -d
```

### コンテナ停止
```bash
docker compose -f docker-compose.approach-b.yml stop
```

### コンテナ再起動
```bash
docker compose -f docker-compose.approach-b.yml restart
```

### コンテナに入る（インタラクティブシェル）
```bash
docker exec -it isaac-sim-ur5e-approach-b bash
```

## トラブルシューティング

### Python パッケージの確認
```bash
docker exec isaac-sim-ur5e-approach-b python -c "import sys; print('\\n'.join(sys.path))"
```

### ROS 環境変数の確認
```bash
docker exec isaac-sim-ur5e-approach-b bash -c "source /opt/ros311/setup.bash && env | grep ROS"
```

### インストール済みパッケージの確認
```bash
docker exec isaac-sim-ur5e-approach-b python -m pip list | grep -E "rclpy|psutil|pin"
```

## 環境変数設定用のエイリアス（コンテナ内で使用）

コンテナ内で `.bashrc` または手動で以下を実行：

```bash
alias ros_env='source /opt/ros/jazzy/setup.bash && source /opt/ros311/setup.bash'
alias ws_env='source /opt/ros/jazzy/setup.bash && source /opt/ros311/setup.bash && source /workspaces/isaac-sim-ur5e/colcon_ws/install/setup.bash'
```

使用例：
```bash
docker exec isaac-sim-ur5e-approach-b bash -c "ws_env && ros2 launch core bring_up.launch.py headless:=true"
```

## 注意事項

1. **環境の source 順序が重要**:
   - Isaac Sim Python env (`/isaac-sim/setup_python_env.sh`)
   - ROS Jazzy apt版 (`/opt/ros/jazzy/setup.bash`) ← launch 用
   - ROS 311 ビルド版 (`/opt/ros311/setup.bash`) ← Python 3.11 統一
   - colcon_ws (`/workspaces/isaac-sim-ur5e/colcon_ws/install/setup.bash`)

2. **Python 3.11 の確認**:
   - すべての Python 操作で `/isaac-sim/kit/python/bin/python3.11` が使われていることを確認

3. **headless モードの推奨**:
   - コンテナ内で GUI を使わない場合は `headless:=true` を指定

4. **ログの確認**:
   - Isaac Sim のログ: `/isaac-sim/kit/logs/Kit/Isaac-Sim Python/5.1/`
   - ROS ログ: `/root/.ros/log/`
