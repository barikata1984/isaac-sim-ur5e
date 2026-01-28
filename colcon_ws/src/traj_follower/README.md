# Trajectory Follower

Isaac Sim上のUR5eロボットに対して、JSON形式の軌道データに従って関節位置指令を送信するROS 2パッケージです。

## 前提

*   Isaac Sim上でUR5eがSpawnされ、ROS 2通信が確立されていること（`core` パッケージの `isaac_sim_gui.launch.py` 等を実行中であること）。
*   `trajectory` パッケージの `trajectory_generator` で生成されたJSON形式の軌道データがあること。

## ビルド

```bash
cd /workspaces/isaac-sim-ur5e/colcon_ws
colcon build --packages-select traj_follower
source install/setup.bash
```

## 実行方法

標準入力（Enterキー）による開始トリガーを確実に機能させるため、`ros2 run` を使用して実行します。

### 基本コマンド

```bash
ros2 run traj_follower follower_node --ros-args -p json_path:=$(pwd)/src/trajectory/results/spline.json
```

**手順:**

1.  上記コマンドを実行します。
2.  ターミナルに以下が表示されます。
    ```
    [INFO] [...]: Press Enter in the terminal to start the trajectory...
    ```
3.  Isaac Simのシミュレーションが再生中であることを確認し、**Enterキー** を押します。
4.  ロボットが軌道データに従って動作します。

### オプション

*   **ループ再生**:
    パラメータ `loop` を `true` に設定すると、軌道をループ再生します。

    ```bash
    ros2 run traj_follower follower_node --ros-args -p json_path:=$(pwd)/src/trajectory/results/spline.json -p loop:=true
    ```

## Launchファイルでの実行（非推奨）

Launchファイルを使用する場合、標準入力が取得しづらいため、起動後3秒で自動的に動作を開始するフォールバック機能が働きます。

```bash
ros2 launch traj_follower play_trajectory.launch.py json_path:=$(pwd)/src/trajectory/results/spline.json
```
