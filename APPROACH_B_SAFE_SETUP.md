# アプローチB 安全セットアップガイド

## 概要

既存のコンテナを保持したまま、アプローチBの新しいコンテナを並行して使用する手順です。

## 環境の並行運用

### コンテナ名の使い分け

| 環境 | コンテナ名 | イメージ名 | 用途 |
|------|-----------|-----------|------|
| **現行環境** | `isaac-sim-ur5e` | `isaac-sim-ur5e` | 既存の開発環境（保持） |
| **アプローチB** | `isaac-sim-ur5e-approach-b` | `isaac-sim-ur5e:approach-b-python311` | 新しい統一環境（テスト） |

両方のコンテナを同時に起動することも可能ですが、**ポート競合に注意してください**（network_mode: host を使用しているため）。

---

## セットアップ手順

### Phase 1: Docker イメージのビルドとコンテナ起動

#### ステップ 1: 現在のコンテナ状態を確認

```bash
# 現在実行中のコンテナを確認
docker ps

# 既存コンテナが実行中の場合、そのまま残しておく
```

#### ステップ 2: アプローチB のイメージをビルド

```bash
cd /home/atsushi/workspace/isaac-sim-ur5e/docker

# アプローチB の docker-compose を使用してビルド
docker-compose -f docker-compose.approach-b.yml build --no-cache
```

**ビルド時間:** 初回は 30〜60分 かかります。

**ビルド中の確認事項:**
- [ ] Python 3.11 のシンボリックリンク作成
- [ ] システムパッケージのインストール
- [ ] Python パッケージ（empy==3.3.4 等）のインストール
- [ ] ROS 2 ソースのダウンロード
- [ ] ROS 2 のビルド（/opt/ros311）

#### ステップ 3: ビルドログの確認

```bash
# ビルドが成功したか確認
docker images | grep isaac-sim-ur5e

# 期待される出力:
# isaac-sim-ur5e   approach-b-python311   <IMAGE_ID>   <SIZE>   <TIME>
```

#### ステップ 4: アプローチB コンテナの起動

```bash
# アプローチB のコンテナを起動
cd /home/atsushi/workspace/isaac-sim-ur5e/docker
docker-compose -f docker-compose.approach-b.yml up -d

# 起動確認
docker ps | grep approach-b

# ログを確認（エラーがないか）
docker-compose -f docker-compose.approach-b.yml logs -f
```

#### ステップ 5: 基本動作確認（コンテナ内）

```bash
# アプローチB コンテナに入る
docker exec -it isaac-sim-ur5e-approach-b /bin/bash

# Python バージョン確認
python --version
# 期待: Python 3.11.x

# which python
which python
# 期待: /usr/local/bin/python -> /isaac-sim/kit/python/bin/python3.11

# ROS 環境を source
source /isaac-sim/setup_python_env.sh
source /opt/ros311/setup.bash

# ROS 2 CLI 動作確認
ros2 --help

# Python import テスト
python -c "import sys; print(sys.version)"
python -c "import rclpy; print('rclpy:', rclpy.__file__)"

# 成功したら exit
exit
```

---

### Phase 2: VSCode devcontainer でコンテナに接続

#### ステップ 6: devcontainer 設定の準備

現時点では、既存の `.devcontainer/devcontainer.json` はそのままにして、**アプローチB専用の設定を別に使用**します。

**オプション A: devcontainer.json を一時的に置き換える方法**

```bash
cd /home/atsushi/workspace/isaac-sim-ur5e

# 既存の devcontainer.json をバックアップ
cp .devcontainer/devcontainer.json .devcontainer/devcontainer.json.backup

# アプローチB の設定を使用
cp .devcontainer/devcontainer.approach-b.json .devcontainer/devcontainer.json
```

**オプション B: VSCode の "Attach to Running Container" を使用（推奨）**

この方法では、既存の devcontainer.json を変更せずに、実行中のコンテナに直接接続できます。

#### ステップ 7: VSCode でコンテナに接続

**オプション A を選んだ場合:**

1. VSCode でプロジェクトフォルダを開く
2. `F1` → `Dev Containers: Reopen in Container`
3. コンテナが起動し、`postCreateCommand` で colcon ビルドが自動実行される

**オプション B を選んだ場合（推奨）:**

1. VSCode でプロジェクトフォルダを開く
2. `F1` → `Dev Containers: Attach to Running Container...`
3. `isaac-sim-ur5e-approach-b` を選択
4. 新しいVSCodeウィンドウが開く
5. `File` → `Open Folder` → `/workspaces/isaac-sim-ur5e` を開く

#### ステップ 8: コンテナ内での環境確認

VSCode のターミナルで以下を実行：

```bash
# Python バージョン確認
python --version
# 期待: Python 3.11.x

# 環境変数確認
echo $PYTHONPATH
# 期待: /opt/ros311/lib/python3.11/site-packages

echo $ROS_DISTRO
# 期待: jazzy

# ROS 環境を source（初回のみ必要な場合）
source /isaac-sim/setup_python_env.sh
source /opt/ros311/setup.bash

# ROS 2 動作確認
ros2 topic list

# Python import テスト
python -c "import rclpy; print('rclpy OK')"
python -c "import std_msgs.msg; print('std_msgs OK')"
python -c "from isaacsim import SimulationApp; print('Isaac Sim OK')"
```

---

### Phase 3: colcon ワークスペースのビルド

#### ステップ 9: colcon_ws をビルド

```bash
cd /workspaces/isaac-sim-ur5e/colcon_ws

# ROS 環境を source（まだの場合）
source /opt/ros311/setup.bash

# ビルド実行
colcon build --symlink-install --cmake-args \
    -DPython3_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11 \
    -DPYTHON_EXECUTABLE=/isaac-sim/kit/python/bin/python3.11

# ビルド結果を確認
ls -la install/

# ビルド結果を source
source install/setup.bash
```

#### ステップ 10: パッケージのテスト

```bash
# kinematics パッケージ
ros2 run kinematics kinematics_node --help

# trajectory パッケージ
ros2 run trajectory trajectory_generator --help

# （該当する場合）iparam_identification
python src/iparam_identification/scripts/run_identification_test.py
```

---

## コンテナの切り替え方法

### 既存環境に戻る場合

```bash
# アプローチB コンテナを停止（削除はしない）
cd /home/atsushi/workspace/isaac-sim-ur5e/docker
docker-compose -f docker-compose.approach-b.yml stop

# 既存コンテナを起動（停止していた場合）
docker-compose -f docker-compose.yml up -d

# VSCode で既存環境に再接続
# オプションA を使った場合: devcontainer.json を元に戻す
cp .devcontainer/devcontainer.json.backup .devcontainer/devcontainer.json
# その後、F1 → Dev Containers: Reopen in Container

# オプションB を使った場合: 既存コンテナにアタッチ
# F1 → Dev Containers: Attach to Running Container → isaac-sim-ur5e
```

### アプローチB 環境に切り替える場合

```bash
# アプローチB コンテナを起動
cd /home/atsushi/workspace/isaac-sim-ur5e/docker
docker-compose -f docker-compose.approach-b.yml up -d

# VSCode でアプローチB環境に接続
# オプションA: devcontainer.json を切り替え
# オプションB: Attach to Running Container → isaac-sim-ur5e-approach-b
```

---

## トラブルシューティング

### ❌ `ros2 launch` コマンドが使えない

**症状:**
```bash
$ ros2 launch
ros2: error: argument Call `ros2 <command> -h` for more detailed usage.: invalid choice: 'launch'
```

**原因:**
`rosinstall_generator` のパッケージリストに `ros2launch` パッケージが含まれていない。

**解決策:**

1. Dockerfile.approach-b の `rosinstall_generator` セクションを確認：

```dockerfile
RUN rosinstall_generator --deps --rosdistro jazzy \
    rcutils rcl rmw rclpy \
    std_msgs sensor_msgs geometry_msgs nav_msgs \
    tf2 tf2_ros tf2_msgs \
    ros2topic ros2node ros2run ros2pkg ros2launch \  # ← ros2launch を追加
    launch launch_ros launch_xml launch_yaml \        # ← launch パッケージを追加
    ros_environment common_interfaces rosgraph_msgs \
    > ros2.jazzy.rosinstall \
    && mkdir -p src && vcs import src < ros2.jazzy.rosinstall
```

2. イメージを再ビルド：

```bash
docker compose -f docker/docker-compose.approach-b.yml down --rmi all
docker compose -f docker/docker-compose.approach-b.yml build --no-cache
```

3. 確認：

```bash
docker exec isaac-sim-ur5e-approach-b bash -c \
    "source /opt/ros311/setup.bash && ros2 launch --help"
```

---

### ❌ Python パッケージのインポートエラー: `AssertionError: SRE module mismatch`

**症状:**
```bash
$ source /opt/ros/jazzy/setup.bash
AssertionError: SRE module mismatch
```

**原因:**
`/opt/ros/jazzy/setup.bash` は apt でインストールされた ROS Jazzy（Python 3.12 用）の環境を設定します。Isaac Sim の Python 3.11 とバイナリ互換性がありません。

**解決策:**

**`/opt/ros/jazzy/setup.bash` を source しない** でください。代わりに：

```bash
# ✅ 正しい方法
bash  # 新しいシェルを起動（環境リセット）
source /opt/ros311/setup.bash  # Python 3.11 用の ROS 2 のみ

# ❌ 間違った方法
source /opt/ros/jazzy/setup.bash  # これはしない
```

Dockerfile の `.bashrc` 設定も修正が必要な場合があります（/opt/ros/jazzy の行を削除）。

---

### ⚠️ Python パッケージ管理の重要な制約

#### 問題: apt でインストールした Python パッケージが使えない

**症状:**
```bash
$ apt install python3-opencv
$ python -c "import cv2"
ModuleNotFoundError: No module named 'cv2'
```

**原因:**
`apt install python3-*` は**システム Python 3.12** の `/usr/lib/python3.12/` にインストールされます。Isaac Sim Python 3.11 はこのパスを見ないため、インポートできません。

**解決策と開発ガイドライン:**

#### ✅ Python パッケージは必ず pip でインストール

```bash
# ❌ NG: apt は使わない
apt install python3-opencv

# ✅ OK: pip を使用
pip install opencv-python
```

#### ✅ C/C++ ライブラリは apt で OK

```bash
# ✅ OK: Python と無関係なライブラリ
apt install libeigen3-dev
apt install libopencv-dev  # ヘッダーファイルと C++ ライブラリ
apt install libboost-all-dev
```

#### パッケージの判断基準

| 種類 | インストール方法 | 理由 |
|------|------------------|------|
| Python パッケージ | `pip install <package>` | Isaac Sim Python 3.11 に直接インストール |
| C/C++ ライブラリ | `apt install lib<package>-dev` | Python バージョンに依存しない |
| ROS 2 パッケージ | ソースからビルド（rosinstall_generator） | Python 3.11 互換バイナリを生成 |

#### 開発中にパッケージを追加する場合

```bash
# 1. コンテナに入る
docker exec -it isaac-sim-ur5e-approach-b bash

# 2. ❌ これはしない
# apt install python3-<package>

# 3. ✅ pip を使う
pip install <package>

# 4. 動作確認後、requirements.txt に追加
echo "<package>>=<version>" >> requirements.txt

# 5. git commit
git add requirements.txt
git commit -m "Add <package> dependency"
```

#### apt 専用パッケージへの対処

一部のパッケージ（`python3-apt`, `python3-gi` など）は pip で入手できません。

**対処法:**
1. pip で代替パッケージがないか確認（例: `PyGObject`）
2. ソースからビルド（Isaac Sim Python で）
3. 本当に必要かを再検討（設計変更で回避）
4. システム Python 3.12 との併用（subprocess 経由）

**ロボット工学でよく使うパッケージの対処法:**

| パッケージ | apt 専用？ | 対処法 |
|-----------|-----------|--------|
| OpenCV | No | `pip install opencv-python` |
| NumPy/SciPy | No | `pip install numpy scipy` |
| PCL | Yes（代替あり） | `pip install open3d` で代替 |
| PyQt5 | No | `pip install PyQt5` |
| VTK | No | `pip install vtk` |
| RealSense | No | `pip install pyrealsense2` |
| python3-apt | Yes | システム Python 併用 |

---

### ビルドエラー: "empy module not found"

```bash
# コンテナ内で確認
docker exec -it isaac-sim-ur5e-approach-b /bin/bash
python -c "import em; print(em.__version__)"

# 3.3.4 でない場合は再インストール
/isaac-sim/kit/python/bin/python3.11 -m pip uninstall -y em empy
/isaac-sim/kit/python/bin/python3.11 -m pip install empy==3.3.4
```

### ビルドエラー: "Python.h not found"

```bash
# シンボリックリンクを確認
docker exec -it isaac-sim-ur5e-approach-b ls -l /usr/include/python3

# 存在しない場合は作成
docker exec -it isaac-sim-ur5e-approach-b \
    ln -sf /isaac-sim/kit/python/include/python3.11 /usr/include/python3
```

### ポート競合エラー

両方のコンテナが `network_mode: host` を使用しているため、同時起動時にポート競合が発生する可能性があります。

**対策:**
- 片方のコンテナを停止してからもう片方を起動する
- または、docker-compose.approach-b.yml で `network_mode: bridge` に変更し、必要なポートのみを公開する

### コンテナが起動しない

```bash
# ログを確認
docker-compose -f docker-compose.approach-b.yml logs

# コンテナの状態を確認
docker ps -a | grep approach-b

# 強制再起動
docker-compose -f docker-compose.approach-b.yml down
docker-compose -f docker-compose.approach-b.yml up -d
```

---

### ビルド時の警告について

#### CMake unused variables

```
CMake Warning:
  Manually-specified variables were not used by the project:
    PYTHON_EXECUTABLE
    PYTHON_INCLUDE_DIR
    PYTHON_LIBRARY
```

**これは問題ありません。** C++ のみのパッケージは Python 変数を使用しないため、警告が表示されます。

#### setuptools_scm git エラー

```
ERROR setuptools_scm._file_finders.git listing git files failed - pretending there aren't any
```

**これは問題ありません。** Docker ビルド中に `.git` ディレクトリがないため表示されますが、パッケージは正常にビルドされます。

---

## クリーンアップ（アプローチBを削除する場合）

テストが完了し、アプローチBが不要になった場合：

```bash
# コンテナを停止・削除
cd /home/atsushi/workspace/isaac-sim-ur5e/docker
docker-compose -f docker-compose.approach-b.yml down

# イメージを削除
docker rmi isaac-sim-ur5e:approach-b-python311

# Named volumes を削除（colcon キャッシュ）
docker volume rm docker_colcon_build_cache
docker volume rm docker_colcon_install_cache
docker volume rm docker_colcon_log_cache

# devcontainer.json を元に戻す（オプションAを使った場合）
cp .devcontainer/devcontainer.json.backup .devcontainer/devcontainer.json
```

---

## 成功時の次のステップ

アプローチBが正常に動作することを確認できたら：

1. **既存環境との比較**
   - パフォーマンス
   - 安定性
   - xacro エラーの解消確認

2. **本番環境への移行判断**
   - 問題がなければ、既存環境を置き換え
   - または、並行運用を継続

3. **ドキュメントの更新**
   - CLAUDE.md に新しい環境のセットアップ手順を追加
   - README.md を更新

---

## チェックリスト

### Phase 1: Docker

- [ ] 既存コンテナの状態を確認
- [ ] アプローチB イメージのビルド成功
- [ ] アプローチB コンテナの起動成功
- [ ] Python 3.11 の確認
- [ ] ROS 2 CLI の動作確認
- [ ] `import rclpy` の動作確認

### Phase 2: devcontainer

- [ ] VSCode でコンテナに接続成功
- [ ] 環境変数の確認
- [ ] Pylance の動作確認
- [ ] ターミナルで ROS コマンドが使用可能

### Phase 3: colcon

- [ ] colcon ビルド成功
- [ ] 既存パッケージの動作確認
- [ ] テストの実行成功

---

## まとめ

この手順により、既存環境を壊すことなく、アプローチBを安全にテストできます。問題が発生した場合は、既存環境にすぐに戻せます。

**推奨: オプションB（Attach to Running Container）を使用することで、devcontainer.json を変更せずにテストできます。**
