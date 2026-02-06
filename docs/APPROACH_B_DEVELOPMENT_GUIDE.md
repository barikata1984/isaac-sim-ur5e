# Approach B 開発ガイド

## 概要

このドキュメントは、Approach B 環境での日常的な開発作業のガイドラインです。

## Python 環境の理解

### 基本構成

```
Isaac Sim Python 3.11 (/isaac-sim/kit/python/bin/python3.11)
    ↓ すべてのパッケージをここにインストール
ROS 2 Jazzy (/opt/ros311) - Python 3.11 でビルド
```

**重要:** システム Python 3.12 (`/usr/bin/python3.12`) は使用しません。

### 環境変数

```bash
# Python
ISAAC_PYTHON=/isaac-sim/kit/python/bin/python3.11
python -> /isaac-sim/kit/python/bin/python3.11

# ROS
ROS_DISTRO=jazzy
LD_LIBRARY_PATH=/opt/ros311/lib:...

# PYTHONPATH
PYTHONPATH=/opt/ros311/lib/python3.11/site-packages:...
```

## パッケージ管理の黄金ルール

### ✅ DO: pip を使用

```bash
# Python パッケージは必ず pip
pip install numpy
pip install opencv-python
pip install scipy
```

### ❌ DON'T: apt の python3-* パッケージ

```bash
# これらは絶対に使わない
apt install python3-numpy     # ❌
apt install python3-opencv    # ❌
apt install python3-scipy     # ❌
```

**理由:** apt の `python3-*` パッケージは Python 3.12 用で、Isaac Sim Python 3.11 では動作しません。

### ✅ DO: C/C++ ライブラリは apt

```bash
# ヘッダーファイルと C++ ライブラリ
apt install libeigen3-dev      # ✅ OK
apt install libopencv-dev      # ✅ OK
apt install libboost-all-dev   # ✅ OK
```

## 開発ワークフロー

### 新しいパッケージを追加する場合

#### ステップ 1: コンテナ内でテスト

```bash
# コンテナに入る
docker exec -it isaac-sim-ur5e-approach-b bash

# 環境を確認
python --version  # Python 3.11.13

# pip でインストール
pip install <package-name>

# 動作確認
python -c "import <package>; print('OK')"
```

#### ステップ 2: requirements.txt に追加

```bash
# ホスト側で
echo "<package-name>>=<version>" >> requirements.txt

# 例
echo "opencv-python>=4.8.0" >> requirements.txt
```

#### ステップ 3: git commit

```bash
git add requirements.txt
git commit -m "Add <package-name> dependency"
```

#### ステップ 4: 他の開発者への共有

他の開発者は以下で同期：

```bash
docker exec isaac-sim-ur5e-approach-b pip install -r requirements.txt
```

または、コンテナを再ビルド：

```bash
docker compose -f docker/docker-compose.approach-b.yml down
docker compose -f docker/docker-compose.approach-b.yml build
docker compose -f docker/docker-compose.approach-b.yml up -d
```

### ROS 2 パッケージを追加する場合

#### オプション A: rosdep で管理（推奨）

プロジェクトの `package.xml` に依存関係を追加：

```xml
<package>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <!-- 新しい依存関係 -->
  <depend>nav2_msgs</depend>
</package>
```

#### オプション B: Dockerfile に追加（大規模な変更）

`Dockerfile.approach-b` の `rosinstall_generator` セクションに追加：

```dockerfile
RUN rosinstall_generator --deps --rosdistro jazzy \
    rcutils rcl rmw rclpy \
    # ... 既存のパッケージ ...
    nav2_msgs \  # ← 新しいパッケージ
    > ros2.jazzy.rosinstall
```

その後、イメージを再ビルド：

```bash
docker compose -f docker/docker-compose.approach-b.yml build --no-cache
```

## よくある質問

### Q1: OpenCV を使いたい

**A:** pip でインストール

```bash
# 基本版
pip install opencv-python

# contrib モジュール付き
pip install opencv-contrib-python

# 確認
python -c "import cv2; print(cv2.__version__)"
```

**CUDA サポートが必要な場合:**
ソースからビルドが必要（複雑）。まずは CPU 版で試してください。

### Q2: PCL (Point Cloud Library) を使いたい

**A:** Open3D で代替（推奨）

```bash
pip install open3d
```

Open3D は PCL の機能をカバーし、より使いやすいです。

### Q3: PyQt5 が必要

**A:** pip でインストール可能

```bash
pip install PyQt5

# または PySide6
pip install PySide6
```

### Q4: python3-apt が必要（apt 専用パッケージ）

**A:** システム Python 3.12 との併用

```python
# Isaac Sim Python スクリプト内
import subprocess

# apt パッケージ管理はシステム Python で実行
result = subprocess.run([
    '/usr/bin/python3.12', '-c',
    'import apt; ...'
], capture_output=True)
```

または、設計を変更して apt 操作を避ける。

### Q5: パッケージがインストールできない

**症状:**
```bash
$ pip install <package>
ERROR: Could not find a version that satisfies the requirement
```

**対処法:**

1. PyPI で検索: https://pypi.org/
2. 別名を試す（例: `python-pcl` → `open3d`）
3. ソースからビルド
4. 本当に必要か再検討

### Q6: ROS 2 パッケージが見つからない

**症状:**
```bash
$ ros2 run <package> <node>
Package '<package>' not found
```

**対処法:**

```bash
# 環境を source
source /opt/ros311/setup.bash
source /workspaces/isaac-sim-ur5e/colcon_ws/install/setup.bash

# パッケージを確認
ros2 pkg list | grep <package>

# なければ追加が必要
```

### Q7: import エラー: ModuleNotFoundError

**症状:**
```bash
$ python -c "import cv2"
ModuleNotFoundError: No module named 'cv2'
```

**対処法:**

```bash
# 1. Python バージョン確認
python --version  # Python 3.11.13

# 2. インストール済みパッケージ確認
pip list | grep opencv

# 3. なければインストール
pip install opencv-python

# 4. PYTHONPATH 確認
echo $PYTHONPATH
```

## トラブルシューティング

### AssertionError: SRE module mismatch

**原因:** `/opt/ros/jazzy/setup.bash` を source した

**解決:**

```bash
# 新しいシェルを起動
bash

# /opt/ros311 のみを source
source /opt/ros311/setup.bash
```

### ros2 launch が使えない

**原因:** ros2launch パッケージが不足

**解決:** Dockerfile を修正して再ビルド（既に修正済み）

### Docker ビルドが遅い

**原因:** キャッシュが効いていない

**解決:**

```bash
# 通常のビルド（キャッシュあり）
docker compose -f docker/docker-compose.approach-b.yml build

# キャッシュなし（問題がある場合のみ）
docker compose -f docker/docker-compose.approach-b.yml build --no-cache
```

## ベストプラクティス

### 1. requirements.txt で管理

```bash
# プロジェクトルート
/workspaces/isaac-sim-ur5e/
  ├── requirements.txt  ← ここに記述
  ├── colcon_ws/
  └── ...
```

```txt
# requirements.txt の例
numpy>=1.26.0
scipy>=1.11.0
opencv-contrib-python>=4.8.0
matplotlib>=3.7.0
open3d>=0.17.0
pin==2.7.0
```

### 2. devcontainer で自動インストール

`.devcontainer/devcontainer.approach-b.json`:

```json
{
  "postCreateCommand": "pip install -r requirements.txt && cd colcon_ws && colcon build --symlink-install"
}
```

### 3. Dockerfile の順序

```dockerfile
# 1. システムパッケージ（apt）
RUN apt-get install libeigen3-dev

# 2. Python ビルドツール（pip）
RUN pip install colcon-common-extensions

# 3. ROS 2 ソースビルド

# 4. プロジェクト固有パッケージ（pip）
COPY requirements.txt /tmp/
RUN pip install -r /tmp/requirements.txt
```

### 4. コンテナの状態管理

```bash
# 起動
docker compose -f docker/docker-compose.approach-b.yml up -d

# 停止（コンテナは保持）
docker compose -f docker/docker-compose.approach-b.yml stop

# 停止 & 削除（キャッシュは保持）
docker compose -f docker/docker-compose.approach-b.yml down

# 完全削除（キャッシュも削除）
docker compose -f docker/docker-compose.approach-b.yml down --volumes
```

## チェックリスト

### 新しいパッケージを追加する前に

- [ ] PyPI で検索済み
- [ ] apt の python3-* パッケージではない
- [ ] pip でインストール可能
- [ ] 動作確認済み
- [ ] requirements.txt に追加済み
- [ ] git commit 済み

### トラブルが発生したら

- [ ] Python バージョン確認（3.11.13）
- [ ] `/opt/ros/jazzy/setup.bash` を source していない
- [ ] `/opt/ros311/setup.bash` を source 済み
- [ ] pip list で確認
- [ ] PYTHONPATH を確認
- [ ] ドキュメント（APPROACH_B_SAFE_SETUP.md）を参照

## 参考資料

- [APPROACH_B_SAFE_SETUP.md](APPROACH_B_SAFE_SETUP.md) - セットアップとトラブルシュート
- [APPROACH_B_IMPLEMENTATION_PLAN.md](APPROACH_B_IMPLEMENTATION_PLAN.md) - 実装計画と結果
- [APPROACH_B_COMMANDS.md](APPROACH_B_COMMANDS.md) - コマンドリファレンス
- [CLAUDE.md](CLAUDE.md) - プロジェクト全体の設定

## まとめ

**Approach B での開発の鍵:**

1. **Python パッケージは pip のみ**
2. **C/C++ ライブラリは apt**
3. **requirements.txt で管理**
4. **/opt/ros/jazzy は source しない**

この4つのルールを守れば、安定した開発環境が維持できます。
