# UR5e 逆動力学実装計画

## 目的

物体の慣性パラメータ推定のため、tool0座標系での速度・角速度・加速度・角加速度を関節データから計算する機能を実装する。

## 前提条件

- 外部ライブラリ（Pinocchio、KDL等）は使用せず**自前実装**
- 座標系は**tool0座標系に統一**
- ROS 2パッケージとして実装
- 既存プロジェクトのコーディングパターンに準拠

---

## パッケージ構成

```
colcon_ws/src/ur5e_kinematics/
├── package.xml
├── setup.py
├── resource/ur5e_kinematics
├── src/
│   ├── __init__.py
│   ├── dh_parameters.py      # DHパラメータ定義
│   ├── transforms.py         # 同次変換行列ユーティリティ
│   ├── forward_kinematics.py # 順運動学
│   ├── jacobian.py           # ヤコビアン計算
│   ├── velocity_kinematics.py # 速度・加速度計算
│   ├── kinematics_node.py    # ROS 2ノード
│   └── kinematics_cli.py     # CLIツール
└── test/
    ├── test_transforms.py
    ├── test_forward_kinematics.py
    ├── test_jacobian.py
    └── test_velocity_kinematics.py
```

---

## 実装フェーズ

### Phase 1: 基盤実装

| ファイル | 内容 |
|---------|------|
| `dh_parameters.py` | UR5e DHパラメータを`@dataclass(frozen=True)`で定義 |
| `transforms.py` | DH行列生成、回転行列抽出、歪対称行列 |
| `forward_kinematics.py` | 順運動学計算（位置・姿勢）、全中間フレーム取得 |

**UR5e DHパラメータ (Modified DH, Craig convention)**:
| Joint | a (m) | d (m) | alpha (rad) |
|-------|-------|-------|-------------|
| 1 | 0 | 0.089159 | π/2 |
| 2 | -0.425 | 0 | 0 |
| 3 | -0.392 | 0 | 0 |
| 4 | 0 | 0.10915 | π/2 |
| 5 | 0 | 0.09465 | -π/2 |
| 6 | 0 | 0.0823 | 0 |

### Phase 2: ヤコビアン実装

| ファイル | 内容 |
|---------|------|
| `jacobian.py` | 幾何ヤコビアン計算、tool0フレーム変換、特異点検出 |

**計算方法**（回転関節）:
- 線速度: `J_v_i = z_{i-1} × (p_ee - p_{i-1})`
- 角速度: `J_ω_i = z_{i-1}`

**tool0座標系変換**:
```
J_tool = [R_6_0   0   ] × J_base
         [0     R_6_0]
```

### Phase 3: 速度運動学

| ファイル | 内容 |
|---------|------|
| `velocity_kinematics.py` | 速度・加速度計算のファサード |

**計算式**:
- 速度: `ẋ = J(q) × dq`
- 加速度: `ẍ = J(q) × ddq + dJ(q,dq) × dq`

**出力データクラス `Tool0Twist`**:
- `linear_velocity`: [vx, vy, vz] [m/s]
- `angular_velocity`: [ωx, ωy, ωz] [rad/s]
- `linear_acceleration`: [ax, ay, az] [m/s²]
- `angular_acceleration`: [αx, αy, αz] [rad/s²]

### Phase 4: ROS 2統合

| ファイル | 内容 |
|---------|------|
| `kinematics_node.py` | ROS 2ノード（サブスクライブ/パブリッシュ） |
| `kinematics_cli.py` | tyroベースCLI |

**トピック**:
| 入力 | 出力 |
|------|------|
| `joint_positions` (6要素) | `tool0_velocity` (6要素: v,ω) |
| `joint_velocities` (6要素) | `tool0_acceleration` (6要素: a,α) |
| `joint_accelerations` (6要素) | `tool0_twist` (12要素: v,ω,a,α) |

---

## 主要クラス設計

### VelocityKinematics (メインAPI)

```python
class VelocityKinematics:
    def compute_full_state(
        self,
        q: np.ndarray,      # 関節位置 [rad] (6,)
        dq: np.ndarray,     # 関節速度 [rad/s] (6,)
        ddq: np.ndarray     # 関節加速度 [rad/s²] (6,)
    ) -> Tool0Twist:
        """tool0座標系での速度・加速度を計算"""
```

---

## 変更対象ファイル

### 新規作成
- `colcon_ws/src/ur5e_kinematics/` （パッケージ全体）

### 参照（変更なし）
- `trajectories/src/base_trajectory.py` - dataclassパターン参照
- `trajectories/setup.py` - setup.py構造参照

---

## 検証方法

### 1. 単体テスト (pytest)

| テスト | 検証内容 |
|--------|---------|
| `test_transforms.py` | DH行列の直交性、逆変換 |
| `test_forward_kinematics.py` | ゼロ姿勢での位置、既知姿勢との比較 |
| `test_jacobian.py` | 数値微分との比較、特異点検出 |
| `test_velocity_kinematics.py` | 静止状態でゼロ、単一関節回転テスト |

```bash
cd /workspaces/isaac-sim-ur5e/colcon_ws
colcon test --packages-select ur5e_kinematics
colcon test-result --verbose
```

### 2. Isaac Simとの比較検証

軌道再生中にIsaac Simから取得したエンドエフェクタ速度と計算値を比較:
- 許容誤差: 位置 1e-4 m、角度 1e-4 rad

---

## 実行コマンド

```bash
# ビルド
cd /workspaces/isaac-sim-ur5e/colcon_ws
colcon build --packages-select ur5e_kinematics
source install/setup.bash

# CLI: 単一計算
ros2 run ur5e_kinematics kinematics_cli compute \
  --q 0 -1.57 0 -1.57 0 0 \
  --dq 0.1 0 0 0 0 0 \
  --ddq 0 0 0 0 0 0

# CLI: バッチ処理（軌道ファイルから）
ros2 run ur5e_kinematics kinematics_cli batch \
  --json-path data/trajectory.json \
  --output-path data/tool0_states.json

# ROS 2ノード起動
ros2 run ur5e_kinematics kinematics_node --ros-args -p frame:=tool
```

---

## 調査で判明した事項

### 現実UR5e内蔵F/Tセンサ

| 項目 | 仕様 |
|------|------|
| 測定範囲 | 力: ±50N, トルク: ±10Nm |
| 精度 | 力: ±4.0N, トルク: ±0.3Nm |
| サンプリングレート | 最大500Hz (RTDE) |
| 座標系 | **ベース座標系で表現**（tool0で測定） |
| データ取得 | `actual_TCP_force` (RTDE) または `get_tcp_force()` (URScript) |

### Isaac Sim UR5e 現状

- F/Tセンサ: シミュレートされていない（追加が必要）
- センサ設定: なし
- ビルトインアセット `/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd` を使用

---

## 今後のフェーズ（参考）

### フェーズ2: Isaac Sim F/Tセンサ追加
- UR5eモデルにF/Tセンサを追加
- ROS 2トピックでセンサデータ公開
- 現実URとの整合性検証

### フェーズ3: 慣性パラメータ同定
- 運動方程式のセットアップ（ニュートン・オイラー）
- 最小二乗法/全最小二乗法による同定
- 検証
