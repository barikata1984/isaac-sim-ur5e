# Kinematics Package

シリアルリンクロボットの順運動学とヤコビアン計算を提供するパッケージです。

## 機能概要

- **順運動学**: Modified DH規約に基づく各リンクフレームの変換行列計算
- **ヤコビアン**: Space/Body Jacobianの計算
- **手先速度・加速度**: ヤコビアンを用いた手先ツイストと加速度の計算

## ディレクトリ構成

```
kinematics/
├── README.md           # 本ファイル
├── package.xml         # ROS 2パッケージ定義
├── setup.py            # Pythonパッケージ設定
├── pytest.ini          # pytest設定
├── resource/
│   └── kinematics      # amentリソースマーカー
├── src/
│   ├── __init__.py     # パッケージ初期化
│   └── kinematics.py   # 順運動学・ヤコビアン
└── test/
    ├── __init__.py
    ├── conftest.py         # pytest fixture定義
    └── test_kinematics.py  # kinematics.pyのテスト
```

## モジュール詳細

### kinematics.py

順運動学とヤコビアン計算を提供します。pymlg規約 `[ω, v]`（角速度、線速度）を使用。

| 関数 | 説明 |
|------|------|
| `forward_kinematics(q, dh_params)` | 順運動学（ベース→手先変換） |
| `forward_kinematics_all_frames(q, dh_params)` | 全リンクフレームの変換 |
| `space_jacobian(q, dh_params)` | Space Jacobian（ベースフレーム基準） |
| `body_jacobian(q, dh_params)` | Body Jacobian（手先フレーム基準） |
| `geometric_jacobian(q, dh_params, frame)` | 幾何ヤコビアン（frame選択可能） |
| `tool0_twist(q, dq, dh_params, frame)` | 手先ツイスト V = J·dq |
| `tool0_acceleration(q, dq, ddq, dh_params, frame)` | 手先加速度 |

### 内部関数

| 関数 | 説明 |
|------|------|
| `_dh_transform(a, d, alpha, theta)` | Modified DH変換行列（4×4） |
| `_numerical_jacobian(q, dh_params, eps)` | 数値微分によるヤコビアン計算 |

## テスト

### テスト内容

#### test_kinematics.py（13テスト）

| テストクラス | 検証内容 |
|-------------|---------|
| `TestForwardKinematics` | FKの形状、全フレーム計算、関節独立性 |
| `TestJacobian` | ヤコビアン形状、数値微分との比較、Space/Body関係 |
| `TestTwist` | ツイスト計算（ゼロ速度、形状、線形性） |
| `TestAcceleration` | 加速度計算（形状、ゼロ速度時の簡略化） |
| `TestSingularity` | 特異点近傍での挙動 |

### テスト実行手順

```bash
# パッケージディレクトリへ移動
cd /workspaces/isaac-sim-ur5e/colcon_ws/src/kinematics

# 全テスト実行
python3 -m pytest test/ -v

# 特定テストファイルのみ
python3 -m pytest test/test_kinematics.py -v

# 特定テストクラスのみ
python3 -m pytest test/test_kinematics.py::TestJacobian -v
```

## 使用例

```python
import numpy as np
from pymlg.numpy import SE3
from kinematics import forward_kinematics, space_jacobian, body_jacobian, tool0_twist

# UR5e DH parameters (例)
dh_params = np.array([
    [0.0,     0.089159,  np.pi/2],
    [-0.425,  0.0,       0.0],
    [-0.392,  0.0,       0.0],
    [0.0,     0.10915,   np.pi/2],
    [0.0,     0.09465,  -np.pi/2],
    [0.0,     0.0823,    0.0],
])

# 関節角・速度
q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
dq = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

# 順運動学
T_ee = forward_kinematics(q, dh_params)
print(f"End-effector position: {T_ee[:3, 3]}")

# ヤコビアン
J_s = space_jacobian(q, dh_params)
J_b = body_jacobian(q, dh_params)
print(f"Jacobian shape: {J_s.shape}")

# Space/Body Jacobian の関係検証
# J_s = Ad_{T_0n} @ J_b
Ad_T = SE3.adjoint(T_ee)
J_s_from_body = Ad_T @ J_b
print(f"J_s == Ad_T @ J_b: {np.allclose(J_s, J_s_from_body)}")

# 手先ツイスト [ω, v]
V = tool0_twist(q, dq, dh_params)
print(f"End-effector twist [ω, v]: {V}")
print(f"  Angular velocity: {V[:3]}")
print(f"  Linear velocity:  {V[3:]}")
```

## 依存関係

- `numpy`
- `pymlg` - Lie群演算ライブラリ（SE3, SO3）

## 規約

- **Twist規約**: pymlg形式 `[ω, v]`（角速度、線速度）- Lynch & Park準拠
- **DH規約**: Modified DH（Craig）、パラメータ順序 `[a, d, alpha]`

**注意**: 本パッケージは `[ω, v]` 規約を使用しています。Isaac Sim/Pinocchio（`[v, ω]` 規約）との連携時は変換が必要です：

```python
# [ω, v] から [v, ω] への変換
def omega_v_to_v_omega(twist_wv):
    return np.concatenate([twist_wv[3:], twist_wv[:3]])

# [v, ω] から [ω, v] への変換
def v_omega_to_omega_v(twist_vw):
    return np.concatenate([twist_vw[3:], twist_vw[:3]])
```

## ヤコビアン構造

`[ω, v]` 規約におけるヤコビアンの構造：

```
J = [J_angular]   # (3, n) 角速度成分
    [J_linear ]   # (3, n) 線速度成分

V = J @ dq

V[0:3] = ω (angular velocity)
V[3:6] = v (linear velocity)
```
