# Dynamics Package

Newton-Euler法に基づくシリアルリンクロボットの逆動力学計算を提供するパッケージです。
Lynch and Park 2017, Chapter 8（Dynamics of Open Chains）に基づいて実装されています。

## 機能概要

- **Newton-Euler逆動力学**: τ = M(q)q̈ + c(q,q̇) + g(q) の計算
- **質量行列**: M(q) の計算
- **重力トルク**: g(q) の計算
- **コリオリ・遠心力**: c(q,q̇) の計算
- **空間慣性行列**: 重心・関節フレームにおける6×6空間慣性行列

## ディレクトリ構成

```
dynamics/
├── README.md           # 本ファイル
├── package.xml         # ROS 2パッケージ定義
├── setup.py            # Pythonパッケージ設定
├── pytest.ini          # pytest設定
├── resource/
│   └── dynamics        # amentリソースマーカー
├── src/
│   ├── __init__.py         # パッケージ初期化
│   ├── spatial_inertia.py  # 空間慣性行列ユーティリティ
│   └── newton_euler.py     # Newton-Euler逆動力学
└── test/
    ├── __init__.py
    ├── conftest.py                  # pytest fixture定義
    ├── test_spatial_inertia.py      # 空間慣性テスト
    ├── test_newton_euler.py         # Newton-Eulerテスト
    └── test_pinocchio_comparison.py # Pinocchioクロス検証
```

## モジュール詳細

### newton_euler.py

Newton-Euler再帰アルゴリズム（RNEA）による逆動力学計算を提供します。
内部計算はpymlgの `[ω, v]` 規約（角速度、線速度）を使用。

| クラス/関数 | 説明 |
|------------|------|
| `NewtonEulerResult` | 逆動力学計算結果（τ, ツイスト, 加速度, レンチ） |
| `NewtonEulerDynamics` | Newton-Euler動力学クラス |

#### NewtonEulerDynamics メソッド

| メソッド | 説明 |
|---------|------|
| `from_robot_params(params, gravity)` | ロボットパラメータから生成 |
| `inverse_dynamics(q, dq, ddq, F_tip)` | 逆動力学計算 |
| `forward_pass(q, dq, ddq)` | 前方反復（ツイスト・加速度伝播） |
| `backward_pass(transforms, twists, accels, F_tip)` | 後方反復（レンチ・トルク計算） |
| `mass_matrix(q)` | 質量行列 M(q) |
| `gravity_torques(q)` | 重力トルク g(q) |
| `coriolis_centrifugal(q, dq)` | コリオリ・遠心力 c(q,dq) |

### spatial_inertia.py

空間慣性行列の計算ユーティリティを提供します。pymlg規約 `[ω, v]` を使用。

| 関数 | 説明 |
|------|------|
| `spatial_inertia_at_com(mass, inertia)` | 重心における空間慣性（6×6） |
| `spatial_inertia_at_frame(mass, inertia, com_pos)` | 関節フレームにおける空間慣性 |
| `transform_spatial_inertia(G, T)` | 空間慣性のフレーム変換 |
| `is_positive_definite(G)` | 正定値判定 |
| `is_symmetric(G)` | 対称性判定 |

## テスト

### テスト内容

#### test_newton_euler.py（17テスト）

| テストクラス | 検証内容 |
|-------------|---------|
| `TestNewtonEulerResult` | 結果データ構造の検証 |
| `TestInverseDynamics` | 静的平衡、無重力、τ=Mq̈+c+g の恒等式 |
| `TestGravityTorques` | 重力トルクの形状、構成依存性 |
| `TestMassMatrix` | 対称性、正定値性、構成依存性 |
| `TestCoriolisCentrifugal` | 形状、ゼロ速度、速度二乗依存性 |
| `TestForwardBackwardPass` | ツイスト伝播の正確性 |
| `TestExternalWrench` | 外力がトルクに与える影響 |

#### test_pinocchio_comparison.py（8テスト）

業界標準ライブラリPinocchioとのクロス検証テスト。

| テストクラス | 検証内容 |
|-------------|---------|
| `TestPinocchioComparison` | 重力トルク、質量行列、逆動力学の比較 |
| `TestTwistComparison` | リンクツイストの直接比較（規約変換後） |
| `TestMassMatrixProperties` | 質量行列固有値の一致 |

**重要**: 規約の違い
- 本実装: pymlg `[ω, v]` 規約
- Pinocchio: `[v, ω]` 規約
- テストでは `[v, ω]` → `[ω, v]` に変換して比較

#### test_spatial_inertia.py（11テスト）

| テストクラス | 検証内容 |
|-------------|---------|
| `TestSpatialInertiaAtCom` | 形状、ブロック対角、対称性、正定値性 |
| `TestSpatialInertiaAtFrame` | 平行軸定理、結合項の正確性 |
| `TestUR5eInertia` | UR5eパラメータでの検証 |
| `TestTransformSpatialInertia` | フレーム変換、固有値保存 |

### テスト実行手順

```bash
# パッケージディレクトリへ移動
cd /workspaces/isaac-sim-ur5e/colcon_ws/src/dynamics

# 全テスト実行
python3 -m pytest test/ -v

# 特定テストファイルのみ
python3 -m pytest test/test_newton_euler.py -v
python3 -m pytest test/test_pinocchio_comparison.py -v
python3 -m pytest test/test_spatial_inertia.py -v

# Pinocchioクロス検証のみ
python3 -m pytest test/test_pinocchio_comparison.py -v

# 特定テストクラスのみ
python3 -m pytest test/test_newton_euler.py::TestMassMatrix -v
```

### Pinocchioクロス検証について

Pinocchio（`pip install pin`）がインストールされている場合、以下を検証：

1. **重力トルク**: `pin.computeGeneralizedGravity()` と比較
2. **質量行列**: `pin.crba()` と比較
3. **逆動力学**: `pin.rnea()` と比較
4. **リンクツイスト**: `pin.forwardKinematics()` のツイストと比較

Pinocchioがない場合、これらのテストは自動的にスキップされます。

## 使用例

```python
import numpy as np
from newton_euler import NewtonEulerDynamics
from ur5e import UR5eParameters  # robots/ur パッケージ

# UR5eパラメータからdynamicsインスタンス作成
params = UR5eParameters()
dynamics = NewtonEulerDynamics.from_robot_params(params)

# 関節状態
q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
dq = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
ddq = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# 逆動力学
result = dynamics.inverse_dynamics(q, dq, ddq)
print(f"Required torques: {result.tau}")

# 質量行列
M = dynamics.mass_matrix(q)
print(f"Mass matrix shape: {M.shape}")
print(f"Mass matrix eigenvalues: {np.linalg.eigvalsh(M)}")

# 重力トルク
g = dynamics.gravity_torques(q)
print(f"Gravity torques: {g}")

# コリオリ・遠心力
c = dynamics.coriolis_centrifugal(q, dq)
print(f"Coriolis/centrifugal: {c}")

# 検証: τ = M @ ddq + c + g
tau_check = M @ ddq + c + g
print(f"τ = Mq̈ + c + g: {tau_check}")
print(f"Match: {np.allclose(result.tau, tau_check)}")
```

## 依存関係

- `numpy`
- `pymlg` - Lie群演算ライブラリ
- `pinocchio` (オプション) - クロス検証用

## 規約

- **Twist規約**: pymlg形式 `[ω, v]`（角速度、線速度）- Lynch & Park準拠
- **空間慣性規約**: pymlg形式 `[ω, v]` に対応する構造
- **DH規約**: Modified DH（Craig）、パラメータ順序 `[a, d, alpha]`

**注意**: 本パッケージはすべて `[ω, v]` 規約で統一されています。Pinocchio/Isaac Sim（`[v, ω]` 規約）との比較時は適切な変換が必要です。

## アルゴリズム概要

### Newton-Euler再帰アルゴリズム（RNEA）

```
初期化:
  V_0 = 0                    # 基底ツイスト
  V̇_0 = [0, 0, 0, -g]       # 重力を基底加速度として扱う（[ω, v]規約）

前方反復 (i = 1 to n):
  T_{i,i-1} = DH変換の逆
  Ad_{i,i-1} = SE3.adjoint(T_{i,i-1})
  V_i = Ad_{i,i-1} @ V_{i-1} + A_i * θ̇_i
  V̇_i = Ad_{i,i-1} @ V̇_{i-1} + [ad_{V_i}] @ A_i * θ̇_i + A_i * θ̈_i

後方反復 (i = n to 1):
  F_i = G_i @ V̇_i - [ad_{V_i}]^T @ G_i @ V_i + Ad^T_{i+1,i} @ F_{i+1}
  τ_i = A_i^T @ F_i
```

## 参考文献

- Lynch, K. M., & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press. Chapter 8.
