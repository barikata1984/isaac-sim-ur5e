# Inertial Parameter Identification - Implementation Log

実装の進捗と決定事項を記録するログ。
`implementation_plan.md` と対になるドキュメント。

---

## Phase 1: センサー統合 ✅ 完了

**実装日**: 2026-02-03
**ステータス**: 完了
**テスト**: 36件 全パス

### 1.1 実装したファイル

```
src/sensor/
├── __init__.py           # 公開API定義
├── data_types.py         # データ型クラス
├── data_buffer.py        # 時系列データ管理
└── contact_sensor.py     # センサーインターフェース

test/
├── conftest.py           # テストフィクスチャ
└── test_sensor.py        # ユニットテスト (36件)
```

### 1.2 実装内容の詳細

#### 1.2.1 data_types.py

| クラス | 説明 | 用途 |
|--------|------|------|
| `SensorData` | 1タイムステップの測定データ | データ収集時の基本単位 |
| `EstimationData` | 回帰行列Aと観測ベクトルy | 推定アルゴリズムへの入力 |
| `EstimationResult` | 推定結果 + 品質指標 | 推定結果の出力・評価 |
| `WrenchStamped` | タイムスタンプ付き力/トルク | センサー出力の表現 |

**設計決定**:
- `SensorData` は生の測定値を保持、`EstimationData` への変換は `DataBuffer` で実行
- `EstimationResult` にプロパティとして `mass`, `center_of_mass`, `inertia_matrix` を実装
  - `phi` ベクトルからの導出を自動化
  - `inertia_at_com` で重心座標系への変換も提供

#### 1.2.2 data_buffer.py

| メソッド | 機能 |
|----------|------|
| `add_sample()` | サンプル追加（FIFO、max_samples制限） |
| `get_stacked_data()` | 推定用 (A_stacked, y_stacked) 取得 |
| `compute_accelerations_from_velocities()` | 数値微分による加速度計算 |
| `save_to_file()` / `load_from_file()` | JSON形式での永続化 |
| `save_to_npz()` / `load_from_npz()` | NumPy形式（大規模データ向け） |
| `subsample()` | ダウンサンプリング |
| `get_time_range()` | 時間範囲抽出 |

**設計決定**:
- `kinematics` パッケージへの依存は `get_stacked_data()` 呼び出し時のみ
  - バッファ自体は kinematics なしで動作可能
- 加速度計算には scipy の `uniform_filter1d` を使用（オプション）

#### 1.2.3 contact_sensor.py

| クラス/関数 | 説明 |
|-------------|------|
| `WrenchSourceBase` | 抽象基底クラス（力/トルクソース） |
| `IsaacSimStateCollector` | Isaac Sim からの状態取得 |
| `InverseDynamicsWrenchSource` | 逆動力学による力/トルク計算 |
| `SimulatedForceSensor` | テスト用シミュレートセンサー |
| `compute_offset_compensation_matrix()` | オフセット補償行列（論文 Eq.7-8） |
| `apply_offset_compensation()` | オフセット適用（論文 Eq.9） |

**設計決定**:

1. **力/トルク取得の抽象化**
   - Isaac Sim では直接的な F/T センサーがないため、複数のソースに対応
   - `WrenchSourceBase` を継承することで将来の拡張に対応

2. **シミュレートセンサーの提供**
   - `SimulatedForceSensor`: 既知パラメータから理論的な力/トルクを計算
   - ノイズ追加機能付き（推定アルゴリズムのテストに使用）

3. **オフセット補償の実装**
   - 論文 Eq.7-9 に準拠
   - 単一行列・スタック行列の両方に対応

### 1.3 テスト結果

```
============================== 36 passed in 0.40s ==============================
```

| テストクラス | テスト数 | 内容 |
|--------------|----------|------|
| `TestSensorData` | 4 | データ型の作成・検証 |
| `TestEstimationData` | 2 | 回帰行列形式の検証 |
| `TestEstimationResult` | 7 | 推定結果プロパティの検証 |
| `TestWrenchStamped` | 2 | タイムスタンプ付きデータ |
| `TestDataBuffer` | 12 | バッファ操作・永続化 |
| `TestOffsetCompensation` | 5 | オフセット補償計算 |
| `TestSimulatedForceSensor` | 2 | シミュレートセンサー |
| `TestInverseDynamicsWrenchSource` | 2 | 逆動力学計算 |

### 1.4 Phase 1 の Open Questions への回答

| 質問 | 回答/対応 |
|------|----------|
| Isaac Sim Contact Sensor の使用方法 | 直接使用は複雑なため、`InverseDynamicsWrenchSource` で代替可能な設計に |
| センサーフレームと tool0 の関係 | `kinematics.compute_regressor()` が tool0 フレームで出力するため、そのまま使用 |
| サンプリングレート | `DataBuffer` で任意のレートに対応。`subsample()` でダウンサンプリング可能 |

### 1.5 Phase 2 への引き継ぎ事項

1. **推定用データの準備**
   - `DataBuffer.get_stacked_data(kinematics)` で (A, y) を取得
   - これを Phase 2 の推定アルゴリズムに渡す

2. **テストデータの生成**
   - `SimulatedForceSensor` を使用して既知パラメータからテストデータを生成
   - ノイズレベルを変えて推定精度を評価可能

3. **オフセット補償**
   - 推定前に `apply_offset_compensation()` を適用
   - `g_init` は初期姿勢での重力ベクトル

### 1.6 使用例

```python
from iparam_identification.sensor import (
    DataBuffer, SensorData, SimulatedForceSensor, EstimationResult
)
from kinematics import PinocchioKinematics

# === セットアップ ===
kin = PinocchioKinematics.for_ur5e()

# 既知パラメータ（テスト用）
phi_true = np.array([
    1.0,      # mass [kg]
    0.0,      # m*cx
    0.0,      # m*cy
    0.05,     # m*cz (CoM at 5cm in z)
    0.01,     # Ixx [kg·m²]
    0.0, 0.0, # Ixy, Ixz
    0.01,     # Iyy
    0.0,      # Iyz
    0.01,     # Izz
])

sensor = SimulatedForceSensor(kin, phi_true, noise_force_std=0.1)

# === データ収集 ===
buffer = DataBuffer(max_samples=5000)

for t in range(1000):
    # 軌道から q, dq, ddq を取得（trajectories パッケージ使用）
    q, dq, ddq = trajectory.get_value_at(t * dt)

    # 力/トルク測定（シミュレート）
    wrench = sensor.measure(q, dq, ddq, timestamp=t * dt)

    # バッファに追加
    buffer.add_sample(SensorData(
        timestamp=t * dt,
        q=q, dq=dq, ddq=ddq,
        force=wrench.force,
        torque=wrench.torque,
    ))

# === 推定用データ準備 ===
A, y = buffer.get_stacked_data(kin)
print(f"Data shape: A={A.shape}, y={y.shape}")
# Data shape: A=(6000, 10), y=(6000,)

# === データ保存 ===
buffer.save_to_npz("estimation_data.npz")

# === Phase 2 で使用 ===
# phi_hat = batch_total_least_squares(A, y)  # Phase 2 で実装
# result = EstimationResult(phi=phi_hat, ...)
```

---

## Phase 2: 推定アルゴリズム 🔄 未実装

**ステータス**: 未着手

### 予定実装内容

| ファイル | 優先度 | 内容 |
|----------|--------|------|
| `batch_ls.py` | 高 | バッチ最小二乗法（ベースライン） |
| `batch_tls.py` | 高 | バッチ全最小二乗法（メイン手法） |
| `rls.py` | 中 | 再帰最小二乗法 |
| `rtls.py` | 中 | 再帰全最小二乗法（論文の主要貢献） |
| `svd_update.py` | 中 | インクリメンタルSVD |

### Phase 1 からの入力

```python
# DataBuffer から取得
A, y = buffer.get_stacked_data(kinematics)

# A: (N*6, 10) - スタックされた回帰行列
# y: (N*6,)    - スタックされた観測ベクトル
```

### 期待される出力

```python
# EstimationResult (data_types.py で定義済み)
result = EstimationResult(
    phi=phi_hat,           # (10,) 推定パラメータ
    condition_number=κ,    # 条件数
    residual_norm=r,       # 残差ノルム
    n_samples=N,           # サンプル数
)

print(result)
# EstimationResult:
#   Mass: 1.0023 kg
#   Center of Mass: [0.0012, -0.0008, 0.0498] m
#   Inertia Matrix (sensor frame):
#     [0.010012, 0.000023, -0.000015]
#     [0.000023, 0.009987, 0.000018]
#     [-0.000015, 0.000018, 0.010003]
#   Condition Number: 15.23
#   Residual Norm: 0.023456
#   Samples: 1000
```

---

## Phase 3: 補助機能 🔄 未実装

**ステータス**: 未着手

### 予定実装内容

| ファイル | 内容 |
|----------|------|
| `excitation_trajectory.py` | 励起軌道設計 |
| `condition_optimizer.py` | 条件数最適化 |
| `jerk_limiter.py` | ジャーク制限 |

---

## 変更履歴

| 日付 | Phase | 内容 |
|------|-------|------|
| 2026-02-03 | 1 | センサー統合モジュール実装完了 |
| - | 2 | （予定） |
| - | 3 | （予定） |
