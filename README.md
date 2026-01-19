# Isaac Sim UR5e ROS2 Control

NVIDIA Isaac Sim を使用した UR5e マニピュレータの ROS2 制御プロジェクトです。

## 概要

このプロジェクトでは、Isaac Sim のシミュレーション環境内で Universal Robots UR5e アームを ROS2 経由で制御できます。ROS2 トピックを通じてターゲット位置を送信し、逆運動学（IK）を使用してアームを移動させます。

## 機能

- **ROS2 トピック経由のアーム制御**: `/target_position` トピックにターゲット座標を送信してアームを制御
- **複数のマニピュレータ対応**: UR3, UR3e, UR5, UR5e, UR10, UR10e, UR16e をサポート
- **逆運動学（IK）**: Lula IK ソルバーによる正確な位置制御
- **headless モード対応**: GUI なしでのシミュレーション実行が可能

## 必要条件

- Docker と Docker Compose
- NVIDIA GPU（RTX シリーズ推奨）
- NVIDIA Container Toolkit

## セットアップ

### 1. コンテナのビルドと起動

```bash
cd docker
docker-compose up -d
docker-compose exec isaac-sim bash
```

### 2. ワークスペースへ移動

```bash
cd /workspaces/isaac-sim-ur5e
```

## 使い方

### ROS2 Launch ファイルを使用（推奨）

**ターミナル 1: Isaac Sim 制御を起動**

```bash
ros2 launch launch/ur5e_control.launch.py
```

**ターミナル 2: ターゲット位置を送信**

```bash
ros2 topic pub /target_position geometry_msgs/Point "{x: 0.4, y: 0.4, z: 0.5}"
```

### オプション

```bash
# 別のマニピュレータを使用
ros2 launch launch/ur5e_control.launch.py manipulator:=ur10e

# headless モードで実行
ros2 launch launch/ur5e_control.launch.py headless:=true
```

### 直接実行

```bash
# Isaac Sim ROS2 制御
./scripts/run_isaac_ros2.sh scripts/ros2_control.py --manipulator ur5e

# スタンドアロン制御（ROS2 不使用）
python scripts/simple_control.py --manipulator ur5e
```

## プロジェクト構成

```
isaac-sim-ur5e/
├── docker/
│   ├── Dockerfile          # Isaac Sim + ROS2 Jazzy コンテナ
│   └── docker-compose.yml  # Docker Compose 設定
├── launch/
│   ├── ur5e_control.launch.py       # メイン launch ファイル
│   └── target_publisher.launch.py   # ターゲットパブリッシャー
├── scripts/
│   ├── run_isaac_ros2.sh   # ROS2 環境分離ラッパー
│   ├── ros2_control.py     # ROS2 ベースのアーム制御
│   ├── simple_control.py   # スタンドアロン制御
│   └── target_publisher.py # ターゲット位置パブリッシャー
└── src/
    └── env_loader.py       # マニピュレータ環境ローダー
```

## 対応マニピュレータ

| マニピュレータ | IK サポート |
|---------------|-------------|
| UR3           | ✅          |
| UR3e          | ✅          |
| UR5           | ✅          |
| UR5e          | ✅          |
| UR10          | ✅          |
| UR10e         | ✅          |
| UR16e         | ✅          |

## ROS2 トピック

| トピック名 | メッセージ型 | 説明 |
|-----------|-------------|------|
| `/target_position` | `geometry_msgs/Point` | ターゲット位置 (x, y, z) |

## トラブルシューティング

### GUI が表示されない

ホストマシンで X11 認証を許可してください：

```bash
xhost +local:root
```

### root ユーザーエラー

環境変数 `OMNI_KIT_ALLOW_ROOT=1` が設定されていることを確認してください（Dockerfile で自動設定済み）。

### ROS2 ライブラリ競合

`ros2 launch` 経由で実行する場合、`run_isaac_ros2.sh` ラッパーが自動的にコンテナの ROS2 環境をクリアし、Isaac Sim の内蔵 ROS2 ブリッジを使用します。

## ライセンス

このプロジェクトは開発・研究目的で作成されています。Isaac Sim の使用には NVIDIA のライセンス条項が適用されます。
