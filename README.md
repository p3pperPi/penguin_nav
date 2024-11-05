# penguin_nav

Navigation 2 を使用した経路計画行うROS 2パッケージです。

## 概要

`penguin_nav` は Navigation 2 を使用して経路計画を行うROS 2パッケージです。以下の要素で構成されています。

- Navigation 2 の経路計画の利用
    - `nav2_bringup` パッケージの `navigation_launch.py` をベースに、 `collision_monitor` による衝突回避を追加しています。
- Pose を TF に変換する機能
    - 自己位置推定が TF ではなく Pose を出す場合に対応するためです。
- ウェイポイントを設定するスクリプト
    - XY座標を読み込み `goThroughPoses` API を通して経路を渡しています。
    - Yaw 角は、`i` -> `i+1` のベクトルから計算しています。

## インストール

このリポジトリをROS 2ワークスペースにクローンし、依存パッケージをインストールしてからビルドします。

```shell
# リポジトリをクローン
cd ~/ros2_ws/src
git clone https://github.com/AbudoriLab-TC2024/penguin_nav.git

# 依存パッケージをインストール
sudo apt-get update
sudo apt-get install ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup python3-pandas

# ビルド
cd ~/ros2_ws
colcon build --packages-select penguin_nav
```

## 使用方法

Navigation 及びウェイポイント設定スクリプトを起動します。

**ターミナル1**

```shell
source install/setup.sh

# Navigation + pose_to_tf
ros2 launch penguin_nav nav.launch.xml
```

**ターミナル2**

```shell
source install/setup.sh

# ウェイポイント設定スクリプト
ros2 run penguin_nav follow_path.py -- <path/to/waypoints.csv>
```

## パラメータ

### nav.launch.xml

以下は `pose_to_tf` のパラメータです。launchファイルに直接ハードコーディングされていることにご注意ください。

| パラメータ名 | 型 | 説明 | デフォルト値 |
|:-|:-:|:-|:-|
| `child_frame_id` | `string` | Pose を TF に変換する際の子フレームID。親フレームは Pose の Header を継承します | `base_link` |

[config/nav2_params.yaml](./config/nav2_params.yaml) が Navigation 2 のパラメータです。各設定は [Configuration Guide](https://docs.nav2.org/configuration/index.html) に従います。



### follow_path.py

[penguin_nav/follow_path.py](./penguin_nav/follow_path.py) に座標の列名がハードコーディングされているため、必要に応じて修正してください。

## トピック

- 入力:
    - `/scan` (`sensor_msgs/msg/LaserScan`) - 2D LiDAR 形式の点群データを受け取ります。Glocal costmap、Local costmap、及び collision monitor に使用されます。
    - `/pcl_pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`) - ロボットの自己位置を受け取ります。
- 出力:
    - `/cmd_vel` (`geometry_msgs/msg/Twist`) - 制御コマンドです。


## その他

### Quick fix and DDS issue with Nav2

DDSの問題により上手く動作しないかもしれないです。その場合は[こちら](https://roboticsbackend.com/ros2-nav2-tutorial/#Quick_fix_and_DDS_issue_with_Nav2) を参考に DDS を cyclonedds に変更してください。
