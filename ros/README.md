# misa_msgs — misa-run の ROS 2 インタフェース

速度は `geometry_msgs/Twist` の `cmd_vel` で受け、**それ以外は項目ごとの
サービス**で受ける。Twist は速度しか運べないため。

| 口 | 型 | 中身 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度（best-effort, depth 1） |
| `~/set_mode` | `misa_msgs/SetMode` | 脱力 / 初期姿勢 / 歩行 |
| `~/set_gait` | `misa_msgs/SetGait` | Crawl / Walk / Trot |
| `~/play_pose` | `misa_msgs/PlayPose` | 再生を 1 回だけ起こす |
| `~/set_body_attitude` | `misa_msgs/SetBodyAttitude` | roll / pitch / yaw |
| `~/set_height` | `misa_msgs/SetHeight` | 立ち高さの差分 |

## 設計上の約束

**`cmd_vel` が来なくなったら速度だけ 0 にし、モードは変えない。** pub/sub
では「publisher が黙っている」が正常な状態にもなりうるので、時間切れを
切断と同じには扱えない。荷重がかかった四足を勝手に脱力させると崩れるので、
止めたいなら `set_mode` を明示的に呼ぶこと（時間切れの長さは
プロファイルの `state_timeout_ms`）。

**起動直後は脱力。** ROS が繋がった瞬間に立ち上がらない。

**`set_gait` の `ok = true` は「受け取った」であって「切り替わった」では
ない。** 歩容の切り替えは遊脚が無いときだけ成立する（踏み替えが飛ぶため）。

**`cmd_vel` は best-effort・depth 1。** 制御ループで信頼性を求めると再送で
遅延が跳ねる。古い指令が遅れて届くほうが有害。reliable な publisher から
best-effort な subscriber は繋がるので、既定 QoS の相手とも噛み合う。

## ビルド

```sh
source /opt/ros/jazzy/setup.bash
cd ros && colcon build --packages-select misa_msgs && cd ..

export AMENT_PREFIX_PATH=$PWD/ros/install/misa_msgs:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=$PWD/ros/install/misa_msgs/lib:$LD_LIBRARY_PATH
# ros2 の CLI から型を触るときだけ
export PYTHONPATH=$PWD/ros/install/misa_msgs/lib/python3.12/site-packages:$PYTHONPATH

cargo build --release --features ros2          # 実機
cargo build --release --features sim,ros2      # MuJoCo で試す
```

`r2r` は crates.io のものなので ros2_rust のオーバーレイは要らない。ただし
**独自インタフェースなので `AMENT_PREFIX_PATH` に `misa_msgs` が要る**
（`std_msgs` だけで済む `go2-gait-runner` より 1 段重い）。

## 動かしてみる

```sh
misa-run sim --robot robots/namiashi.toml --pilot ros2 --gait trot --secs 0

ros2 service call /misa_run/set_gait misa_msgs/srv/SetGait '{gait: 2}'
ros2 service call /misa_run/set_mode misa_msgs/srv/SetMode '{mode: 2}'
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.12}}'
```

`cmd_vel` を止めると、その場で立ったまま停まる（モードは歩行のまま）。
