# misa-runner

四脚ロボット **namiashi**（LKMTech V3 モータ ×12 + 腕の RC サーボ ×1）を
プロポ（Futaba S.BUS）で操縦する実機アプリ。

`go2-gait-runner` が Unitree Go2 に対して果たしている役割の namiashi 版で、
歩容そのもの（`quadruped-gait`）やモデル（`misarta` / `.misa`）には手を入れず、
**実機の配線・座標変換・モード遷移・操縦入力**だけをここに持つ。

## できること

| | |
|---|---|
| 移動 | 前進後退 / 左右旋回 / 左右真横移動（スティック 3 軸） |
| 歩容 | Crawl / Walk / Trot をプロポのスイッチで切替 |
| 姿勢 | 胴体高さをスティックで上下 |
| 演出 | `.misa` のポーズ / シーケンス再生（挨拶）、チキンヘッド※ |
| 可視化 | `--viz` で articara に実時間描画（Zenoh） |
| 安全 | 受信断・フェイルセーフで速度 0・その場起立、Ctrl-C で脱力 |

※ チキンヘッドと挨拶の腕動作は、腕サーボをアプリから駆動できる構成でのみ
有効。現状は受信機直結なので腕は**観測のみ**（[未確定・既知の制限](#未確定既知の制限)）。

**引き継ぎ・設計判断・SBC 移行の手順**は [`doc/handover.md`](doc/handover.md)、
**配線とモータ id の対応表**は [`doc/motor_map.md`](doc/motor_map.md)。

## ハードウェア

`nm_board/ch348` rev2 基板（CH348L, USB-C → 8ch UART）が実機 I/F。
UART の割り当ては `spec_rev2_0_0_asbuilt.md` §4 のとおり:

| UART | 役割 | I/F |
|---|---|---|
| 0–3 | LEG1–4 = FL / FR / RL / RR（各 3 モータ） | RS485 |
| 4 | ARMA（腕サーボ） | RS485 / TTL 切替 |
| 5 | IMU（WitMotion IWT603） | TTL |
| 6 | S.BUS（受信専用） | 反転 TTL |
| 7 | ARMB（予備） | RS485 / TTL 切替 |

`/dev/ttyCH9344USB*` の番号は列挙順で決まるので当てにせず、ch9344 の
`GETUARTINDEX` ioctl で**物理 UART 番号**を引いて対応付ける。

## ビルド

依存はすべて GitHub の**公開**リポジトリへの git 依存なので、新しいマシン
（SBC など）でも兄弟チェックアウトは要らない。SSH 鍵も認証情報も要らない。

```sh
git clone --recurse-submodules https://github.com/takarakasai/misa-runner.git
cd misa-runner
cargo build --release      # 依存は cargo が GitHub から取ってくる
cargo test
```

**`--recurse-submodules` を忘れないこと。** `models/namiashi/` は
[`namiashi_description`](https://github.com/takarakasai/namiashi_description) の
submodule で、モデル（`.misa`）と meshes がそこにある。忘れると `models/namiashi/` が
空のままで `check` が「読み込みに失敗」になる。後から入れるなら:

```sh
git submodule update --init
```

Zenoh（`--viz`）が要らない環境ではこちらのほうが軽い（20 MB / ビルドも速い）:

```sh
cargo build --release --no-default-features
```

**ブリッジ越しの機体（keel など）を実機でビルドするなら、その機体の
リポジトリを見ること。** 手順も profile も実行ファイルもあちらにある。

### 兄弟クレートも一緒に直したいとき

`misa-actuator` や `sbus` を misa-runner と併行して直す場合だけ、
path override を張る:

```sh
./scripts/dev-siblings.sh          # 兄弟を clone / 更新し .cargo/config.toml に [patch] を書く
./scripts/dev-siblings.sh --off    # git 依存へ戻す
```

**これを実行していない間、ローカルの兄弟チェックアウトへの変更はビルドに
反映されない。** cargo は `Cargo.lock` が指す GitHub の revision を見る。
「直したのに変わらない」の原因はたいていこれ。`cargo tree -p misa-hal`
で解決先（ローカルパスか git URL か）が確認できる。

`.cargo/config.toml` は追跡していない（人ごと・マシンごとに違うため）。

### feature

| feature | 何が入るか | 要るもの |
|---|---|---|
| `viz`（既定） | Zenoh でライブ配信 | — |
| `sim` | MuJoCo で動力学込みに回す | MuJoCo 3.8 の共有ライブラリ |
| `render` | MuJoCo の絵を PNG に落とす | EGL |
| `ros2` | **ROS 2 から操縦**（cmd_vel + サービス 5 本） | 標準 msg と `ros/misa_msgs`。**機体に依らない** |

**機体固有の繋ぎ方はここに入れない。** ブリッジ越しの機体（相手の独自
メッセージに束縛されるもの）は、**別リポジトリのクレートが [`Backend`] を
実装して差す。** 入れてしまうと、cmd_vel で操縦したいだけの機体でも
向こうの colcon ws が要るようになる。

```rust
fn main() -> std::process::ExitCode {
    misa_runner::main_with(&[&MyBridge])
}
```

`Backend` は「このプロファイルを扱えるなら `Plant` と `Pilot` を作る」
だけの口。サブコマンドも制御則も misa-runner のものがそのまま出る。
実例は keel-runner（`misa-plant-ksm` + `keel-run`）。

## 使い方

**立ち上げは上から順に。** 各段が通ってから次へ行くと、詰まった場所が常に
1 段で分かる。

```sh
misa-run check                 # 設定とモデルの検証（実機に触れない）
misa-run ports                 # CH348 のポート一覧（何も開かない）
misa-run dump --gait trot      # 歩容を実機なしで再生し可動域を検証
misa-run imu  --secs 10        # IMU 受信の確認
misa-run sbus --secs 10        # プロポ入力と解釈結果の確認
misa-run legs --secs 10        # 脚バスの状態と実効周期（**指令は送らない**）
misa-run calib scan            # 応答するモータ id を数える（指令は送らない）
misa-run run  --robot robots/namiashi.toml
misa-run run  --robot robots/namiashi.toml --record run.rec   # 毎周期を記録
```

設定は 1 枚の TOML（`robots/namiashi.toml`）。雛形は
`misa-run config --out robots/namiashi.toml` で生成できる。

### プロポ割り当て

**組み込みの既定（`--config` を付けずに起動したとき）と、同梱の
`robots/namiashi.toml` は別物。** スイッチ 4 本の並びが違う。実機は後者で
動かすので、迷ったら `misa-run check --robot …` の表示を正とすること。

| CH | 組み込み既定 | `robots/namiashi.toml` |
|---|---|---|
| 1 | 左右真横（エルロン、反転） | 同左（反転なし） |
| 2 | 前後（エレベータ） | 同左 |
| 3 | 胴体高さ（スロットル） | 同左 |
| 4 | 旋回（ラダー、反転） | 同左（反転なし） |
| 5 | モード 3 段: 脱力 / 初期姿勢 / 歩行 | 同左 |
| 6 | 歩容 3 段: Crawl / Walk / Trot | 同左 |
| 7 | ポーズ再生（立ち上がりで 1 回） | **腕サーボ**（観測のみ） |
| 8 | チキンヘッド / 姿勢モード | 同左 |
| 9 | 腕サーボ（観測のみ） | **ポーズ再生**（立ち上がりで 1 回） |
| 10 | 振る足の選択（下 = `greeting` / 上 = `greeting_alt`） | 同左 |

腕サーボは受信機が直接駆動し、アプリは角度を**観測するだけ**。CH8 は
`gait.body_attitude_max_rad > 0` なら姿勢モードのスイッチになる（[下表](#プロポ割り当て-1)）。

チャンネル・エンドポイント・不感帯・エクスポは全部 `[teleop]` で変更できる。
`misa-run sbus` を見ながら合わせるのが早い。

## 実機の立ち上げ

機体ごとに手順書がある。**上から 1 段階ずつ、合格条件を満たしてから進む。**

| 機体 | 手順書 | 特徴 |
|---|---|---|
| namiashi | [`doc/bringup_checklist.md`](doc/bringup_checklist.md) | シリアル直結。校正（可動域・符号・ゼロ点）をこちらで採る |
| keel | **keel-runner リポジトリ**の `doc/` | STM ブリッジ越し。手順も profile も実行ファイル（`keel-run`）もあちらにある |

## 校正（実機に通電したら最初にやること）

起動直後の設定は `sign = +1` / `zero_pose_rad = 0` / 可動域は URDF 値、という
**推測**でしかない。1 軸でも符号が逆なら起立の瞬間に自壊する。`calib` は
その 3 つを 1 軸ずつ実機で確定して設定へ書き戻す。

```sh
# 1) 誰が居るか（指令は送らない）
misa-run calib scan --max-id 8

# 2) 可動域を実測（脱力させ、手で端から端まで動かす）
misa-run calib range --leg FL --joint thigh --write robots/namiashi.toml

# 3) 符号を確定（1 軸だけ 5° 動かし、モデルの + 方向か答える）
misa-run calib move  --leg FL --joint thigh --write robots/namiashi.toml

#    2) と 3) を 12 軸ぶん繰り返す

# 4) ゼロ点（指定した姿勢で保持してからゼロ出し、その姿勢角を記録）
misa-run calib zero --pose constrain --write robots/namiashi.toml
```

安全のための約束:

- **1 度に 1 軸しか投入しない。** `move` は対象軸だけ `EnableJoint` し、
  終わったら必ず `DisableJoint` で戻す。残り 2 軸は最後まで脱力のまま
- **開くポートも 1 本だけ。** `--leg FL` なら UART0 しか掴まない
- **既定の振り幅は 5°、速度 0.3 rad/s。** 取り違えていても壊れない大きさ
- **`--write` を明示したときだけ**設定ファイルへ書き戻す

`--write` の書き戻しは `AppConfig` から TOML を作り直すので、**手書きの
コメントは消える**。`misa-run config --out` で生成したファイルを校正で
上書きしていく運用を前提にしている。

## 動力学で確かめる（MuJoCo）

`dump` が「指令がそのまま実現したら関節角はどうなるか」なのに対し、`sim` は
**重力と接触の中で本当に立っていられるか**を見る。articara の MuJoCo を
`Plant` の実装として使うので、実機と同じ制御則・同じ `exchange` を通る。

```sh
export MUJOCO_DYNAMIC_LINK_DIR=$HOME/.mujoco/mujoco-3.8.0/lib
export LD_LIBRARY_PATH=$MUJOCO_DYNAMIC_LINK_DIR
cargo run --release --features sim -- \
    sim --robot robots/namiashi.toml --gait trot --vx 0.15 --secs 8
```

### プロポで操縦しながら見る

`--pilot sbus` で**実物の送信機から操縦できる**。要るのは受信機と CH348 基板
だけで、脚もモータも要らない。チャンネル割り当て・不感帯・エクスポという
間違えやすいところを、実機を壊さずに確かめられる。

```sh
# 1) MuJoCo 側（プロポで操縦 + articara へ配信）
cargo run --release --features sim -- sim --robot robots/namiashi.toml \
    --pilot sbus --secs 0 --viz --viz-endpoint tcp/127.0.0.1:7447

# 2) 別端末で articara（Live gait feed で購読）
cd ../articara && cargo run --release --features viz -- \
    --model ../namiashi_description/namiashi.misa
```

**`--viz` か `--pilot sbus` を付けると自動で実時間になる。** 付けないと
MuJoCo を全力で回すので、10 秒ぶんが 1 秒で終わって目でも手でも追えない。
`--realtime` で明示もできる。

配信は **planned（指令）と measured（MuJoCo の実測）の両方**。受け側が
ゴーストで重ねて描くので、追従できていない軸が目で分かる。

```text
t[s]   状態         胴体 z[m]  roll   pitch  接地
 3.00  立ち姿勢へ           0.213  -0.001 +0.000  ■■■■
 4.50  歩容              0.209  +0.005 -0.004  □■■□
 6.00  歩容              0.212  -0.033 -0.022  ■□■■
終端 胴体位置 (+0.601, +0.101, +0.211)  最低高さ 0.033 m
転倒なし
```

胴体が 1 rad 以上傾いたら転倒として終了コード 1 を返す。`--record` も付く。

**MuJoCo は既定のビルドに入っていない。** `misa-plant-mujoco` はワークスペースの
メンバから外してあるので、MuJoCo を入れていない環境でも `cargo test` は通る。

見られないもの: RS485 の往復遅れ・バスのジッタ・モータの一次遅れ・受信断。
**脱力も再現されない**（位置アクチュエータにその概念が無いので `Idle` の軸は
その場で保持される）。ここを通ったから実機が通るとは考えないこと。

### keel を MuJoCo で動かす（ROS 2 から操縦して articara で見る）

2 台目の機体をひととおり動かす手順。**実機はいらない。**

環境（毎回いる）:

```sh
source /opt/ros/jazzy/setup.bash
export AMENT_PREFIX_PATH=$PWD/ros/install/misa_msgs:$PWD/ref/ksm_mvp_real_ws/install:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=$PWD/ros/install/misa_msgs/lib:$PWD/ref/ksm_mvp_real_ws/install/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$PWD/ros/install/misa_msgs/lib/python3.12/site-packages:$PYTHONPATH
export MUJOCO_DYNAMIC_LINK_DIR=$HOME/.mujoco/mujoco-3.8.0/lib
export LD_LIBRARY_PATH=$MUJOCO_DYNAMIC_LINK_DIR:$LD_LIBRARY_PATH
```

初回だけ `cd ros && colcon build --packages-select misa_msgs && cd ..`。

```sh
# 1) runner（MuJoCo + 配信 + ROS 操縦）。--secs 0 で Ctrl-C まで
cargo run --release --features sim,ros2 -- \
    sim --robot robots/keel.toml --pilot ros2 --gait trot --secs 0 \
        --kp 1200 --kv 30 --timestep 0.0005 --base-height 0.42 \
        --viz --viz-endpoint tcp/127.0.0.1:7447

# 2) 別端末で articara。Live gait feed に tcp/127.0.0.1:7447 を入れて Start
cd ../articara && cargo run --release --features viz -- \
    --model ../keel/model/proto2_asset/urdf/mvp_v2.misa

# 3) さらに別端末で操縦。**トピックは名前空間つき**
ros2 service call /keel/misa_run/set_gait misa_msgs/srv/SetGait '{gait: 2}'
ros2 service call /keel/misa_run/set_mode misa_msgs/srv/SetMode '{mode: 2}'
ros2 topic pub -r 20 /keel/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.12}}'
```

`cmd_vel` を止めると 300 ms 後に速度が 0 に落ちて、その場で立つ
（**モードは変えない**）。

胴体をその場で傾ける（足は接地したまま）:

```sh
ros2 service call /keel/misa_run/set_body_attitude \
    misa_msgs/srv/SetBodyAttitude '{roll: 0.0, pitch: 0.3, yaw: 0.0}'
```

**上限は「合成量」で、軸ごとではない**（`gait.body_attitude_max_rad`、keel は
0.20）。超えた要求は向きを保ったまま丸めて、応答にそう書く。0 のときは
`ok: false` で返す — **「受け付けた」と言っておいて何も起きないのが
いちばん困る**ので。

**`--kp 1200 --kv 30 --timestep 0.0005` が要る。** 既定の 2 ms では減衰を
上げられず（articara の PD は明示的なので `kv < 2·I/dt`）、kp だけ上げると
進む量が 64% → 135% → 166% → 45% と暴れる。刻みを 0.5 ms にして初めて収束し、
指令の 94〜97% が出る。**実機のゲインとは別物**で、あちらは STM ブリッジが
MIT モードで持つ。

うまくいっているかは終了時の 3 行で見る:

| 見るもの | 良い状態 |
|---|---|
| 追従誤差 | 膝で 0.07 rad 以下。**歩幅（0.05 m 前後）と比べて意味を持つ** |
| 接地中の足の滑り | 0.03 m/s 前後。指令速度と同じ桁なら歩容は成立していない |
| 遊脚で上がった高さ | `swing_height_m` に届いていること |

**安全ゲートは実機の指令に掛かっている**（2026-09-02 に配線。それまでは影で
回して出力を捨てていた）。可動域・目標の変化率・観測の古さをここで丸め、
何をしたかを `SafetyVerdict` として記録に残す。**記録に載るのは丸めたあとの
指令**で、実機へ出したものと一致する。

`dump` が「歩容が要求する目標の変化率」と「ゲートの上限」を毎回突き合わせる。
**超えていたら実機ではゲートが歩容を鈍らせる。** 上限は
`hardware.max_target_rate_rad_s` とモデルの定格速度の厳しいほう。

**録画は `--cam-fixed` を付ける。** 地面が無地なので、既定の追従カメラだと
歩いても止まって見える。`--cam-x` / `--cam-y` を移動の中点に置くと、始点も
終点も画面に入る。指令ごとの見本は `videos/`（`README.txt` に条件と結果）。

**車輪 4 軸は articara に出ない。** `GaitVizFrame` が脚 12 関節しか運ばない
ため。MuJoCo の中では存在していて、指令は出していない（歩容の仕事ではない
ので脱力のまま）。

台本だけで回すなら `--pilot ros2` を外して `--vx 0.15` などを渡す。ROS 2 の
口が要らないので `--features sim` だけでビルドできる。

### MuJoCo の絵を動画にする

`--viz`（articara へ Zenoh）は**関節角だけ**で、接地も地面も出ない。動きを
動画で残すなら、MuJoCo をオフスクリーン（EGL）で描いて PNG を並べ、ffmpeg で
まとめる。GUI もディスプレイも要らない。

```sh
cargo run --release --features render -- sim --robot robots/keel.toml \
    --gait trot --vx 0.12 --secs 9 --kp 200 --kv 2.0 --base-height 0.35 \
    --video /tmp/keel --cam-dist 2.2 --cam-el -18 --cam-az 120

ffmpeg -framerate 30 -i /tmp/keel/frame_%05d.png \
    -vf "eq=brightness=0.28:contrast=1.5" -c:v libx264 -pix_fmt yuv420p out.mp4
```

**明るさ補正は要る。** articara の MJCF エクスポータは光源を出さないので、
MuJoCo の既定ヘッドライトだけになって暗い。

**既定は 640×480。** MuJoCo のオフスクリーンバッファの既定がこれで、超えると
描かれた領域だけが左上に寄って黒帯になる。大きくするにはモデルの
`<visual><global offwidth/offheight>` が要るが、エクスポータはそれを出さない。

カメラは `--cam-az`（方位、90 で真横・180 で真後ろ）/ `--cam-el` /
`--cam-dist` / `--cam-z`。**機体の大きさで変える** — keel は 2.2 前後、
namiashi は 1.1 前後。

`--features render` は**ローカルの articara を見る**（`.cargo/config.toml` の
`[patch]`）。キャッシュしている git revision には `render` feature も
`MujocoSim::mj_model` / `mj_data_mut` も無いため。articara を更新すれば外せる。

#### チキンヘッドを見る

平地を歩くだけでは胴体がほとんど傾かず、ヘッドは 1° しか動かない。
**胴体を傾けて撮る**と分かる（`gait.body_attitude_max_rad > 0` が要る）。

```sh
cargo run --release --features render -- sim --robot robots/namiashi.toml \
    --gait trot --vx 0.08 --tilt-pitch 0.35 --chicken --secs 9 \
    --cam-az 90 --cam-dist 1.1 --video /tmp/ch_on
```

`--chicken` の有無で `arm_pitch_joint` に 20° の差が出る（ON は頭が水平の
まま、OFF は胴体と一緒に下を向く）。**シムでしか見られない** — 実機の
namiashi は腕が受信機直結でアプリから駆動できないため。

## 記録と再生

`--record PATH` を付けると、毎周期の **意図・観測・指令・安全判定** を 1 本の
ファイルへ落とす。書き込みは別スレッドで、詰まったら捨てて数えるので、
**制御周期は待たされない**（取りこぼした数は終了時に出る）。

```sh
misa-run run  --robot robots/namiashi.toml --record before.rec
misa-run replay before.rec                 # 要約（周期数・丸めが入った周期）
misa-run replay before.rec after.rec       # 指令を差分する
```

**差分に許容差は無い。** 見たいのは「値が近いか」ではなく「同じ計算をしたか」
なので、1 bit でも違えば食い違いとして軸名・フィールド・値を出し、終了コード 1
を返す。制御ループを触る改修は、これで挙動を変えていないことを確かめられる。

**実機なしでも録れる。** `dump` にも `--record` があるので、歩容と状態機械の
回帰試験は実機を触らずに回る:

```sh
misa-run dump --robot robots/namiashi.toml --gait trot --vx 0.1 --secs 10 --record a.rec
```

記録は 200 Hz × 13 軸で約 190 KB/s（3 秒で 557 KB）。診断のために録るもので、
常時走らせる想定ではない。

## 安全側の作り

| 何 | どこ | 効き方 |
|---|---|---|
| 目標角のスルーレート制限 | `legs.max_target_rate_rad_s`（既定 3 rad/s） | 歩容切替や IK クランプで目標が跳んでも、脚の飛び出しにならない |
| 軸の速度上限 | `legs.default_max_speed_rad_s`（既定 8 rad/s） | モータ側が守る「軸が何 rad/s で回るか」 |
| 可動域クランプ | `motors[].min_rad` / `max_rad` | HAL が指令を必ず内側へ丸める |
| 異常ビット監視 | `legs.status_interval_ms`（既定 1 s） | 過電流・過熱・ストールを検出して ERROR ログ |
| 受信断 | `control.teleop_timeout_ms` | 速度 0・その場起立 |
| 傾きの監視 | `control.max_tilt_rad`（省略時は姿勢指令の上限 + 0.3 rad） | 鉛直から傾きすぎたら ERROR ログ（**脱力はしない**） |
| Ctrl-C / SIGTERM | — | 全軸を脱力してから終了 |

**スルーレート制限と軸の速度上限は別物。** 前者は「目標が何 rad/s で動くか」、
後者は「軸が何 rad/s で回るか」。目標が 1.5 rad 跳んだとき、後者だけだと
8 rad/s で追いに行ってしまう。

**異常ビットで自動脱力はしない。** 立っている四足を脱力させると倒れるので、
ERROR を毎回はっきり出したうえで、止めるかどうかは operator がモード
スイッチで決める。

**傾きの監視も同じ扱い**（報告するだけ）。**接地センサが無い機体では、転倒に
近づいたことを知る手がかりが IMU の姿勢角しかない。** 測るのは鉛直からの傾き
（`cos θ = cos roll · cos pitch`）で、roll と pitch の和ではない — 足すと斜めに
傾いたときに過大評価になる。しきい値を固定値にせず
`gait.body_attitude_max_rad + 0.3 rad`（最低 0.5 rad）から導くのは、**意図して
傾ける量が機体ごとに違う**から（namiashi 0.6 → 0.9 rad、keel 0.20 → 0.50 rad）。
指令どおり傾けただけで報告が出ると、本当の転倒と区別が付かなくなる。
報告は立ち上がりだけで、戻ったら INFO を 1 行出す。`replay` が「傾き超過」の
周期数と最大値を数える。

## 動作モード

```text
  脱力 ──(CH5: 中/上)──▶ 初期姿勢へ ──▶ 初期姿勢 ──(CH5: 上)──▶ 立ち姿勢へ ──▶ 歩容
   ▲                                       ▲                                    │
   │                                       └──────────(CH5: 中)─────────────────┤
   └────────────────────────(CH5: 下 = 脱力)─────────────────────────────────────┘
                                        歩容 ──(CH9)──▶ ポーズ再生 ──▶ 立ち姿勢へ
```

### プロポ割り当て

| CH | 役割 |
|---|---|
| CH1 / CH2 / CH4 | 左右 (vy) / 前後 (vx) / 旋回 (wz) |
| CH3 | 高さ（**歩容側が未対応で効かない**） |
| CH5 | 脱力 / 初期姿勢 / 歩容 |
| CH6 | 歩容種別 (Crawl / Walk / Trot) |
| **CH7** | **腕（観測のみ。アプリからは駆動しない）** |
| CH8 | 姿勢モード（下表） |
| **CH9** | **ポーズ再生（手を振る）**。**受け付けるのは歩容中 (CH5 上段) だけ** |
| **CH10** | **振る足の選択**（下 = `greeting` / 上 = `greeting_alt`） |

| CH8 | CH1 | CH3 | CH4 |
|---|---|---|---|
| OFF | 横移動 (vy) | 高さ（**歩容側が未対応で効かない**） | 旋回 (wz) |
| **ON** | **胴体ロール** | **胴体ピッチ** | **胴体ヨー** |

**ON の間は横移動・高さ・旋回が 0 になる。** 同じスティックを 2 つの意味で
使うため。前後 (CH2) だけは残るので、傾けたまま歩ける。

**胴体姿勢は歩容が出した足先位置を回してから IK を解き直して作る。**
足は接地したまま胴体だけ傾く。歩容側に姿勢制御は無い
（`set_body_attitude_observed` は FullCentroidal 専用で既定の Champ では
no-op）ので、namiashi 側で足している。

**既定は無効**（`gait.body_attitude_max_rad = 0`）。上げるほど脚の可動域を
食うので、`dump --tilt-roll R --tilt-pitch P` で先に当たりを取ること。
実測の可動域は軸で違う:

| 軸 | 可動域内の上限 |
|---|---|
| ロール | 0.65 rad (37°) |
| ピッチ | 0.70 rad (40°) 以上 |
| **ヨー** | **1.2 rad (69°) 以上** — hip の横可動域が広いため |
| ロール + ピッチ同時 | 合成量で頭打ちにしてあるので上限どおり |

`body_attitude_max_rad` は 3 軸に共通で掛かるので、**ヨーだけ大きく振りたい
場合は上限がロール側に律速される**。必要なら軸ごとの上限に分けられる。

| CH5 | 状態 |
|---|---|
| 下 | 脱力 |
| **中** | **初期姿勢で保持**（`control.start_pose`）。試合はこの姿勢で合図を待つ |
| 上 | 歩容（スティック中立なら立ち姿勢で静止） |

- **脱力中の目標角は実測角**。起立に移った瞬間に 0 rad へ飛ばない。
- **ポーズ再生は CH5 を上段に置いたまま最後まで通る**（`wave_fr` / `wave_fl`
  は 3.8 s）。**振る前に胴体を機首上げ 15°・高さ 220 mm へ起こし、前足先を
  地上 180 mm まで上げる**（`ready` への 0.8 s で姿勢が大きく変わる）。
  3 本足で立つので、実機では周囲を空けて試すこと。途中で止めたいときは **CH5 を中段へ戻す**と立ち姿勢を経て初期
  姿勢へ帰る。かつては「歩行要求で中断」にしており、再生に入れるのが CH5
  上段 = 歩行要求のときだけである以上、**トリガした次の周期で必ず打ち切られ、
  手を振る前に立ち姿勢へ戻っていた**。
- **歩容の切り替えは「遊脚が無いとき」だけ**受け付ける。脱力中・初期姿勢・
  遷移中に加えて、**歩容中でも速度 0 で立っていれば切り替わる**（速度 0 の
  歩容は全脚接地・位相凍結なので、差し替えても飛ぶ遊脚が無い）。
  **歩いている最中は切り替わらない** — 踏み替えが飛ぶため。
- **受信断のフェイルセーフはモードを変えない。速度だけ 0 にする。**
  脱力中に切れたら脱力のまま、初期姿勢なら初期姿勢のまま、歩行中なら
  **その場で立ったまま**保持する。かつて一律「起立」を返しており、
  **脱力中に受信が切れると立ち上がっていた**。逆に `Stand` へ丸めるのも
  誤りで、CH5 中段が初期姿勢保持になった今は**歩行中の受信断で
  しゃがみ込む**ことになる。
- **速度指令は `gait.velocity_ramp_s`（既定 0.5 s）で鈍らせる。** 歩容は
  速度がちょうど 0 になった瞬間に全脚を接地へスナップさせる
  （`vel.is_zero()` は厳密な等値比較）ので、スティックが中立を通過する
  たびに立脚静止へ落ちていた。ランプが時間的なヒステリシスになる。

## articara で描画して確かめる

歩容を実機に流す前に、`quadruped-gait` の
[`GaitVizFrame`](https://github.com/takarakasai/quadruped-gait) を Zenoh へ配信し、
articara の **Live gait feed** に描かせて目で確認できる。実機なし（`dump`）でも
実機を動かしながら（`run`）でも同じキーに流れる。

```sh
# 1) 配信側（実機なしで歩容だけ流す。--viz は自動で実時間になる）
misa-run dump --gait trot --vx 0.1 --secs 60 --viz --viz-endpoint tcp/127.0.0.1:7447

# 2) 受信側（別端末）
cd ../articara && cargo run --release --features viz -- \
    --model ../namiashi_description/namiashi.misa
#   → Live gait feed パネルで endpoint に tcp/127.0.0.1:7447 を入れて Start
```

`--viz-endpoint` はマルチキャスト探索が効かないホスト（同一ホスト / WSL2）向け。
効く環境なら両側とも省略してよい。実機を動かしながら見るなら `run` にも
同じ `--viz` 系オプションを付ける。

**送っているのはモデル座標系の角度**、つまりモータへ行く指令そのもの。
`GaitVizFrame::from_output` は歩容 / IK の符号のまま詰めるので、それを
そのまま流すと膝が反転して描かれる（向こうの doc コメントの警告）。
ここでは実機へ送るのと同じ [`JointVec`] からフレームを組んでいるので、
**画面に出た姿勢がそのまま実機の指令**になる。遷移中やポーズ再生中も描ける
のはこのため。ただし `GaitVizFrame` は脚 12 関節しか運ばないので、
`arm_pitch_joint` は articara 側で動かない。

## 設計上の要点

### 脚バスは 1 本 1 スレッドで自由走行

RS485 は半二重の要求応答で、待ち時間は USB の往復レイテンシに律速される
（ワイヤ上のビット時間より桁で大きい）。バスを跨いだ並列化だけが効くので、
バス 1 本にスレッド 1 本を割り当て、制御ループは共有スロットに目標を書いて
最新値を読むだけにしてある。

こうすると制御周期がバスのジッタから切り離され、**実際に何 Hz 出ているかを
`misa-run legs` で測ってから `control.rate_hz` を決められる**。

### 座標変換は HAL に閉じている

上位（歩容・ポーズ・チキンヘッド）はモデル（URDF / `.misa`）の関節角しか
扱わない。実機との差は設定の `sign` と `zero_pose_rad` だけ:

```text
q_motor = sign * (q_model − zero_pose_rad)
q_model = sign *  q_motor + zero_pose_rad
```

`zero_pose_rad` は **ゼロ出しを行った姿勢のモデル関節角**。LKMTech V3 の
位置制御は `rezero` で置いたソフトゼロからの相対量なので、「どの姿勢で
ゼロ出ししたか」を書いておかないとモデル角と対応が付かない。

### 立ち高さは `nominal_foot_body` に書き込む

`quadruped-gait` の `set_body_height_m` は `LinearCrawl` 専用で、CHAMP 系は
`LegKinematics::nominal_foot_body` を見る。`gait.stance_height_m` をどの歩容
でも効かせるため、コントローラを組むたびにこの Z を書き換えている
（`robot::Robot::kin_at_height`）。

### 既定の歩容モードは 3 種とも CHAMP 系

`GaitMode::LinearCrawl` は胴体を +X 直線に載せる専用プランナで、**横移動
(vy) と旋回 (wz) の指令を受け付けない**。「前後・左右・旋回をプロポで操る」
という要件に合わないので既定では使わない。直進の安定性を追い込みたいときだけ
`gait.crawl_use_linear = true` で選ぶ。

## 未確定・既知の制限

- **初期姿勢（250×350×700 mm の直方体に収める姿勢）は未確定。**
  `control.start_pose` が指す `.misa` のポーズ名で決まる。暫定で
  モデルに入っている `constrain`（thigh 1.0 / calf −2.0）を指している。
- **腕は受信機直結で、アプリからは駆動しない**（`[hardware.arm].protocol =
  "receiver_direct"`）。したがって**チキンヘッドと挨拶の腕動作は現状無効**。
  `teleop.arm` のチャンネル（既定 CH9）から角度を**観測**して、ログ・可視化・
  モデル状態には実際の角度を入れている。サーボの品種が決まったら
  `ArmProtocol` に variant を足し、`misa_hal::arm::ArmServo` の実装を
  差し込めば `is_app_driven() = true` になり、両方が自動的に有効になる。
- **無応答モータ 1 台あたり約 20 ms 待つ。** `lkmotor_driver::Rs485Driver` が
  シリアルの read タイムアウトを固定 20 ms で開き、締切判定を read の後に
  行うため、`response_timeout_ms` を 5 と書いても効かない。実測で 3 台無応答の
  バスは 16 Hz まで落ちる。生きているモータしかいなければ影響しないが、
  1 台落ちたときの縮退性能はこれで決まる。直すなら misa-actuator 側。
- **制御はまだ位置制御のみ。** `JointMode::Torque` の口は空けてあるが、
  MPC / WBC のトルク制御へ進むのは基盤が動いてから。LKMTech の MIT は
  ホスト側エミュレーション（`measure` + `set_torque` の 2 往復）なので、
  通信レートが半分になる点に注意。

## 構成

```
crates/
├── misa-hal/         実機の抽象化
│   ├── ch348.rs          UART 番号 → ポート（探索は sbus::discover に委譲）
│   ├── config.rs         配線・モータ id・符号・可動域（TOML）
│   ├── legs.rs           RS485 脚バス ×4（バス 1 本 1 スレッド）
│   ├── imu.rs            WitMotion 受信スレッド
│   ├── sbus.rs           S.BUS 受信スレッド（sbus クレートの上）
│   ├── arm.rs            ArmServo トレイト + 受信機直結 / 未配線
│   └── joint.rs          関節の並び順と値型
└── misa-runner/      アプリ
    ├── config.rs         制御・歩容・操縦・ポーズの設定
    ├── robot.rs          .misa の読み込みと歩容の組み立て
    ├── teleop.rs         S.BUS → 操縦指令
    ├── controller.rs     モード遷移の状態機械（実機非依存）
    ├── pose.rs           .misa のポーズ / シーケンス再生
    ├── chicken.rs        チキンヘッド
    ├── runner.rs         実機の制御ループ
    ├── calib.rs          符号・ゼロ点・可動域の実機校正
    ├── dump.rs           実機なしの歩容検証
    ├── viz.rs            articara へのライブ配信（Zenoh）
    └── diag.rs           実機を動かさない確認コマンド
```
