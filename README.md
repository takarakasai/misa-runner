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
| 全身制御 | WBC（階層 QP）で解き、**トルク / 速度 / 位置**から出力を選ぶ（既定は無効） |

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

**ブリッジ越しの機体（namiashi2 など）を実機でビルドするなら、その機体の
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

**`sim` / `render` を付けるなら、先に環境変数を入れること。** 無いと
`mujoco-rs` のビルドスクリプトが「pkg-config exited with status code 1」で
落ちる（MuJoCo を pkg-config で探しに行って見つからない）。

```sh
export MUJOCO_DYNAMIC_LINK_DIR=$HOME/.mujoco/mujoco-3.8.0/lib
export LD_LIBRARY_PATH=$MUJOCO_DYNAMIC_LINK_DIR:$LD_LIBRARY_PATH   # 実行時にも要る
```

**feature を触ったら総当たりで確認する。**

```sh
./scripts/check-features.sh          # ビルドのみ
./scripts/check-features.sh --test   # テストも
```

`render` だけで試していると `sim` 単体が壊れていても気づかない
（`render = ["sim", ...]` と積み上がっているため）。実際に落ちていた。
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
実例は namiashi2-runner（`misa-plant-ksm` + `namiashi2-run`）。

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
| namiashi2 | **namiashi2-runner リポジトリ**の `doc/` | STM ブリッジ越し。手順も profile も実行ファイル（`namiashi2-run`）もあちらにある |

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

### キーボードで操縦しながら歩容パラメータを詰める

`--pilot keys` は端末のキーで動かす。**歩容パラメータを歩きながら替えられる**
のがこちらの主目的で、周期・遊脚高さ・歩幅・接地比が対象。

```sh
cargo run --release --features sim -- sim --robot robots/keel.toml \
    --gait trot --kp 1200 --kv 30 --timestep 0.0005 --pilot keys
```

```text
w / s  前後      a / d  横        q / e  旋回      space  速度を 0
0/1/2  脱力/初期姿勢/歩行         z/x/c  Crawl/Walk/Trot
r / f  立ち高さ  i/k/j/l  傾ける  v  水平へ

t / g  周期   1 段 0.05 s     **揺れにいちばん効く**
y / b  遊脚   1 段 0.005 m
u / n  歩幅   1 段 0.01 m     速度から決まる着地点の上限
. / ,  接地比 1 段 0.02       0.5 が trot
m      いまの歩容の基準値へ戻す
```

状態行に現在値が出て、**基準値と違う項目には `*` が付く**。`z` / `x` / `c` で
歩容を替えると、その歩容の基準値に戻る（周期の基準は歩容ごとに違うので、
trot で詰めた値を crawl へ持ち込ませない）。

**替えても跳ねない。** `quadruped_gait::AnyGaitController::set_config` は
**位相を保つ**ので、歩容を作り直したときのように接地と遊脚の割り当てが
跨がない。作り直す道（歩容の切り替え）は 4 脚接地かつ速度 0 に限ってある。

台本からも同じ経路を通せる。**端末が要らないので試験と掃引に使える。**

```sh
misa-run sim --robot robots/keel.toml --gait trot --vx 0.12 \
    --cycle 0.45 --swing 0.06 --step-length 0.12 --duty 0.55

# 歩きながら替える（8 秒で切り替え、跳ねないかを見る）
misa-run sim ... --tune-at 8.0 --cycle 1.00
```

**詰めた値が定格に収まるかは `dump` で見る。** `sim` は動力学、`dump` は
要求レートと可動域で、同じフラグを受け付ける。

```sh
misa-run dump --robot robots/keel.toml --gait trot --swing 0.10
#   FL  hip  0.02/8.00  thigh  3.1/8.00  calf  9.16/8.00 ✗   ← ゲート超え
```

実機（`run`）には出していない。プロポは歩容パラメータを触らない
（チャンネルが足りない）ので、`Intent::gait_tune` は既定の「上書きなし」で
入り、**上書きを送らない操縦系の挙動は変わらない**。

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

### namiashi2 を MuJoCo で動かす（ROS 2 から操縦して articara で見る）

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
    sim --robot robots/namiashi2.toml --pilot ros2 --gait trot --secs 0 \
        --kp 1200 --kv 30 --timestep 0.0005 --base-height 0.42 \
        --viz --viz-endpoint tcp/127.0.0.1:7447

# 2) 別端末で articara。Live gait feed に tcp/127.0.0.1:7447 を入れて Start
cd ../articara && cargo run --release --features viz -- \
    --model ../namiashi2/model/proto2_asset/urdf/mvp_v2.misa

# 3) さらに別端末で操縦。**トピックは名前空間つき**
ros2 service call /namiashi2/misa_run/set_gait misa_msgs/srv/SetGait '{gait: 2}'
ros2 service call /namiashi2/misa_run/set_mode misa_msgs/srv/SetMode '{mode: 2}'
ros2 topic pub -r 20 /namiashi2/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.12}}'
```

`cmd_vel` を止めると 300 ms 後に速度が 0 に落ちて、その場で立つ
（**モードは変えない**）。

胴体をその場で傾ける（足は接地したまま）:

```sh
ros2 service call /namiashi2/misa_run/set_body_attitude \
    misa_msgs/srv/SetBodyAttitude '{roll: 0.0, pitch: 0.3, yaw: 0.0}'
```

**上限は「合成量」で、軸ごとではない**（`gait.body_attitude_max_rad`、namiashi2 は
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
cargo run --release --features render -- sim --robot robots/namiashi2.toml \
    --gait trot --vx 0.12 --secs 9 --kp 200 --kv 2.0 --base-height 0.35 \
    --video /tmp/namiashi2 --cam-dist 2.2 --cam-el -18 --cam-az 120

ffmpeg -framerate 30 -i /tmp/namiashi2/frame_%05d.png \
    -vf "eq=brightness=0.28:contrast=1.5" -c:v libx264 -pix_fmt yuv420p out.mp4
```

**明るさ補正は要る。** articara の MJCF エクスポータは光源を出さないので、
MuJoCo の既定ヘッドライトだけになって暗い。

**既定は 640×480。** MuJoCo のオフスクリーンバッファの既定がこれで、超えると
描かれた領域だけが左上に寄って黒帯になる。大きくするにはモデルの
`<visual><global offwidth/offheight>` が要るが、エクスポータはそれを出さない。

カメラは `--cam-az`（方位、90 で真横・180 で真後ろ）/ `--cam-el` /
`--cam-dist` / `--cam-z`。**機体の大きさで変える** — namiashi2 は 2.2 前後、
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
傾ける量が機体ごとに違う**から（namiashi 0.6 → 0.9 rad、namiashi2 0.20 → 0.50 rad）。
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

### CHAMP の crawl は静的歩容になっていない

**接地パターンを数えると分かる**（`sim`、namiashi、前進 0.05、20 秒）。

| | 全脚接地 | 1 本遊脚 | **2 本以上浮く** | roll 振幅 | 進む量 |
|---|---|---|---|---|---|
| CHAMP（既定） | 41% | 35% | **24%** | 9.9° | 64% |
| LinearCrawl（`crawl_use_linear = true`） | 88% | 12% | **0%** | 2.4° | 101% |
| 理論（接地比 0.85） | 40% | 60% | 0% | — | 100% |

接地比 0.85 の crawl なら 2 本以上が同時に浮くことは無い。全脚接地の割合は
理論どおりなので**歩容の計画は正しく、動力学側で余計に足が離れている**。
CHAMP の crawl は重心を支持三角形へ寄せないので、1 本上げるたびに胴体が
傾き、浮くべきでない足まで離れて残りが滑る。

**追従誤差は 0.016 rad しかなく、ゲインを上げると悪化する**（2 本以上が
24% → 68%）。つまり追従の問題ではない。

**crawl は「横・旋回が効くが揺れる」（CHAMP）か「直進だけで正しい」
（LinearCrawl）のどちらかを選ぶことになる。** trot は roll 1.4° で問題ない。

### 既定の歩容モードは 3 種とも CHAMP 系

`GaitMode::LinearCrawl` は胴体を +X 直線に載せる専用プランナで、**横移動
(vy) と旋回 (wz) の指令を受け付けない**。「前後・左右・旋回をプロポで操る」
という要件に合わないので既定では使わない。直進の安定性を追い込みたいときだけ
`gait.crawl_use_linear = true` で選ぶ。

## WBC（全身制御）

階層 QP（`quadruped_gait::wbc`、3 優先度の HoQP）に毎周期
`x = [q̈ | f_GRF | τ]` を解かせ、**その解をトルク・速度・位置のどれかの指令に
して出す**。解くのは 1 回で、出し方だけが変わる。

```sh
misa-run sim --robot robots/namiashi.toml --wbc-output torque   --gait crawl --vx 0.05
misa-run run --robot robots/namiashi.toml --wbc-output position
```

設定に書くのが正規の入口。`--wbc` / `--wbc-output` は掃引と切り分け用で、
プロファイルの `[wbc]` に重なる。

```toml
[wbc]
enabled = true
output  = "position"   # torque | velocity | position
```

**既定は無効**で、書かなければ従来どおり歩容の IK 出力をそのまま位置制御で
流す。有効にすると**脚 12 軸だけ**が WBC の解に変わる（補助軸の腕は
チキンヘッドの担当のまま）。

### 3 つの出力の違い

| | 出す量 | 実機で効く経路 |
|---|---|---|
| **`position`（推奨）** | `q_計画 + ½·q̈·dt²` と `torque_ff_nm` | 位置指令（`0xA4`）+ 前置トルク |
| `velocity` | `q̇_計画 + q̈·dt + kp·(q_計画 − q_実測)` と `torque_ff_nm` | 速度指令（`0xA2`）+ 前置トルク |
| `torque` | `τ_WBC + kp·(q*−q) + kd·(q̇*−q̇)` を 1 本のトルクに | トルク指令（`0xA1`） |

**トルク出力も純粋な τ ではない。** legged_control は
`setCommand(pos_des, vel_des, kp=5, kd=3, torque)` で、トルクに**弱い関節
PD** を添えて出す（hybrid joint、
[`ref/legged_control/legged_controllers/src/legged_controller.cpp`](../articara/ref/legged_control/legged_controllers/src/legged_controller.cpp)）。
**WBC が出すのは加速度であって位置ではなく、位置の誤差を戻す積分器が
どこにも無い**ので、τ だけだとモデル誤差のぶん関節がずれ続ける。この PD が
その錨になる。LKMTech は MIT を持たないので**ホスト側で足して 1 本の
トルクにしてから**出す（`[wbc] joint_kp / joint_kd`、既定 **5 / 0.3**）。
両方 0 にすれば純粋なトルクになる。

**`joint_kd` に legged_control の 3 をそのまま使ってはいけない。** あちらの
kd はモータ内 2.5 kHz の PD の値で、ここは 200 Hz のホスト側。脚リンクの
慣性（1e-3 kg·m² 程度）に対して kd·dt/I が 2 を超え、**離散の微分項が
発振する**。MuJoCo で立ち止まらせただけで、起動した最初の周期から τ が
±18 N·m で毎周期符号を替え、上限で削られた結果として胴体が 0.20 → 0.03 m
まで沈んだ（`MISA_WBC_TAU=<n>` で WBC の τ と Plant の実トルクを並べて
見つけた）。kd ≤ 0.3 なら τ_WBC が Plant の実トルクと一致して 0.200 m に
立つ。kd=1.0 で既に崩れ始める（5 走行のヨーが −44°）。**2026-09-06 より
前のトルク出力の数字は全部この発振の上に載っていて無効。**

`position` は同じ hybrid を**関節の位置サーボ側で**やる形で、articara が
namiashi の WBC を検証したときの構成
（[`articara/tests/wbc_walk.rs`](../articara/tests/wbc_walk.rs)）。
**τ を受け取れない機体では、位置出力の WBC は歩容をそのまま流すのと
変わらない**（LKMTech の `0xA4` がそれ）。MuJoCo と MIT を持つ機体では
前置として効く。

### 遊脚は直交空間で解く

legged_control の `formulateSwingLegTask` と同じ形:

```text
  J_足 · q̈ = kp·(p* − p) + kd·(ṗ* − ṗ) − J̇·v
```

`quadruped_gait::wbc::tasks::swing_leg` は**関節空間**なので使っていない
（優先度の積み方も自前で持っている）。関節空間の PD は同じゲインでも脚の
姿勢で効きが変わる — ヤコビアンが姿勢に依るので、伸び切った脚では足先が
ほとんど動かない。ゲインは legged_control の `task.info` と同じ 350 / 37。

### トルク上限は連続定格 × 係数

`.misa` の `effort` は**連続定格**（namiashi は hip / thigh 1.5、calf
2.205 N·m。減速比で割るとどちらもモータ軸 0.14〜0.15 N·m で、同じ 1 個の
数字が比で配られている）。瞬間はそれより出るので、係数で持ち上げる:

```toml
[wbc]
torque_scale  = 2.0    # 瞬間は連続定格の 2 倍まで（軸ごとの比は保つ）
max_torque_nm = 0.0    # 絶対値の頭打ち。0 で無効
```

`sim` では**アクチュエータ側の上限も同じ係数で上がる**（揃えないと QP が
出ないトルクを当てにする）。上げれば歩けるようになる話ではない — kd=3 の
発振を抱えたまま 2 倍にしても進む量は落ちた。連続定格のままで trot 0.80 が
歩けている（下の表）。

### 実機のトルク制御にはトルク定数が要る

`[hardware.legs] torque_constant_nm_per_a` を書いていないシリアル構成では、
`set_torque` に渡した数が**そのまま電流 (A) として線に乗る**
（`MotorConfig::current_units` が `Kt = 1/減速比` を選ぶため）。WBC が出すのは
N·m なので、そのまま流すと 12 軸ぶんの N·m が A に読み替えられる。定格
1.5 N·m なら 1 軸 1.5 A、12 軸で 18 A — 電源の電流制限 5 A
（[`doc/motor_map.md`](doc/motor_map.md)）を大きく超えてレールが崩壊する。

そのため**トルク定数が無い構成では `SerialPlant` が `Torque` を名乗らず**、
`run` は起動時に止まる（`check` も知らせる）。`sim` のトルクは MuJoCo へ行く
ので単位は N·m のまま正しく、この制限は掛からない。

### どこまで動くか

**歩容の詰め方でまるで変わる。進む量だけを見て WBC の良し悪しを判断しない
こと。** 歩容が出せる速度は `歩幅 / (周期 × 接地比)` で頭打ちになり、
ライブラリの既定では crawl が 0.042 m/s しか出ない（→
[トルク上限は…](#歩幅が速度の上限を決める)）。

MuJoCo・同梱の namiashi・16 秒（2026-09-06）。articara が詰めた歩容の値
（`step_length_m = 0.145`、周期 trot 0.320 / walk 0.500 / crawl 0.800、
`swing_height_m = 0.04`、`mpc_capture_point_gain_s = 0`）、MPC 歩容で:

| 歩容 | 指令 | WBC 無効 | 位置 | 位置 + MPC | トルク + MPC（kd=3、無効） | **トルク + MPC（kd=0.3）** |
|---|---|---|---|---|---|---|
| trot | 0.80 m/s | +9.302 m | +9.794 m | +10.375 m（104 %） | +4.070 m | **+11.799 m**（118 %） |
| walk | 0.33 m/s | +3.830 m | +3.894 m | +4.022 m（97 %） | +1.048 m | **+4.853 m**（118 %） |
| crawl | 0.17 m/s | +2.017 m | +2.231 m | +2.182 m（103 %） | +0.150 m | **+2.482 m**（117 %） |

括弧は指令に対する追従率（進んだ距離 / (12.5 s × 指令)）。**転倒はどれも
無し。** 位置出力の追従率は articara の `namiashi_tuned_gaits_hold` が押さえて
いる 90〜110 % の帯に入る。

**トルク出力は `joint_kd` を 3 → 0.3 に下げただけで位置出力を追い越した。**
横ずれも小さい（trot 16 s で 0.46 m。位置出力は 1.8 m、ヨー 12.6°）。ただし
**指令より 18 % 速い**（trot / walk / crawl とも）。速度の追従は MPC の参照
（`velocity_track_kp`）と接地足の滑りの両方が絡み、まだ詰めていない。

### 取り込んだ改善と採用可否

legged_control と文献（Sleiman 2021、Grandia 2022、Bellicoso 2016）から
1 つずつ入れて、上と同じ 5 走行（trot 0.78 / 0.80 / 0.82、walk 0.33、
crawl 0.17、各 16 s）で測った。基準は `joint_kd = 0.3`・MPC・トルク出力
（平均追従率 1.18、trot の横ずれ 0.46 m、ヨー 0.4°、転倒 0/5）。

| # | 項目 | 設定 | 結果（平均追従率 / trot 横ずれ / ヨー） | 採用 |
|---|---|---|---|---|
| 0 | **hybrid PD の kd を 200 Hz 向けに** | `[wbc] joint_kd`（既定 0.3） | 0.26 → 1.18、転倒 0/5。kd=0 は 1.21 / 0.83 m / 4.8°、kd=1.0 は 0.96 / ヨー −44° | **既定 0.3** |
| 1 | MPC に推定した高さ・姿勢を渡す（legged_control の `setCurrentObservation`） | `[gait] mpc_observe_pose`（既定 true） | ON 1.21 / 0.83 m / 4.8°、OFF 1.18 / 0.96 m / 6.0°（kd=0 で比較）。平地では差が小さいが、高さ・姿勢の誤差が MPC に**構造的に見えない**のを直すもの | **既定 true**（選択可） |
| 2 | 18 状態 LKF（legged_control の `KalmanFilterEstimate`） | `[gait] estimator = "kalman"` | 1.20 / **0.13〜0.23 m** / −3〜−5°。横ずれは半分以下、ヨーは少し増える。MuJoCo の加速度計は胴体速度の差分で作っている | **選択可**（既定 `leg_odometry`。実機の IMU で確かめてから） |
| 3 | 実測の接地（力 5 N で切る） | `[wbc] use_measured_contact = true`、`contact_force_threshold_n` | 1.18 / **1.1〜1.5 m** / 10°。着地の早い相は拾えるが横ずれとヨーが悪化 | **選択可**（既定 false） |
| A | 遊脚の加速度誤差積分（Grandia 式 39–40） | `[wbc] swing_accel_integral_k / _sat_nm` | K=0.3・飽和 0.5: 1.06 / −1.2 m / −7°（飽和に張り付く）。飽和 0.15: 1.13 / 0.43 m / 1°（基準と同等） | **選択可**（既定 0 ＝無効） |
| 7 | 着地時の足の鉛直速度（`swing_touchdown_vz`） | — | quadruped-gait では `centroidal` の `legged_control_parity` 経路だけが見る。`mpc` / `champ` には効かない | 見送り |
| C | 接地力の変調（Bellicoso: 離地前に 0 へ、着地後に立ち上げ） | — | 接地力タスクは優先度 2 で、4 脚接地では解を動かせず 2 脚接地では力が一意に決まる（`MISA_WBC_NULLSPACE`）。歩容も相の残り時間を出していない | 見送り |
| D | 重み付き単一 QP（Grandia） | — | quadruped-gait の `wbc` は HoQP だけ。自前の 3 段も同じ | 見送り |

**#0 だけが効いた改善で、あとは横ずれ・ヨーの詰めに効く選択肢。** 5 走行は
全部平地で、#1〜#3 の本領（外乱・段差・着地のずれ）は測っていない。

歩容を**既定値のまま**（歩幅 0.06/0.08/0.10 m）0.05 m/s で走らせると:

| 歩容 | コントローラ | WBC 無効 | 位置 | 速度 | トルク |
|---|---|---|---|---|---|
| crawl | champ（既定） | +0.388 m | +0.436 m | +0.029 m | +0.221 m |
| crawl | **mpc** | +0.403 m | **+0.476 m** | −0.874 m | 9.5 s で転倒 |
| trot | champ（既定） | +0.496 m | +0.590 m | +0.108 m | 4.5 s で転倒 |
| trot | **mpc** | +0.154 m | **+0.659 m** | +0.603 m | 4.0 s で転倒 |

- **位置出力は素の歩容より良い。** 詰めた設定で +0.2〜9 %、既定で +12〜19 %。
- **MPC は位置出力をさらに少し良くする。** 詰めた設定では差が小さいが、
  既定の crawl ではヨーのずれが 19.4° → 8.9° と目に見えて減る。
- **MPC 単体（WBC 無効）は trot を悪くする**（既定設定で +0.496 → +0.154 m）。
  接地点の捕捉点フィードバックが、追従の悪い相手に対して正帰還になる。
  `gait.mpc_capture_point_gain_s = 0` で消える（articara も 0 に落として
  いる）。
- **この表のトルク列は `joint_kd = 3` のもので無効**（上の「`joint_kd` に
  legged_control の 3 をそのまま使ってはいけない」）。詰めた設定での再測は
  上の表。
- **ゲインはすべて 2.4 kg のモデルで詰めたもの。** 実機は 3.3 kg で脚に
  73 %（補正モデルは `articara/tests/fixtures/namiashi/`）。実機へ持って
  いく前に採り直すこと。
- **速度出力は詰め切れていない。** シムのアクチュエータのゲイン
  （`--kv-velocity`、既定 20）に強く依る。位置を先に見ること。

### 歩幅が速度の上限を決める

`歩幅 / (周期 × 接地比)` がその歩容の最高速度。ライブラリの既定では:

| | 歩幅 | 周期 | 接地比 | 上限 |
|---|---|---|---|---|
| crawl | 0.06 | 1.667 | 0.85 | **0.042 m/s** |
| walk | 0.08 | 0.600 | 0.75 | 0.178 m/s |
| trot | 0.10 | 0.400 | 0.50 | 0.500 m/s |

**同梱の `robots/namiashi.toml` は `max_vx_m_s = 0.15` を宣言しているが、
crawl では 0.042 m/s しか出ない。** 追従率だけが落ちる。歩幅を上げれば直る
（`[gait] step_length_m`。articara は 3 歩容とも 0.145 m ＝ 脚長 0.306 m の
47 % に置いている）。**上げると遊脚が擦る**ので `swing_height_m` も一緒に
上げること。同梱プロファイルはまだ既定のままにしてある — 実機の可動域と
一緒に決める話なので。

### 歩容コントローラ（MPC）

歩容の**型**（Crawl / Walk / Trot ＝踏み替えの並び。プロポの CH6）とは別に、
**どのコントローラで解くか**を選べる。

```toml
[gait]
controller = "mpc"    # auto | champ | linear_crawl | mpc | centroidal
```

```sh
misa-run sim --robot robots/namiashi.toml --gait-controller mpc --wbc-output position
```

| | 接地力の予測 | 観測 | |
|---|---|---|---|
| `auto` | 無し | 見ない | **既定。** `crawl_use_linear` の従来どおりの解釈 |
| `champ` | 無し | 見ない | 開ループの運動学 |
| `linear_crawl` | 無し | 見ない | 直進専用（横移動と旋回を受け付けない） |
| `mpc` | SRBD | 速度 | 胴体を 1 剛体と見た MPC |
| `centroidal` | 重心 SRBD | 速度 | 重心のずれと慣性を持つ版 |

**MPC の機体諸元はモデルから入れる。** 入れないと quadruped-gait の既定
（Cheetah 級の 9 kg・慣性 0.07/0.26/0.24）で走り、namiashi（2.4 kg・
0.008/0.034/0.034）とは 4〜8 倍ずれる。`misa-run check` が実際に入る値を出す:

```
歩容コントローラ: MPC (SRBD)（Crawl=Mpc Walk=Mpc Trot=Mpc）
  MPC の機体諸元: 質量 2.400 kg / 慣性 diag (0.0083, 0.0338, 0.0338) kg·m²
                  / 重心 (+0.0015, +0.0047, +0.0003) m
```

### 参照は 2 通りある

| 歩容 | `f_grf_des` | `a_base_des` |
|---|---|---|
| `mpc` / `centroidal` | MPC が解いた接地力（鈍らせて使う） | その接地力から Newton–Euler で作った胴体加速度 + 姿勢 PD |
| それ以外 | 体重を立脚数で割った鉛直力（**静的配分**） | 姿勢 PD と、脚オドメトリで測った胴体の位置・速度への PD |

後者は**準静的**で、加速に伴う荷重移動も支持多角形の乗り換えも入っていない。
どちらを使ったかは `run` の状態行に「参照 MPC / 準静的」として出る。

接地足は歩容の立脚フラグ（足裏センサが無いのでこれが唯一。MuJoCo では
`[wbc] use_measured_contact = true` で垂直力 5 N 超の足を立脚へ倒せる）。
脚オドメトリ（[`estimator.rs`](crates/misa-runner/src/estimator.rs)）は
「接地していると歩容が言っている足は世界に対して止まっている」という仮定
だけを使う。世界座標の絶対位置は出せないが、**支持足に対する相対位置**は
出せて、転ぶかどうかに効くのはそちらだけ。同じ推定を MPC 歩容も使う
（`[gait] mpc_observe_pose`、既定 true — 高さと roll / pitch を MPC の現在
状態に入れる。quadruped-gait の SRBD MPC は既定で「公称高さ・水平」を現在
状態に置くので、これが無いと沈んでも傾いても MPC には見えない）。

```toml
[gait]
estimator = "kalman"   # leg_odometry（既定）| kalman
```

`kalman` は legged_control の 18 状態 LKF
（`legged_estimation::LinearKalmanEstimator`。胴体位置・速度・足 4 本の
世界位置）。IMU の加速度で予測して足の運動学で補正し、接地していない足は
共分散を 100 倍にして見ない。高さと速度がこちらに替わり、**計画に対する
位置誤差は脚オドメトリのまま**（LKF は計画を知らない）。MuJoCo の加速度計は
胴体速度の差分で作っている（モデルに IMU サイトが無い）ので、実機の IMU の
雑音での挙動は未確認。

### 計算時間

MuJoCo の物理込みで測った 1 周期あたり（開発機。**SBC ではもっと掛かる**）:

| | ms/tick | 制御だけ |
|---|---|---|
| WBC 無効 / CHAMP | 0.20 | — |
| WBC 無効 / MPC | 0.54 | 0.34 |
| WBC / CHAMP | 1.76 | 1.56 |
| **WBC / MPC** | **2.32** | **2.1** |

**200 Hz の予算は 5 ms**なので、開発機では 4 割ほど。SBC へ持っていく前に
`run` の状態行の `遅延最大=` を見ること。足りなければ `control.rate_hz` を
下げるか `gait.mpc_horizon_steps` を削る（既定 10 段 × 30 ms）。

### 調べ方

`MISA_WBC_DEBUG=1` で毎周期 1 行出る（参照の出どころ・接地・脚オドメトリ・
姿勢・角速度・要求した胴体加速度・解の胴体加速度・接地力の合計、それと
トルクが定格の 9 割を超えた軸）。**要求と解の胴体加速度が一致していれば
QP は仕事をしている**ので、そこが合っていて実機が付いてこないならモデルか
飽和を疑う。

`MISA_WBC_TAU=<n>` で `sim` が n 周期ごとに **WBC の τ と Plant が実際に
掛けたトルク**を脚ごとに並べる。位置出力で立ち止まらせれば Plant 側は
重力補償そのものなので、**モデルが合っているかの検算**になる（namiashi は
hip 0.42 / thigh 0.02 / calf 0.70 N·m で一致した）。トルク出力で両者が
食い違えば、その差はホスト側の PD か上限の削りで、kd の発振はこれで見つけた。

`MISA_WBC_NULLSPACE=1` で優先度ごとの零空間の次元が出る。**下の優先度に
自由度が残っているか**はこれで分かる — 0 なら、そこより下のタスクは何を
言っても解を動かせない。接地力の参照（優先度 2）が 4 脚接地で効かないのは
これで確かめた。

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
- **同梱プロファイルの歩幅はライブラリ既定のまま。** `max_vx_m_s = 0.15` を
  宣言しているが crawl は 0.042 m/s しか出ない（`歩幅 / (周期 × 接地比)`）。
  歩幅を上げれば直るが、遊脚の擦りと可動域は実機で確かめてから決めること。
  詳しくは [歩幅が速度の上限を決める](#歩幅が速度の上限を決める)。
- **WBC のトルク出力（位置ループを外す形）は歩けない。** 詰めた歩容でも
  指令の 2〜13 % しか進まない。**WBC が出すのは加速度で、位置の誤差を戻す
  積分器がどこにも無い**のが理由で、articara も同じ結論で hybrid（＝この
  crate の位置出力）へ移している。実用は `output = "position"`。
- **WBC の速度出力は詰め切れていない。** シムのアクチュエータのゲイン
  （`--kv-velocity`）に強く依り、MPC 歩容と組むと後ろへ走る。
- **MPC 歩容は位置出力と組むこと。** WBC 無効のまま MPC を選ぶと、接地点の
  捕捉点フィードバックが追従の悪い相手に対して正帰還になり、trot が
  +0.496 → +0.154 m に落ちる（`gait.mpc_capture_point_gain_s = 0` で消える）。
- **WBC のトルク・速度出力は実機で回したことがない。** HAL からモータの
  コマンド（`0xA1` / `0xA2`）まで配線して MuJoCo で確かめてあるが、
  namiashi の実機に流したことはまだない。LKMTech の MIT は
  ホスト側エミュレーション（`measure` + `set_torque` の 2 往復）なので、
  通信レートが半分になる点にも注意。

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
    ├── wbc.rs            全身制御（階層 QP）と、その解の出し方
    ├── estimator.rs      接地足から胴体の速度・高さを測る（脚オドメトリ）
    ├── pose.rs           .misa のポーズ / シーケンス再生
    ├── chicken.rs        チキンヘッド
    ├── runner.rs         実機の制御ループ
    ├── calib.rs          符号・ゼロ点・可動域の実機校正
    ├── dump.rs           実機なしの歩容検証
    ├── viz.rs            articara へのライブ配信（Zenoh）
    └── diag.rs           実機を動かさない確認コマンド
```
