# realbot_build — keel を実機（機体の PC）でビルドする

対象機: **keel**（STM ブリッジ経由 / ROS 2）
制御機: **機体の PC — aarch64 / ROS 2 humble**。ソースから**ネイティブビルド**する。
関連: [`../ros/README.md`](../ros/README.md)（ブリッジとの入出力）/
[`handover.md`](handover.md)（namiashi の SBC 移行。**あちらとは前提が違う**）/
[`../robots/keel.toml`](../robots/keel.toml)

作成日: 2026-09-03

一括で確認するなら:

```sh
source /opt/ros/humble/setup.bash
./scripts/setup-realbot.sh              # 前提を調べるだけ。何も入れない
./scripts/setup-realbot.sh --apt --build
```

**機体で確かめた環境**（2026-09-03、`setup-realbot.sh` の出力）:

```
aarch64 / Linux 5.15.185-rt-tegra / 8 コア / RAM 61.4 GiB / 空き 1.7 TiB
ROS 2 humble / rustc 1.98.0 / libclang-14.so.13
build-essential pkg-config git curl libudev-dev colcon すべて既存
```

RAM が 61 GiB あるので `--jobs` を絞る必要はない（§5）。**カーネルが PREEMPT_RT** なのは namiashi の
radxa と違う点で、`run` のジッタを詰める段で効く（§7）。

**ビルドは機体で通った**（2026-09-03、`--features ros2`）。ただし前提のうち
2 つは足りず、手当てが要った:

- **`libclang-dev`** — `libclang1-14` だけでは bindgen が `stdbool.h` を
  開けない（§1）
- **`.cargo/config.toml` を消す** — PC からコピーしたツリーに `[patch]` が
  付いてきて、cargo が解決の段で止まる（§2）

ブリッジとの実通信は 2026-09-04 に通った（§9）。

---

## 0. namiashi の SBC と何が違うか

`handover.md` §5 は namiashi（radxa-cubie-a7z + CH348 の RS485）の話で、
**keel はほとんど当てはまらない。**

| | namiashi | keel |
|---|---|---|
| 制御機 | radxa-cubie-a7z（別置きの SBC） | 機体の PC |
| モータへの経路 | RS485 ×4 を自分で叩く | **STM ブリッジに ROS 2 で頼む** |
| ch9344 ドライバ | **要る**（DKMS） | 要らない |
| `dialout` / udev | 要る | 要らない |
| ROS 2 | 任意（`--features ros2` は操縦用） | **必須**（これが唯一の経路） |
| 独自 msg | misa_msgs だけ | misa_msgs **＋ low_command_msgs / low_state_msgs** |
| 校正値・ゼロ点 | こちらが持つ（`calib`） | **STM が持つ**（こちらは持たない） |
| モデル | `models/namiashi/` の submodule | **相対パスの外部ディレクトリ**（§4 で詰まる） |

ただし `misa-hal` はワークスペースのメンバなので、**keel しか動かさない機体でも
serialport はビルドされる**（`libudev-dev` が要るのはこのため）。

---

## 1. 実機に入れておくもの

| 物 | 要件 | 確かめ方 |
|---|---|---|
| ROS 2 | **humble**（r2r 0.9.5 の対応は foxy/galactic/humble/iron/jazzy/rolling） | `echo $ROS_DISTRO` |
| `sensor_msgs` / `geometry_msgs` | `LowState` が `sensor_msgs/Imu`、`cmd_vel` が `geometry_msgs/Twist` | `ros2 interface show sensor_msgs/msg/Imu` |
| ブリッジの ws | `low_command_msgs` / `low_state_msgs` が colcon build 済み | `ls <ws>/install/share/low_command_msgs` |
| Rust | **1.88 以上** | `rustc -V` |
| apt | `build-essential pkg-config git curl libudev-dev python3-colcon-common-extensions` | `setup-realbot.sh` |
| libclang | bindgen が dlopen できること | `ldconfig -p \| grep libclang` |
| 空き容量 | **4 GiB 以上**（`target/` 2.6 GiB + `~/.cargo` 1 GiB） | `df -h .` |
| 網 | GitHub / crates.io（依存は全部 https の公開リポジトリ。**認証情報は要らない**） | `cargo fetch` |

**Rust は 1.88 以上。** `handover.md` §5.4 の「1.85 以上」はもう足りない
— `Cargo.lock` の 542 パッケージを総なめすると `time 0.3.55` /
`darling 0.23` / `serde_with 3.22` が `rust-version = 1.88` を宣言している。
1.85 では**依存の解決ではなくコンパイルで**落ちるので、原因が分かりにくい。

```sh
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
```

**libclang は 2 つ揃って初めて足りる。** `r2r` は bindgen で `rcl` の束縛を
作るので、

1. **共有ライブラリ** — `clang-sys` が実行時に dlopen する
   （`libclang-dev` でも `libclang1-<N>` でも可）
2. **clang の組み込みヘッダ** — resource dir の `stdbool.h` / `stddef.h` など。
   **`libclang1-<N>` には入っていない**（`libclang-common-<N>-dev`。
   `libclang-dev` か `clang` を入れると付いてくる）

1 だけの機体では、ROS のヘッダを開いた先でこう落ちる（2026-09-03 に踏んだ）:

```
/opt/ros/humble/include/rcutils/rcutils/allocator.h:25:10:
  fatal error: 'stdbool.h' file not found
thread 'main' panicked at r2r_rcl-0.9.5/build.rs:100:10:
  Unable to generate bindings: ClangDiagnostic(...)
```

```sh
sudo apt-get install -y libclang-dev      # 両方入る
clang -print-resource-dir                 # 組み込みヘッダの場所
```

判定を apt のパッケージ名でやらないのはこのため（`libclang1-18` だけの PC でも
`ldconfig` には出る）。`setup-realbot.sh` は **`ldconfig` と
`clang -print-resource-dir` の両方**を見る。

---

## 2. clone

```sh
git clone --recurse-submodules https://github.com/takarakasai/misa-runner.git
cd misa-runner
```

`--recurse-submodules` が要るのは `models/namiashi/`（namiashi のモデル）で、
**keel には要らない**。付けても害はないので付けておく。

`Cargo.lock` を追跡しているので、実機は PC とまったく同じ revision を引く。
**逆に、`Cargo.lock` が指す revision が push されていないと実機だけ落ちる**
（`failed to load source` / `object not found`）。PC で兄弟クレートを直した
直後に実機をビルドするなら、その push を先に済ませること。

コピーで持ち込んだツリーには `ros/build` `ros/install` `target/`
`.cargo/config.toml` が付いてくる（どれも `.gitignore` 対象なので clone では
来ない）。**別の distro / 別の arch で作ったものが混ざる**ので、
`setup-realbot.sh` の §7 が点検する。作り直すのが安全:

```sh
rm -rf ros/build ros/install ros/log     # misa_msgs を humble で作り直す
cargo clean                              # x86_64 の成果物を捨てる
rm -f .cargo/config.toml                 # PC の [patch] を持ち込まない
```

**`.cargo/config.toml` は必ず消すこと。** `dev-siblings.sh` が書く `[patch]` は
PC のローカルパスを指しているので、機体では解決の段で止まる（**使っていない
crate でも止まる**）:

```
error: failed to load source for dependency `articara`
  unable to update /home/keel/work/20260903/articara
  No such file or directory (os error 2)
```

---

## 3. 環境変数

```sh
source /opt/ros/humble/setup.bash
source scripts/realbot-env.sh                       # ブリッジの ws は自動で探す
KSM_WS=~/ksm_mvp_real_ws/install source scripts/realbot-env.sh   # 明示するなら
```

`scripts/realbot-env.sh` が入れるもの:

| 変数 | なぜ要るか |
|---|---|
| `AMENT_PREFIX_PATH` | **独自インタフェースなので、ここに無いと r2r が型を作れずビルドが落ちる。** misa_msgs（このリポジトリ）とブリッジの ws を足す |
| `LD_LIBRARY_PATH` | 実行時に typesupport を dlopen する |
| `IDL_PACKAGE_FILTER` | r2r に束縛を作らせる msg パッケージを絞る |
| `PYTHONPATH` | `ros2 service call` から misa_msgs の型を触るときだけ |

**`IDL_PACKAGE_FILTER` を絞ると実機のビルドが軽くなる。** 未設定だと
`AMENT_PREFIX_PATH` に載っている**全パッケージ**が bindgen にかかる。要るのは:

```
misa_msgs;low_command_msgs;low_state_msgs;geometry_msgs;sensor_msgs;std_msgs
```

`rcl_interfaces` / `builtin_interfaces` / `unique_identifier_msgs` /
`action_msgs` は r2r が自分で足す。`sensor_msgs` が要るのは `LowState` が
`sensor_msgs/Imu` を持つため、`std_msgs` は `Header` のため。
**絞り方を変えると r2r が丸ごと再ビルドされる**（`rerun-if-env-changed`）ので、
付けたり外したりを往復させないこと。

`RMW_IMPLEMENTATION` と `ROS_DOMAIN_ID` は**このスクリプトでは設定しない**。
ブリッジと揃っていないと話せないが、正解を知っているのは向こうなので、
ここで上書きすると「繋がらない理由が 2 つになる」だけ。**ブリッジ側の値を
見て、同じものを自分の側にも入れること**（既定は fastrtps / 0）。

---

## 4. モデルを機体に置く ★実機で最初に落ちるところ★

`robots/keel.toml` の `model` は**このリポジトリの外**を指している:

```toml
model = "../keel/model/proto2_asset/urdf/mvp_v2.misa"
```

**`../keel` は git リポジトリではない**（PC のローカルディレクトリ）。つまり
`git clone` では来ないし、submodule でもないので `--recurse-submodules` でも
来ない。**放っておくと実機の `check` が「モデルが読めない」で止まる。**

道は 3 つ。

| | やり方 | 得 | 損 |
|---|---|---|---|
| a | 機体にも `misa-runner` の隣へ `keel/model/...` を scp して同じ相対位置に置く | プロファイルを触らない | 機体ごとに人手。取り違えたら気づけない |
| b | モデルをこのリポジトリに同梱（`models/keel/`）して `model` を書き換える | `clone && build` で立ち上がる。namiashi と同じ形 | **元（`../keel`）と二重になる。ずれる** |
| c | `model` を機体の絶対パスにする | 機体の都合に合わせられる | プロファイルが機体ごとに分岐する |

**モデルの取り違えは実際に起きている。** 2026-09-01 に古い `mdls/keel/` の
モデル（可動域が別物）で歩容を評価して、2 コミットぶん誤った結論を出した。
どの道を採るにしても、**機体に置いたモデルの日付と sha256 を記録しておくこと。**
いまの正解は:

```
../keel/model/proto2_asset/urdf/mvp_v2.misa
sha256 094bb2c43e96912ac5d8b401782c1ff84ac9a6c212fa02c219c7156fd6ea6235
```

メッシュ（`urdf/mesh/` と `urdf/meshes/decomposed/`、合わせて 750 KiB）は
**`run` には要らない**。無くても `check` と `run` は通るが、当たり判定メッシュ
37 件の警告が出て `sim` は回らなくなる。置けるなら一緒に置く。

---

## 5. ビルド

```sh
cargo build --release --features ros2                     # viz つき（既定）
cargo build --release --no-default-features --features ros2   # 軽いほう
```

`--features ros2` を付け忘れると、`run` が
**「このビルドには ros2 が入っていません」**で落ちる（keel はブリッジしか
経路が無いので、`kind = "ros2"` を扱えないビルドでは何もできない）。

`viz` を残すと `--viz` で articara に実時間描画できる（Zenoh。SBC で `run`、
PC で描く）。要らなければ `--no-default-features` で zenoh ごと消える。

PC での実測（x86_64 / 16 コア / jazzy、依存は取得済み、`target/` は空から）:

| 構成 | 実時間 | CPU 時間 | ピーク RSS | バイナリ | strip 後 |
|---|---|---|---|---|---|
| `--features ros2`（viz つき、フィルタあり） | 2 分 55 秒 | 33 分 | 3.2 GiB | 309 MB | **22 MB** |
| `--no-default-features --features ros2`（フィルタなしで計測） | — | — | — | 79 MB | — |

`target/` は 2.6 GiB。バイナリが大きいのは `[profile.release] debug = true`
（制御ループのジッタを追うためで、速度には影響しない）。

**aarch64 の見込みは CPU 時間から割ること。** 33 分 ÷ コア数 ×（単コアの遅さ）。
8 コアなら 10〜20 分を見ておく。**メモリのほうが先に詰まる** — ピーク
3.2 GiB は 16 並列の rustc なので、**8 GiB 以下の機体では `--jobs 4`**
（`setup-realbot.sh --jobs 4`）に落とす。

---

## 6. 動作確認（モータを動かさない順）

```sh
source /opt/ros/humble/setup.bash && source scripts/realbot-env.sh

./target/release/misa-run check  --robot robots/keel.toml   # 実機に触れない
ros2 topic hz /low_state                                    # ブリッジが出しているか
./target/release/misa-run bridge --robot robots/keel.toml --secs 10
./target/release/misa-run dump   --robot robots/keel.toml
```

| 段 | 見るもの |
|---|---|
| `check` | 関節 23 / nq=16、ポーズ `crouch` / `extend`、MIT ゲイン、トピック名。**メッシュの警告が 37 件出たらモデルの置き方が §4 の通りでない** |
| `ros2 topic hz` | ブリッジが `/low_state` を出しているか。ここが無音なら RMW / `ROS_DOMAIN_ID` の食い違いを先に疑う |
| `bridge` | **指令は脱力のまま**（`kp = kd = τ = 0`）なので繋いでも動かない。何軸を名前で解決できたか・状態がどれだけ古いか・異常ビットが立っていないか |
| `dump` | 歩容の要求が可動域と定格の内側か。`max_target_rate_rad_s` と突き合わせる |

`bridge` が「解決できなかった軸」を名前で挙げたら、**`LowState.joint_names`
とモデルの関節名が食い違っている**。黙って 0 を送らないようにしてある。

---

## 7. `run` に進む前に

ビルド環境ができても、**実機で歩けるかは別**。埋まっていない穴は
memory と `handover.md` にあるが、keel について要点だけ:

- **`[hardware.mit_gains]` の既定 kp 120 / kd 2.0 では立てない。** 立ち上げ用の
  柔らかい値（脚を浮かせて指令角へ寄るのを見る用）。MuJoCo で同じ制御則を
  回すと胴体が 0.250 まで沈む。足だけで立つのは kp 300 から。
- **`start_pose` / `rest_pose` = `crouch`** は IK で作った姿勢。**実機の電源
  投入姿勢と違うなら直す。** 脱力からの遷移はここを始点に張るので、食い違うと
  実機では起こらない軌道が出る。
- **接地センサが無い**（`has_contacts: false`）。接地推定もしていない。歩容は
  接地を入力に取らないので位置制御で歩くところまでは成立するが、早着地・
  遅離地の吸収と脚オドメトリは無い。
- **転倒の手がかりは IMU の姿勢角だけ。** 鉛直から `control.max_tilt_rad`
  （keel は導出値 0.50 rad = 29°）を超えたら `run` が ERROR を出す。
  **自動では脱力しない** — 荷重がかかった四足を脱力させると崩れるので、
  止めるかどうかは operator が決める。実効値は `check` が出す。
- 歩容は開ループ。

**ここから先（モータが力を出す段階）は
[`bringup_checklist_keel.md`](bringup_checklist_keel.md) に分けてある。**
脚を浮かせて励磁 → ゲインを上げる → 接地 → 歩く、の順に合格条件と
「止める条件」を並べたもの。

RT 優先度（`chrt`）は**立ち上げ中は使わない** — 暴走したプロセスを殺しにくく
なる。`run` の状態行に出る「遅延最大」を見てから判断する。ブリッジ側は
`tools/setup_rt.sh` で `cap_sys_nice,cap_ipc_lock` を付ける流儀なので、
`misa-run` にも必要になったら同じ手が使える。

---

## 8. 兄弟クレートを実機で直したくなったら

`./scripts/dev-siblings.sh` で `.cargo/config.toml` に `[patch]` を書き出せば
ローカルのチェックアウトを見る（`--off` で戻る）。**実機では基本使わない** —
PC で直して push、実機は `git pull && cargo build` が筋。
「直したのに変わらない」の原因はたいていこれ。

---

## 9. 未確認

| 件 | 状態 |
|---|---|
| 機体の前提（apt / rust / libclang / ROS） | ✅ 2026-09-03 に機体で確認。`libclang-dev` の追加が要った |
| humble / aarch64 でのビルド | ✅ **2026-09-03 に機体で通った**（`--features ros2`、misa_msgs は humble で 10.5 秒）。所要時間は未計測 |
| ブリッジの ws の場所 | ✅ `misa-runner` の隣（`../ksm_mvp_real_ws/install`）。自動探索が拾う |
| モデルの置き方 | 機体には rsync で `../keel/model/...` に置いた（§4 の a）。**リポジトリには同梱していないので、機体を作り直すとまた要る** |
| `IDL_PACKAGE_FILTER` を絞ったビルド | ✅ PC で確認（`--features ros2` が 2 分 55 秒で通り、`check` も通る） |
| ブリッジとの実通信 | ✅ **2026-09-04 に `run` が通った。** `low_state` 0.0〜1.3 ms 前、200 Hz、遅延最大 0.0 ms、脱力のまま安定。**車輪 4 軸は `/low_state` に載らない**（脚 12 本のみ） |
| RMW / `ROS_DOMAIN_ID` | ✅ 2026-09-04 に `/low_state` が見えたので、ブリッジ側と揃っている |
