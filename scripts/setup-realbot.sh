#!/usr/bin/env bash
# keel の実機（機体の PC / ROS 2 humble / aarch64）で misa-run をビルドできる
# 状態にする。**モータは一切動かさない**（最後に走るのは `check` だけ）。
#
#   ./scripts/setup-realbot.sh                 前提を調べて、足りないものを表示
#   ./scripts/setup-realbot.sh --apt           足りない apt パッケージを入れる (sudo)
#   ./scripts/setup-realbot.sh --build         misa_msgs と misa-run をビルド
#   ./scripts/setup-realbot.sh --apt --build   まとめて
#
#   --lean          viz（zenoh）を外して軽くビルドする
#   --jobs N        rustc の並列数。**メモリが 8 GiB 以下なら 4 くらいに落とす**
#                   （PC の実測でピーク 3.2 GiB。並列を増やすとここが伸びる）
#
# 何も入れず・何もビルドせずに走らせるのが既定。**先に一度素で回して、
# 何が足りないかを読むこと。** 手順の全体は doc/realbot_build.md。
set -uo pipefail

cd "$(dirname "$0")/.."

DO_APT=0; DO_BUILD=0; LEAN=0; JOBS=""
while [ $# -gt 0 ]; do
  case "$1" in
    --apt) DO_APT=1 ;;
    --build) DO_BUILD=1 ;;
    --lean) LEAN=1 ;;
    --jobs) shift; JOBS="${1:-}" ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "不明な引数: $1（--help）" >&2; exit 2 ;;
  esac
  shift
done

fail=0
ok()   { echo "  ✅ $*"; }
warn() { echo "  ⚠  $*"; }
bad()  { echo "  ❌ $*"; fail=$((fail+1)); }

echo "── 1. 機械 ─────────────────────────────"
echo "  arch      $(uname -m)"
echo "  kernel    $(uname -r)"
echo "  cores     $(nproc)"
echo "  mem       $(awk '/MemTotal/ {printf "%.1f GiB", $2/1048576}' /proc/meminfo)"
echo "  disk      $(df -h . | awk 'NR==2 {print $4" 空き ("$6")"}')"
# target/ に 2.6 GiB、~/.cargo に 1 GiB ほど要る（PC の実測）。
avail_kb=$(df -k . | awk 'NR==2 {print $4}')
[ "$avail_kb" -lt 5000000 ] && warn "空きが 5 GiB を切っています（target/ だけで 2.6 GiB 使う）"

echo
echo "── 2. ROS 2 ────────────────────────────"
if [ -z "${ROS_DISTRO:-}" ]; then
  bad "ROS 2 が source されていません:  source /opt/ros/humble/setup.bash"
else
  ok "ROS_DISTRO=$ROS_DISTRO"
  case "$ROS_DISTRO" in
    foxy|galactic|humble|iron|jazzy|rolling) ;;
    *) bad "r2r 0.9 が知らない distro です（対応: foxy galactic humble iron jazzy rolling）" ;;
  esac
fi

echo
echo "── 3. apt パッケージ ───────────────────"
# libudev-dev   serialport（misa-hal は keel でも常にビルドされる）
# colcon        misa_msgs を作る
PKGS=(build-essential pkg-config git curl libudev-dev python3-colcon-common-extensions)
missing=()
for p in "${PKGS[@]}"; do
  if dpkg-query -W -f='${Status}' "$p" 2>/dev/null | grep -q "install ok installed"; then
    ok "$p"
  else
    missing+=("$p"); bad "$p が入っていません"
  fi
done
# **libclang は「パッケージ名」ではなく「dlopen できるか」で見る。** r2r は
# bindgen で rcl の束縛を作り、bindgen は clang-sys が実行時に探した
# libclang を使う。`libclang-dev`（libclang.so 付き）でも
# `libclang1-<N>`（libclang-<N>.so.<N> だけ）でも通るので、パッケージ名で
# 判定すると入っているのに「無い」と言うことになる。
if ldconfig -p 2>/dev/null | grep -qE "libclang(-[0-9]+)?\.so"; then
  ok "libclang（$(ldconfig -p | grep -oE "libclang(-[0-9]+)?\.so[^ ]*" | head -1)）"
else
  missing+=(libclang-dev); bad "libclang が見つかりません（bindgen が動きません）"
fi
if [ ${#missing[@]} -gt 0 ]; then
  echo
  echo "  sudo apt-get install -y ${missing[*]}"
  if [ "$DO_APT" = 1 ]; then
    echo "  → --apt が付いているので入れます"
    sudo apt-get install -y "${missing[@]}" && fail=$((fail-${#missing[@]}))
  fi
fi

echo
echo "── 4. Rust ─────────────────────────────"
# **1.88 以上。** 依存のうち time 0.3.55 / darling 0.23 / serde_with 3.22 が
# rust-version = 1.88 を宣言している（Cargo.lock を総なめして確認）。
# handover.md の「1.85 以上」はもう足りない。
if ! command -v rustc >/dev/null; then
  bad "rustc がありません。rustup で入れる:"
  echo "     curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y"
else
  rv="$(rustc -V | awk '{print $2}')"
  if [ "$(printf '1.88.0\n%s\n' "$rv" | sort -V | head -1)" = "1.88.0" ]; then
    ok "rustc $rv"
  else
    bad "rustc $rv — 1.88 以上が要る（rustup update stable）"
  fi
fi

echo
echo "── 5. 環境変数（scripts/realbot-env.sh）─"
# source して、misa_msgs とブリッジの ws が見えるかを確かめる。
# このスクリプトの中で source しても呼び出し元のシェルには残らないので、
# 使うときは自分で source すること。
# shellcheck source=/dev/null
source scripts/realbot-env.sh >/dev/null 2>&1
if [ -n "${KSM_WS:-}" ] && [ -d "${KSM_WS}/share/low_command_msgs" ]; then
  ok "low_command_msgs / low_state_msgs: $KSM_WS"
else
  bad "ブリッジの ws が見つかりません。KSM_WS=<ksm_mvp_real_ws>/install を指定"
fi
for m in sensor_msgs geometry_msgs; do
  found=0
  IFS=: read -ra prefixes <<<"${AMENT_PREFIX_PATH:-}"
  for pre in "${prefixes[@]}"; do [ -d "$pre/share/$m" ] && found=1 && break; done
  if [ "$found" = 1 ]; then ok "$m"; else
    bad "$m が AMENT_PREFIX_PATH にありません: sudo apt-get install ros-${ROS_DISTRO:-humble}-${m//_/-}"
  fi
done
if [ -d ros/install/misa_msgs ]; then ok "misa_msgs（ビルド済み）"; else
  warn "misa_msgs が未ビルド（--build で作ります）"
fi

echo
echo "── 6. モデル ───────────────────────────"
# robots/keel.toml の model は相対パス。**実機で見えていなければ check が落ちる。**
model="$(awk -F'"' '/^model *=/ {print $2; exit}' robots/keel.toml)"
if [ -f "$model" ]; then
  ok "$model"
  meshdir="$(dirname "$model")"
  if [ -d "$meshdir/mesh" ]; then ok "メッシュ同梱（sim も回せる）"; else
    warn "メッシュがありません。run には不要ですが check が 37 件の警告を出します"
  fi
else
  bad "モデルが見つかりません: $model  → doc/realbot_build.md「モデルを機体に置く」"
fi

echo
if [ "$DO_BUILD" != 1 ]; then
  echo "前提の確認だけ終わりました（NG $fail 件）。ビルドまで進めるなら --build"
  [ "$fail" -gt 0 ] && exit 1
  exit 0
fi
[ "$fail" -gt 0 ] && { echo "NG が $fail 件あるのでビルドしません"; exit 1; }

echo "── 7. misa_msgs ────────────────────────"
( cd ros && colcon build --packages-select misa_msgs ) || exit 1
# 生成物のパスが変わるので環境を張り直す。
# shellcheck source=/dev/null
source scripts/realbot-env.sh || exit 1

echo
echo "── 8. misa-run ─────────────────────────"
feat=(--features ros2)
[ "$LEAN" = 1 ] && feat=(--no-default-features --features ros2)
jobs_arg=()
[ -n "$JOBS" ] && jobs_arg=(--jobs "$JOBS")
echo "  cargo build --release ${feat[*]} ${jobs_arg[*]}"
cargo build --release "${feat[@]}" "${jobs_arg[@]}" || exit 1

echo
echo "── 9. check ────────────────────────────"
./target/release/misa-run check --robot robots/keel.toml || exit 1

cat <<'NEXT'

ビルド環境はできました。次は（doc/realbot_build.md §5）:

  source /opt/ros/humble/setup.bash && source scripts/realbot-env.sh
  ros2 topic hz /low_state                      ブリッジが出しているか
  ./target/release/misa-run bridge --robot robots/keel.toml --secs 10
                                                ← **指令は脱力のまま。動かない**
  ./target/release/misa-run dump   --robot robots/keel.toml

`run` に進む前に doc/realbot_build.md §6（MIT ゲインと初期姿勢）を読むこと。
**kp 120 では立てない。** 脚を浮かせて指令角へ寄るのを見るための値。
NEXT
