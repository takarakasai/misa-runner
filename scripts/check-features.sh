#!/usr/bin/env bash
# feature の組み合わせを総当たりでビルドする。
#
# **`--features render` だけで試していると、`--features sim` 単体が壊れて
# いても気づかない。** 実際に起きた: `render` の下でだけ `mut` が付く変数を
# `&mut` で借りていて、`sim` 単体のビルドが落ちていた (2026-09-05)。
# `render = ["sim", ...]` のように feature が積み上がる形だと、上だけ見て
# 下が壊れる。
#
#   ./scripts/check-features.sh          既定の組み合わせ
#   ./scripts/check-features.sh --test   ビルドに加えてテストも走らせる
#
# `ros2` は ROS 2 と独自 msg が source されていないと落ちる。**それは環境の
# 問題なので、source されていなければ飛ばす**（黙って成功にはしない）。
set -uo pipefail

cd "$(dirname "$0")/.."

DO_TEST=0
[ "${1:-}" = "--test" ] && DO_TEST=1

# MuJoCo を使う feature に要る。無ければ sim / render は飛ばす。
MUJOCO="${MUJOCO_DYNAMIC_LINK_DIR:-$HOME/.mujoco/mujoco-3.8.0/lib}"
if [ -d "$MUJOCO" ]; then
  export MUJOCO_DYNAMIC_LINK_DIR="$MUJOCO"
  export LD_LIBRARY_PATH="$MUJOCO:${LD_LIBRARY_PATH:-}"
  HAVE_MUJOCO=1
else
  HAVE_MUJOCO=0
  echo "⚠ MuJoCo が $MUJOCO に無いので sim / render は飛ばします" >&2
fi

HAVE_ROS=0
if [ -n "${ROS_DISTRO:-}" ] && [ -n "${AMENT_PREFIX_PATH:-}" ]; then
  HAVE_ROS=1
else
  echo "⚠ ROS 2 が source されていないので ros2 は飛ばします" >&2
fi

fail=0
skip=0
run() {
  local label="$1"; shift
  printf "  %-34s " "$label"
  local out rc
  out=$(cargo build --quiet --offline "$@" 2>&1); rc=$?
  if [ $rc -ne 0 ]; then
    echo "★失敗"
    echo "$out" | grep -E "^error" -A4 | head -12 | sed 's/^/      /'
    fail=$((fail + 1))
    return
  fi
  if [ $DO_TEST = 1 ]; then
    out=$(cargo test --quiet --offline "$@" 2>&1); rc=$?
    if [ $rc -ne 0 ]; then
      echo "★テスト失敗"
      echo "$out" | grep -E "^(error|test result: FAILED|failures:)" -A4 | head -12 | sed 's/^/      /'
      fail=$((fail + 1))
      return
    fi
  fi
  echo "OK"
}

echo "feature の総当たり（$([ $DO_TEST = 1 ] && echo "ビルド + テスト" || echo "ビルドのみ")）"
run "（既定 = viz）"
run "--no-default-features" --no-default-features
if [ $HAVE_MUJOCO = 1 ]; then
  run "--features sim" --features sim
  run "--features render" --features render
  run "--no-default-features --features sim" --no-default-features --features sim
else
  skip=$((skip + 3))
fi
if [ $HAVE_ROS = 1 ]; then
  run "--features ros2" --features ros2
  [ $HAVE_MUJOCO = 1 ] && run "--features sim,ros2" --features sim,ros2 || skip=$((skip + 1))
else
  skip=$((skip + 2))
fi

echo
if [ $fail -eq 0 ]; then
  echo "全部通りました（飛ばした組み合わせ: $skip）"
  [ $skip -gt 0 ] && echo "**飛ばしたぶんは確認できていません。** 上の ⚠ を見てください"
  exit 0
fi
echo "$fail 組が落ちています"
exit 1
