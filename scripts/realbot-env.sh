# keel の実機（STM ブリッジ）と話すための環境変数。**source して使う。**
#
#   source /opt/ros/humble/setup.bash        # 実機。PC は jazzy
#   source scripts/realbot-env.sh            # ブリッジの ws は自動で探す
#   KSM_WS=/opt/ksm_mvp_real/install source scripts/realbot-env.sh
#
# 何を入れるか:
#
#   AMENT_PREFIX_PATH   misa_msgs（このリポジトリ）と low_command_msgs /
#                       low_state_msgs（ブリッジ側）。**独自インタフェースなので
#                       ここに無いと r2r が型を生成できずビルドが落ちる。**
#   LD_LIBRARY_PATH     同じ 2 つの lib（実行時に typesupport を dlopen する）
#   IDL_PACKAGE_FILTER  r2r が束縛を作る msg パッケージを絞る。**未設定だと
#                       AMENT_PREFIX_PATH 上の全パッケージを bindgen にかける**
#                       ので、実機（4〜8 コア）では効く
#   PYTHONPATH          `ros2 service call` から misa_msgs の型を触るときだけ
#
# ROS_DOMAIN_ID / RMW_IMPLEMENTATION は**設定しない**。ブリッジと揃っていない
# と話せないが、正解を知っているのは向こうなので、ここで上書きすると
# 「繋がらない理由が 2 つになる」だけ。いまの値は最後に表示する。

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  echo "これは source して使います:  source scripts/realbot-env.sh" >&2
  exit 1
fi

_misa_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "⚠ ROS 2 が source されていません。先に:  source /opt/ros/humble/setup.bash" >&2
fi

# ── misa_msgs（このリポジトリ。colcon の成果物は追跡していないので各機で作る）──
_misa_msgs="$_misa_root/ros/install/misa_msgs"
if [ ! -d "$_misa_msgs" ]; then
  echo "⚠ misa_msgs が未ビルドです:  (cd ros && colcon build --packages-select misa_msgs)" >&2
else
  export AMENT_PREFIX_PATH="$_misa_msgs${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$_misa_msgs/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  for _sp in "$_misa_msgs"/lib/python3*/site-packages; do
    [ -d "$_sp" ] && export PYTHONPATH="$_sp${PYTHONPATH:+:$PYTHONPATH}"
  done
fi

# ── ブリッジ側の ws（low_command_msgs / low_state_msgs）──
#
# 実機では ksm_mvp_real_ws がその機体で colcon build されている。PC では
# ref/ 以下の参照用チェックアウト（.gitignore 済み）。merge-install なので
# install/ 直下が prefix。
if [ -z "${KSM_WS:-}" ]; then
  for _cand in \
    "$HOME/ksm_mvp_real_ws/install" \
    "$HOME/work/ksm_mvp_real_ws/install" \
    "/opt/ksm_mvp_real/install" \
    "$_misa_root/ref/ksm_mvp_real_ws/install"
  do
    [ -d "$_cand" ] && KSM_WS="$_cand" && break
  done
fi

if [ -z "${KSM_WS:-}" ] || [ ! -d "$KSM_WS" ]; then
  echo "⚠ ブリッジの ws が見つかりません。KSM_WS で install の prefix を指定してください" >&2
  echo "   例:  KSM_WS=~/ksm_mvp_real_ws/install source scripts/realbot-env.sh" >&2
elif [ ! -d "$KSM_WS/share/low_command_msgs" ]; then
  echo "⚠ $KSM_WS に low_command_msgs がありません（colcon build 済みの install を指してください）" >&2
else
  export KSM_WS
  export AMENT_PREFIX_PATH="$KSM_WS${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$KSM_WS/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

# ── r2r に生成させる msg パッケージ ──
#
# LowState が sensor_msgs/Imu を、cmd_vel が geometry_msgs/Twist を持つので
# その 2 つと std_msgs（Header）が要る。rcl_interfaces / builtin_interfaces /
# unique_identifier_msgs / action_msgs は r2r が自分で足す。
#
# **絞り方を変えると r2r が丸ごと再ビルドされる**（rerun-if-env-changed）。
# 付けたり外したりを往復させないこと。
export IDL_PACKAGE_FILTER="misa_msgs;low_command_msgs;low_state_msgs;geometry_msgs;sensor_msgs;std_msgs"

echo "ROS_DISTRO        ${ROS_DISTRO:-（未設定）}"
echo "RMW               ${RMW_IMPLEMENTATION:-（既定。ブリッジと揃っているか確認）}"
echo "ROS_DOMAIN_ID     ${ROS_DOMAIN_ID:-0（既定）}"
echo "misa_msgs         ${_misa_msgs}"
echo "ブリッジの ws     ${KSM_WS:-（無し）}"
echo "IDL_PACKAGE_FILTER ${IDL_PACKAGE_FILTER}"

unset _misa_root _misa_msgs _misa_msgs_sp _cand _sp
