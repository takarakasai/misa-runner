//! 実装の型を [`misa_core`] の語彙へ写す。
//!
//! **まだ制御ループはこの語彙で回っていない。** ここにあるのは片方向の変換
//! だけで、[`crate::teleop::OperatorCommand`] や [`crate::jointvec::JointVec`]
//! はそのまま残っている。移行の途中で 2 つの表現が並ぶのは避けられないので、
//! **どちらが目的地かをはっきりさせる**ためにこの module を置いている。
//!
//! この変換が書けること自体が、語彙が実機の状態を表せている証拠になる。
//! 表せていない項目は下に「まだ埋まらないもの」として並べてあり、
//! `Plant` が入る段で埋まる。
//!
//! # まだ埋まらないもの
//!
//! - **軸ごとの古さ** — いまの HAL は軸ごとの更新時刻を持たず、
//!   [`misa_hal::joint::JointState::ok`]（直近のトランザクションが成功したか）
//!   しか無い。「一度でも読めたか」と「いま新しいか」がここで潰れている。
//!   `Plant::exchange` が tick を単位にした時点で分かれる。
//! - **接地** — 足裏センサが無いので実機からは取れない。歩容の立脚相から
//!   推定する手はあるが、それは観測ではなく計画なので `None` のままにする。
//! - **トルク** — 位置制御しか使っていないので、読めていても意味を持たない。

// **まだ誰も呼んでいない。** 呼ぶのは記録の背骨が入る段で、そこで毎周期
// この変換を通してログへ落とす。それまで dead_code になるのは織り込み済み
// なので、警告を消して他の警告が埋もれないようにしておく。
#![allow(dead_code)]

use std::time::Duration;

use misa_core::{
    Axis, AxisCommand, AxisId, AxisRole, AxisTable, Command, ControlMode, Imu, Intent,
    Observation, PoseSlot, Time, Velocity,
};
use misa_hal::imu::ImuSample;
use misa_hal::joint::{JointState, ARM_JOINT_NAME, JOINT_NAMES};
use misa_hal::legs::JointStatus;

use crate::jointvec::JointVec;
use crate::teleop::OperatorCommand;

/// 脚 12 軸 + 腕 1 軸の並び。[`JointVec`] と同じ順序。
///
/// この関数がロボットごとに変わる部分で、いずれプロファイルとモデルから
/// 組み立てる。いまは namiashi の構成をそのまま写している。
pub fn axis_table() -> AxisTable {
    let mut axes = Vec::with_capacity(13);
    for (leg, names) in JOINT_NAMES.iter().enumerate() {
        for (joint, name) in names.iter().enumerate() {
            axes.push(Axis {
                name: (*name).into(),
                role: AxisRole::Leg {
                    leg: leg as u8,
                    joint: joint as u8,
                },
            });
        }
    }
    axes.push(Axis {
        name: ARM_JOINT_NAME.into(),
        role: AxisRole::Aux,
    });
    AxisTable::new(axes).expect("同梱の軸表に重複がある")
}

/// 操縦指令を [`Intent`] へ。
pub fn intent(time: Time, cmd: &OperatorCommand) -> Intent {
    Intent {
        time,
        velocity: Velocity {
            vx_m_s: cmd.vx_m_s,
            vy_m_s: cmd.vy_m_s,
            wz_rad_s: cmd.wz_rad_s,
        },
        body_attitude_rad: cmd.body_attitude_rad,
        height_offset_m: cmd.height_offset_m,
        mode: cmd.mode,
        gait: cmd.gait,
        // **押し続けても 1 回**という立ち上がりの意味は上流が持っている。
        // どの枠を再生するかだけをここで番号に写す。
        play_pose: cmd
            .play_pose
            .then(|| PoseSlot(u8::from(cmd.play_alt))),
        stabilize_head: cmd.chicken_head,
        aux_rad: vec![cmd.arm_rad],
        link_ok: cmd.link_ok,
    }
}

/// 実機の読み戻しを [`Observation`] へ。
///
/// `arm_rad` は腕の角度。受信機直結の構成では**観測値**が入る。
/// `imu_age` は IMU サンプルを受け取ってからの経過。
pub fn observation(
    time: Time,
    states: &[[JointState; 3]; 4],
    status: &[[JointStatus; 3]; 4],
    arm_rad: f64,
    imu: &ImuSample,
    imu_age: Duration,
) -> Observation {
    let mut obs = Observation::empty(13, 4);
    obs.time = time;
    for leg in 0..4 {
        for k in 0..3 {
            let id = AxisId::new((leg * 3 + k) as u16);
            let s = &states[leg][k];
            let st = &status[leg][k];
            let a = obs.get_mut(id).expect("軸表と長さが揃っている");
            a.position_rad = s.position_rad;
            a.velocity_rad_s = s.velocity_rad_s;
            a.torque_nm = None;
            a.health.valid = s.ok;
            a.health.fault_raw = u32::from(st.error_raw);
            a.health.temperature_c = st.valid.then_some(st.temperature_c);
            a.health.voltage_v = st.valid.then_some(st.voltage_v);
        }
    }
    // 腕は駆動していてもいなくても角度は分かる。読めた扱いにする。
    let arm = obs.get_mut(AxisId::new(12)).expect("腕の軸がある");
    arm.position_rad = arm_rad;
    arm.health.valid = true;

    obs.imu = Some(Imu {
        rpy_rad: imu.rpy_rad,
        gyro_rad_s: imu.gyro_rad_s,
        accel_m_s2: imu.accel_m_s2,
        age: imu_age,
    });
    obs
}

/// 目標角のベクトルを位置指令の [`Command`] へ。
///
/// `relaxed` が true なら**モードだけ脱力に落とし、目標角は残す**。
/// 目標を 0 に潰すと復帰した瞬間に全軸が 0 rad へ飛ぶ。
pub fn command(targets: &JointVec, max_speed_rad_s: f64, relaxed: bool) -> Command {
    let mut cmd = Command::idle(13);
    let mode = if relaxed {
        ControlMode::Idle
    } else {
        ControlMode::Position
    };
    for leg in 0..4 {
        for k in 0..3 {
            let id = AxisId::new((leg * 3 + k) as u16);
            *cmd.get_mut(id).expect("軸表と長さが揃っている") = AxisCommand {
                mode,
                ..AxisCommand::position(targets.legs[leg][k], max_speed_rad_s)
            };
        }
    }
    *cmd.get_mut(AxisId::new(12)).expect("腕の軸がある") = AxisCommand {
        mode,
        ..AxisCommand::position(targets.arm, max_speed_rad_s)
    };
    cmd
}

#[cfg(test)]
mod tests {
    use super::*;
    use misa_core::{GaitSelect, ModeRequest};

    /// 試験用の IMU サンプル。`ImuSample` は受信時刻を持つので `Default` が無い。
    fn imu_sample() -> ImuSample {
        ImuSample {
            rpy_rad: [0.0; 3],
            gyro_rad_s: [0.0; 3],
            accel_m_s2: [0.0, 0.0, 9.80665],
            temperature_c: 0.0,
            stamp: std::time::Instant::now(),
        }
    }

    fn idle_operator() -> OperatorCommand {
        OperatorCommand::failsafe(GaitSelect::Crawl, ModeRequest::Relax)
    }

    /// **軸表と、Observation / Command の長さが揃っていること。**
    ///
    /// ここがずれると、名前で引いた添字が別の軸を指す。症状は
    /// 「片脚だけ挙動がおかしい」になり、原因に辿り着きにくい。
    #[test]
    fn the_axis_table_matches_the_vectors_it_indexes() {
        let t = axis_table();
        assert_eq!(t.len(), 13);
        assert_eq!(Command::idle(t.len()).len(), t.len());
        assert_eq!(Observation::empty(t.len(), 4).len(), t.len());
    }

    /// 軸表の並びが [`JointVec`] の並びと一致していること。
    #[test]
    fn the_axis_order_follows_the_joint_vector() {
        let t = axis_table();
        assert_eq!(t.name(AxisId::new(0)), Some("FL_hip_joint"));
        assert_eq!(t.name(AxisId::new(5)), Some("FR_calf_joint"));
        assert_eq!(t.name(AxisId::new(11)), Some("RR_calf_joint"));
        assert_eq!(t.name(AxisId::new(12)), Some(ARM_JOINT_NAME));
        assert_eq!(t.legs().len(), 4);
        assert_eq!(t.aux(), vec![AxisId::new(12)]);
    }

    /// 目標角が、脚の並びどおりに指令へ写ること。
    #[test]
    fn targets_map_onto_the_command_in_leg_order() {
        let mut q = JointVec::zeros();
        q.legs[2][1] = 0.75; // RL_thigh
        q.arm = -0.25;
        let cmd = command(&q, 8.0, false);

        let t = axis_table();
        let id = t.id_of("RL_thigh_joint").unwrap();
        assert_eq!(cmd.get(id).unwrap().position_rad, 0.75);
        assert_eq!(cmd.get(id).unwrap().mode, ControlMode::Position);
        assert_eq!(cmd.get(AxisId::new(12)).unwrap().position_rad, -0.25);
    }

    /// **脱力してもモードが落ちるだけで、目標角は残る。**
    #[test]
    fn relaxing_keeps_the_targets_it_was_holding() {
        let mut q = JointVec::zeros();
        q.legs[0][1] = 1.0;
        let cmd = command(&q, 8.0, true);
        let a = cmd.get(AxisId::new(1)).unwrap();
        assert_eq!(a.mode, ControlMode::Idle);
        assert_eq!(a.position_rad, 1.0);
    }

    /// **読めていない軸は valid が false で残ること。**
    ///
    /// 起動直後の 0 rad を実測の 0 rad と取り違えると、読み戻しが済む前の
    /// 姿勢でログと可視化が埋まる。
    #[test]
    fn axes_that_have_not_answered_are_marked_unread() {
        let mut states = [[JointState::default(); 3]; 4];
        let status = [[JointStatus::default(); 3]; 4];
        states[0][0].ok = true;
        states[0][0].position_rad = 0.5;

        let obs = observation(
            Time::ZERO,
            &states,
            &status,
            0.0,
            &imu_sample(),
            Duration::ZERO,
        );
        assert!(obs.get(AxisId::new(0)).unwrap().health.valid);
        assert!(!obs.get(AxisId::new(1)).unwrap().health.valid);
        assert!(obs.any_unread());
    }

    #[test]
    fn the_operator_command_becomes_an_intent() {
        let mut op = idle_operator();
        op.vx_m_s = 0.2;
        op.play_pose = true;
        op.play_alt = true;
        op.chicken_head = true;
        op.arm_rad = Some(0.4);

        let i = intent(Time::from_secs_f64(1.0), &op);
        assert_eq!(i.velocity.vx_m_s, 0.2);
        assert_eq!(i.play_pose, Some(PoseSlot(1)));
        assert!(i.stabilize_head);
        assert_eq!(i.aux_rad, vec![Some(0.4)]);
    }

    /// ポーズ再生が押されていなければ、どの枠も選ばれないこと。
    #[test]
    fn no_pose_is_selected_unless_the_switch_was_pressed() {
        let op = idle_operator();
        assert_eq!(intent(Time::ZERO, &op).play_pose, None);
    }
}
