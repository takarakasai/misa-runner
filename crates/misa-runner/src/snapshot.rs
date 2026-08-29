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

use std::collections::BTreeMap;
use std::time::Duration;

use misa_core::{
    Axis, AxisCommand, AxisId, AxisLimits, AxisRole, AxisTable, Command, ControlMode, Imu,
    Observation, SafetyConfig, Time,
};
use misa_hal::joint::LegSlot;
use misa_hal::imu::ImuSample;
use misa_hal::joint::{JointState, JOINT_NAMES};
use misa_hal::legs::JointStatus;

use crate::config::{AppConfig, AuxRole};
use crate::jointvec::JointVec;

/// 軸の並びと、そこでの役割の引き当て。
///
/// **並びは「脚 12 軸のあとに補助軸」で固定。** 脚が 4×3 なのは歩容
/// （`quadruped-gait`）がそう作られているからで、そこは動かせない。動くのは
/// 補助軸の本数と役割で、それはプロファイルの `[[aux]]` が決める。
#[derive(Debug, Clone)]
pub struct AxisLayout {
    pub table: AxisTable,
    /// チキンヘッドが動かす軸。無い機体では `None`。
    pub head: Option<AxisId>,
}

impl AxisLayout {
    /// 補助軸の並び（軸表での添字）。プロファイルの `[[aux]]` と同じ順。
    pub fn aux(&self) -> Vec<AxisId> {
        self.table.aux()
    }

    /// 脚の軸数。**歩容が触る範囲**で、ここまでは常に先頭に並ぶ。
    pub const LEG_AXES: usize = 12;

    /// **こちらが指令を出す軸の観測が、全部読めているか。**
    ///
    /// 待つのは指令を出す軸だけ。keel の車輪のように**観測だけの軸は
    /// 相手が出していないことがある**（`/low_state` は脚 12 本しか載せない）。
    /// それを待つと永久に立ち上がれない。実際そうなった (2026-09-04)。
    ///
    /// 逆に、指令を出す軸が読めていないうちに立たせてはいけない。観測は
    /// 0 のままで、脱力からの遷移はそれを実測として始点に張る。
    pub fn commanded_axes_read(&self, obs: &misa_core::Observation) -> bool {
        let head = self.head;
        (0..Self::LEG_AXES)
            .map(|i| AxisId::new(i as u16))
            .chain(head)
            .all(|id| obs.get(id).is_some_and(|a| a.health.valid))
    }
}

/// プロファイルから軸の並びを組む。
pub fn axis_layout(cfg: &AppConfig) -> Result<AxisLayout, String> {
    let mut axes = Vec::with_capacity(AxisLayout::LEG_AXES + cfg.aux.len());
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
    let mut head = None;
    for (i, a) in cfg.aux.iter().enumerate() {
        if a.role == AuxRole::Head {
            head = Some(AxisId::new((AxisLayout::LEG_AXES + i) as u16));
        }
        axes.push(Axis {
            name: a.joint.clone(),
            role: AxisRole::Aux,
        });
    }
    let table = AxisTable::new(axes)?;
    Ok(AxisLayout { table, head })
}

/// 実機設定から [`SafetyConfig`] を組む。
///
/// 可動域とスルーレート制限は、いまも HAL が持っている値そのもの。ここで
/// **同じ値を上の層にも見せる**ことで、丸めた理由を [`misa_core::SafetyVerdict`]
/// として説明できるようにする。HAL 側は最後の防波堤として残す。
///
/// `max_observation_age` は制御周期から決める。バスが遅れて読み戻しが
/// 止まったことを、周期いくつぶんで「見えていない」と判断するか。
/// `model_limits` はモデルが宣言している可動域（[`crate::robot::Robot::limits`]）。
///
/// **優先するのはプロファイルの実測値。** 校正で確定した値のほうがモデルの
/// 設計値より実機に近い。モデルは、校正値を PC が持たない機体（ブリッジ越し）
/// と補助軸の受け皿になる。どちらも無ければ制限しない。
pub fn safety_config(
    cfg: &AppConfig,
    layout: &AxisLayout,
    model_limits: &BTreeMap<String, (f64, f64)>,
    model_rates: &BTreeMap<String, f64>,
    control_period_s: f64,
    stale_ticks: f64,
) -> SafetyConfig {
    // **設定とモデルの定格の、厳しいほう。**
    //
    // 設定側は「目標をどれだけ穏やかに動かすか」という運用の判断で、
    // モデル側は「モータが何 rad/s まで回るか」という物理の事実。
    // 定格より速い目標は追えないので、どちらも超えないところで丸める。
    let rate_of = |id: AxisId| -> f64 {
        let cfg_rate = cfg.hardware.max_target_rate_rad_s();
        let model = layout
            .table
            .name(id)
            .and_then(|n| model_rates.get(n).copied())
            .unwrap_or(0.0);
        match (cfg_rate > 0.0, model > 0.0) {
            (true, true) => cfg_rate.min(model),
            (true, false) => cfg_rate,
            (false, true) => model,
            (false, false) => 0.0,
        }
    };
    let mut axes = Vec::with_capacity(layout.table.len());
    // **可動域の出どころは構成で違う。** バスを直接握る構成は校正で採った
    // 実測値を持つのでそれを使い、ブリッジ越しの機体はモデルの
    // `[joint.limit]` から埋める（向こうも持っているが、こちらで丸めてから
    // 出す。二重にかかるのは無駄ではない）。
    let serial = cfg.hardware.serial().ok();
    for leg in LegSlot::ALL {
        match serial.and_then(|h| h.bus_for(leg)) {
            Some(bus) => {
                for (joint, m) in bus.motors.iter().enumerate() {
                    let id = AxisId::new((leg.index() * 3 + joint) as u16);
                    axes.push(AxisLimits {
                        min_rad: m.min_rad,
                        max_rad: m.max_rad,
                        max_target_rate_rad_s: rate_of(id),
                        max_torque_nm: 0.0,
                    });
                }
            }
            None => {
                // 実測値が無いのでモデルの宣言を使う。
                for joint in 0..3 {
                    let id = AxisId::new((leg.index() * 3 + joint) as u16);
                    let (min_rad, max_rad) = from_model(layout, model_limits, id);
                    axes.push(AxisLimits {
                        min_rad,
                        max_rad,
                        max_target_rate_rad_s: rate_of(id),
                        max_torque_nm: 0.0,
                    });
                }
            }
        }
    }
    // 補助軸の可動域。いまは腕の設定しか持っていないので、**head だけ**
    // その値を使い、ほかはモデルの可動域が入るまで無制限にしておく。
    // 無制限が危ないのは駆動する軸だけで、駆動しない軸は指令が出ない。
    for id in layout.aux() {
        let arm = (layout.head == Some(id)).then(|| serial.map(|h| &h.arm)).flatten();
        let (min_rad, max_rad) = match arm {
            Some(a) => (a.min_rad, a.max_rad),
            None => from_model(layout, model_limits, id),
        };
        axes.push(AxisLimits {
            min_rad,
            max_rad,
            max_target_rate_rad_s: rate_of(id),
            max_torque_nm: 0.0,
        });
    }
    SafetyConfig {
        axes,
        max_observation_age: std::time::Duration::from_secs_f64(
            control_period_s * stale_ticks.max(1.0),
        ),
        max_tilt_rad: cfg.max_tilt_rad(),
    }
}

/// 軸の可動域をモデルの宣言から引く。宣言が無ければ無制限。
fn from_model(
    layout: &AxisLayout,
    model_limits: &BTreeMap<String, (f64, f64)>,
    id: AxisId,
) -> (f64, f64) {
    layout
        .table
        .name(id)
        .and_then(|n| model_limits.get(n))
        .copied()
        .unwrap_or((f64::NEG_INFINITY, f64::INFINITY))
}

/// 実機の読み戻しを [`Observation`] へ。
///
/// `arm_rad` は腕の角度。受信機直結の構成では**観測値**が入る。
/// `imu_age` は IMU サンプルを受け取ってからの経過。
pub fn observation(
    layout: &AxisLayout,
    time: Time,
    states: &[[JointState; 3]; 4],
    status: &[[JointStatus; 3]; 4],
    head_rad: f64,
    imu: &ImuSample,
    imu_age: Duration,
) -> Observation {
    let mut obs = Observation::empty(layout.table.len(), 4);
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
    // head は駆動していてもいなくても角度は分かる（受信機直結でも観測できる）。
    // ほかの補助軸はまだ読む口が無いので、未取得のまま残す。
    if let Some(id) = layout.head {
        if let Some(a) = obs.get_mut(id) {
            a.position_rad = head_rad;
            a.health.valid = true;
        }
    }

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
pub fn command(
    layout: &AxisLayout,
    targets: &JointVec,
    max_speed_rad_s: f64,
    relaxed: bool,
    gains: Option<misa_hal::config::MitGains>,
) -> Command {
    let mut cmd = Command::idle(layout.table.len());
    let mode = if relaxed {
        ControlMode::Idle
    } else {
        ControlMode::Position
    };
    for leg in 0..4 {
        for k in 0..3 {
            let id = AxisId::new((leg * 3 + k) as u16);
            let mut a = AxisCommand {
                mode,
                ..AxisCommand::position(targets.legs[leg][k], max_speed_rad_s)
            };
            // **MIT の機体には kp/kd を毎周期載せる。** 無いと τ が恒等的に
            // 0 になり、位置を指令しているのに脱力したまま崩れる。
            // シリアルの機体はサーボが内部で持つので `None`。
            if let Some(g) = gains {
                let (kp, kd) = g.for_joint(k);
                a.kp_nm_per_rad = kp;
                a.kd_nm_s_per_rad = kd;
            }
            *cmd.get_mut(id).expect("軸表と長さが揃っている") = a;
        }
    }
    // **head 以外の補助軸には指令を出さない。** 車輪を動かすのは歩容の
    // 仕事ではないので、脱力のまま残す（駆動する主体ができたらそこが書く）。
    if let Some(id) = layout.head {
        if let Some(a) = cmd.get_mut(id) {
            *a = AxisCommand {
                mode,
                ..AxisCommand::position(targets.arm, max_speed_rad_s)
            };
            if let Some(g) = gains {
                // ヘッドは脚ではないので、いちばん軽い hip の値を借りる。
                let (kp, kd) = g.for_joint(0);
                a.kp_nm_per_rad = kp;
                a.kd_nm_s_per_rad = kd;
            }
        }
    }
    cmd
}

#[cfg(test)]
mod tests {
    use super::*;
    use misa_hal::joint::ARM_JOINT_NAME;

    fn layout() -> AxisLayout {
        axis_layout(&AppConfig::default()).unwrap()
    }

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


    /// keel を想定した並び。脚 12 + 車輪 4、head 無し。
    fn wheeled_layout() -> AxisLayout {
        let mut cfg = AppConfig::default();
        cfg.aux = ["FL_wheel_joint", "FR_wheel_joint", "RL_wheel_joint", "RR_wheel_joint"]
            .iter()
            .map(|n| crate::config::AuxAxis {
                joint: (*n).into(),
                role: AuxRole::Wheel,
            })
            .collect();
        axis_layout(&cfg).unwrap()
    }

    /// **補助軸の本数が違う機体が入ること。**
    ///
    /// namiashi は腕 1 軸、keel は車輪 4 軸。ここが固定だと 2 台目が載らない。
    /// **待つのは指令を出す軸だけ。**
    ///
    /// keel の車輪は観測だけの軸で、ブリッジの `/low_state` は脚 12 本しか
    /// 載せない。それを待つと `状態を受け取りました` が永久に出ず、立ち
    /// 上がれない。実機で実際に詰まった (2026-09-04)。
    #[test]
    fn observe_only_axes_do_not_block_startup() {
        let lay = layout();
        let mut obs = misa_core::Observation::empty(lay.table.len(), 4);
        assert!(!lay.commanded_axes_read(&obs), "何も読めていなければ待つ");
        // 脚 12 本（と head）だけ埋める。補助軸は未取得のまま。
        for i in 0..AxisLayout::LEG_AXES {
            obs.get_mut(AxisId::new(i as u16)).unwrap().health.valid = true;
        }
        if let Some(id) = lay.head {
            obs.get_mut(id).unwrap().health.valid = true;
        }
        assert!(
            lay.commanded_axes_read(&obs),
            "観測だけの軸が来ていなくても立ち上がれること"
        );
    }

    /// 逆に、**指令を出す軸が 1 本でも欠けていたら待つ。**
    #[test]
    fn a_missing_leg_axis_still_blocks_startup() {
        let lay = layout();
        let mut obs = misa_core::Observation::empty(lay.table.len(), 4);
        for i in 0..lay.table.len() {
            obs.get_mut(AxisId::new(i as u16)).unwrap().health.valid = true;
        }
        assert!(lay.commanded_axes_read(&obs));
        obs.get_mut(AxisId::new(5)).unwrap().health.valid = false;
        assert!(!lay.commanded_axes_read(&obs), "脚が 1 本欠けたら待つ");
    }

    /// **ブリッジ越しの機体には kp/kd を載せる。**
    ///
    /// 載せ忘れると MIT の τ が恒等的に 0 になり、位置を指令しているのに
    /// 機体は脱力したまま崩れる。**ログには「指令どおり出している」と残る**
    /// ので、実機の前で原因を探すことになる。
    #[test]
    fn a_bridge_command_carries_the_mit_gains() {
        let g = misa_hal::config::MitGains {
            kp: [40.0, 60.0, 70.0],
            kd: [1.0, 1.5, 2.0],
        };
        let cmd = command(&layout(), &JointVec::zeros(), 8.0, false, Some(g));
        let t = layout().table;
        for (name, kp, kd) in [
            ("FL_hip_joint", 40.0, 1.0),
            ("FL_thigh_joint", 60.0, 1.5),
            ("FL_calf_joint", 70.0, 2.0),
        ] {
            let a = cmd.get(t.id_of(name).unwrap()).unwrap();
            assert_eq!((a.kp_nm_per_rad, a.kd_nm_s_per_rad), (kp, kd), "{name}");
        }
    }

    /// **シリアルの機体には載せない。** あちらはサーボが内部で持つ。
    #[test]
    fn a_serial_command_leaves_the_gains_alone() {
        let cmd = command(&layout(), &JointVec::zeros(), 8.0, false, None);
        let a = cmd.get(AxisId::new(0)).unwrap();
        assert_eq!((a.kp_nm_per_rad, a.kd_nm_s_per_rad), (0.0, 0.0));
    }

    /// **既定は kp 120 / kd 2.0。** 立ち上げ用の柔らかい値で、この値では
    /// keel は立てない（MuJoCo で同じ制御則を回すと胴体が 0.250 まで沈む。
    /// 足だけで立つのは kp 300 から）。**歩かせる前に上げること。**
    #[test]
    fn the_default_bridge_gains_are_the_soft_bring_up_pair() {
        let h = misa_hal::config::Ros2Hardware::default();
        assert_eq!(h.mit_gains.kp, [120.0; 3]);
        assert_eq!(h.mit_gains.kd, [2.0; 3]);
        assert!(h.mit_gains.validate().is_ok());
    }

    /// **kp が全部 0 の設定は弾く。** 0 は脱力と区別が付かない。
    #[test]
    fn all_zero_gains_are_refused() {
        let g = misa_hal::config::MitGains {
            kp: [0.0; 3],
            kd: [1.0; 3],
        };
        let e = g.validate().unwrap_err();
        assert!(e.contains("脱力"), "{e}");
        assert!(misa_hal::config::MitGains {
            kp: [40.0, 60.0, 60.0],
            kd: [1.0, 1.5, 1.5],
        }
        .validate()
        .is_ok());
    }

    #[test]
    fn a_robot_with_four_wheels_instead_of_an_arm_fits() {
        let lay = wheeled_layout();
        assert_eq!(lay.table.len(), 16);
        assert_eq!(lay.aux().len(), 4);
        assert_eq!(lay.head, None);
        assert_eq!(lay.table.name(AxisId::new(15)), Some("RR_wheel_joint"));
        // 脚の 12 軸は先頭のまま。歩容が触る範囲は動かない。
        assert_eq!(lay.table.legs().len(), 4);
        assert_eq!(lay.table.name(AxisId::new(0)), Some("FL_hip_joint"));
    }

    /// **歩容は車輪に指令を出さない。**
    ///
    /// 車輪を回すのは歩容の仕事ではないので、駆動する主体ができるまで
    /// 脱力のまま残す。ここが Position で埋まると、立った瞬間に車輪が
    /// 0 rad へ動く。
    #[test]
    fn the_gait_leaves_wheels_relaxed() {
        let lay = wheeled_layout();
        let mut q = JointVec::zeros();
        q.legs[0][1] = 0.9;
        let cmd = command(&lay, &q, 8.0, false, None);

        assert_eq!(cmd.len(), 16);
        assert_eq!(cmd.get(AxisId::new(1)).unwrap().mode, ControlMode::Position);
        for id in lay.aux() {
            assert_eq!(
                cmd.get(id).unwrap().mode,
                ControlMode::Idle,
                "{:?} に指令が出ている",
                lay.table.name(id)
            );
        }
    }

    /// **head が無い機体でも観測は組める。**
    #[test]
    fn an_observation_for_a_robot_without_a_head_is_still_full_length() {
        let lay = wheeled_layout();
        let obs = observation(
            &lay,
            Time::ZERO,
            &[[JointState::default(); 3]; 4],
            &[[JointStatus::default(); 3]; 4],
            0.0,
            &imu_sample(),
            Duration::ZERO,
        );
        assert_eq!(obs.len(), 16);
        // 車輪はまだ読む口が無いので未取得のまま。0 rad を実測と取り違えない。
        for id in lay.aux() {
            assert!(!obs.get(id).unwrap().health.valid);
        }
    }

    /// **プロファイルに可動域が無い機体は、モデルの宣言を使う。**
    ///
    /// ブリッジ越しの機体は校正値を PC が持たない。ここが無制限のままだと
    /// ゲートが何も丸めず、`dump` も「すべて範囲内」と言ってしまう。
    #[test]
    fn a_robot_without_calibration_takes_its_limits_from_the_model() {
        let mut cfg = AppConfig::default();
        cfg.hardware = misa_hal::config::HardwareConfig::Ros2(Default::default());
        let lay = axis_layout(&cfg).unwrap();

        let mut model = BTreeMap::new();
        model.insert("FL_thigh_joint".to_string(), (-2.5, 2.5));
        let sc = safety_config(&cfg, &lay, &model, &Default::default(), 0.005, 5.0);

        let id = lay.table.id_of("FL_thigh_joint").unwrap();
        assert_eq!(sc.axes[id.index()].min_rad, -2.5);
        assert_eq!(sc.axes[id.index()].max_rad, 2.5);
        // 宣言が無い軸は無制限のまま。0 rad に固定しない。
        let other = lay.table.id_of("FL_hip_joint").unwrap();
        assert!(sc.axes[other.index()].min_rad.is_infinite());
    }

    /// **実測値があるほうを優先する。** 校正で確定した値のほうが実機に近い。
    #[test]
    fn a_calibrated_axis_keeps_its_measured_range_over_the_models() {
        let cfg = AppConfig::default();
        let lay = axis_layout(&cfg).unwrap();
        let mut model = BTreeMap::new();
        model.insert("FL_hip_joint".to_string(), (-9.9, 9.9));
        let sc = safety_config(&cfg, &lay, &model, &Default::default(), 0.005, 5.0);

        let sh = cfg.hardware.serial().unwrap();
        let m = &sh.bus_for(LegSlot::Fl).unwrap().motors[0];
        assert_eq!(sc.axes[0].min_rad, m.min_rad, "モデルの値に上書きされている");
    }

    /// **チキンヘッドの相手が 2 本ある設定は弾く。**
    #[test]
    fn two_head_axes_are_rejected() {
        let mut cfg = AppConfig::default();
        cfg.aux = ["a_joint", "b_joint"]
            .iter()
            .map(|n| crate::config::AuxAxis {
                joint: (*n).into(),
                role: AuxRole::Head,
            })
            .collect();
        let e = cfg.validate().unwrap_err();
        assert!(e.contains("head"), "{e}");
    }

    /// **軸表と、Observation / Command の長さが揃っていること。**
    ///
    /// ここがずれると、名前で引いた添字が別の軸を指す。症状は
    /// 「片脚だけ挙動がおかしい」になり、原因に辿り着きにくい。
    #[test]
    fn the_axis_table_matches_the_vectors_it_indexes() {
        let t = layout().table;
        assert_eq!(t.len(), 13);
        assert_eq!(Command::idle(t.len()).len(), t.len());
        assert_eq!(Observation::empty(t.len(), 4).len(), t.len());
    }

    /// 軸表の並びが [`JointVec`] の並びと一致していること。
    #[test]
    fn the_axis_order_follows_the_joint_vector() {
        let t = layout().table;
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
        let cmd = command(&layout(), &q, 8.0, false, None);

        let t = layout().table;
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
        let cmd = command(&layout(), &q, 8.0, true, None);
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
            &layout(),
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

    /// **同梱プロファイルから制限表が組めること。**
    ///
    /// 軸表と長さが揃い、可動域が設定の値そのものであること。ここがずれると
    /// ゲートが別の軸の可動域で丸める。
    #[test]
    fn the_shipped_profile_yields_one_limit_per_axis() {
        let text = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../robots/namiashi.toml"
        ))
        .unwrap();
        let cfg = crate::config::AppConfig::from_toml(&text).unwrap();
        let lay = axis_layout(&cfg).unwrap();
        let sc = safety_config(&cfg, &lay, &BTreeMap::new(), &Default::default(), 1.0 / cfg.control.rate_hz, 5.0);
        assert_eq!(sc.axes.len(), lay.table.len());

        // FL の hip は設定の 1 本目のバスの 1 個目のモータ。
        let sh = cfg.hardware.serial().unwrap();
        let m = &sh.bus_for(LegSlot::Fl).unwrap().motors[0];
        assert_eq!(sc.axes[0].min_rad, m.min_rad);
        assert_eq!(sc.axes[0].max_rad, m.max_rad);
        assert_eq!(
            sc.axes[0].max_target_rate_rad_s,
            cfg.hardware.max_target_rate_rad_s()
        );
        // head の可動域は腕の設定から来る。
        let head = lay.head.expect("同梱プロファイルは head を持つ");
        assert_eq!(sc.axes[head.index()].min_rad, sh.arm.min_rad);
    }


}
