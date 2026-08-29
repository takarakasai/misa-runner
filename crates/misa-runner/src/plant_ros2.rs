//! STM のブリッジへ ROS 2 で繋ぐ [`Plant`]。
//!
//! ```text
//!  publish   /low_command_msg   low_command_msgs/LowCommand   目標
//!  subscribe /low_state         low_state_msgs/LowState       現在 + IMU
//! ```
//!
//! # 指令はすべて MIT（インピーダンス）
//!
//! ブリッジは `MotorCommand.control_mode` を**見ていない**。生値をそのまま
//! `device_command_id` へ流すとファームウェアが解釈できないから、という理由で
//! 常に MIT として下ろす（`dap_driver_node.cpp` の `MakeAxisCommandDeviceData`）。
//!
//! したがって毎周期 `position / velocity / torque / kp / kd` を全部載せる。
//! **脱力は「モードを落とす」ではなく `kp = kd = torque = 0`**。position 制御に
//! 速度上限を添える namiashi とは、指令の作り方がそもそも違う。
//!
//! # QoS はブリッジ側に合わせる
//!
//! | | ブリッジ側 | こちら |
//! |---|---|---|
//! | `low_command_msg` | `QoS(1).best_effort()` | 同じ |
//! | `low_state` | SensorDataQoS + keep_last(1) + deadline 5 ms | best-effort / keep_last(1) |
//!
//! reliable にすると再送で遅延が跳ねる。制御ループでは古い指令が遅れて届く
//! ほうが有害。
//!
//! # 対応は関節名で取る
//!
//! `LowState.joint_names[i]` が `joint_states[i]` を名指しする。**最初の
//! 1 通で索引を解決して以後は使い回し**、並びが変わったら張り直す。
//! 毎周期名前で引くと 200 Hz × 16 軸ぶんのハッシュ引きが載る。
//!
//! # 往復の相関が無い
//!
//! `LowState` は**どの指令に対する状態かを持たない**（`ack_seq` が無い）。
//! したがって観測の古さは「こちらが受け取った時刻」でしか測れず、往復遅れも
//! 実測できない。ブリッジ側のメッセージに指令の連番の反響が入れば、
//! 中間層 UDP の Down/Up と同じ相関が取れる。

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use futures::StreamExt;
use misa_core::{
    AxisId, AxisTable, Command, ControlMode, Imu, Observation, Plant, PlantCaps,
};

use crate::config::AppConfig;

type LowState = r2r::low_state_msgs::msg::LowState;
type LowCommand = r2r::low_command_msgs::msg::LowCommand;
type MotorCommand = r2r::low_command_msgs::msg::MotorCommand;

/// spin スレッドが置き、制御ループが読む「最後に届いた状態」。
struct Latest {
    state: Option<LowState>,
    at: Option<Instant>,
}

pub struct Ros2Plant {
    axes: AxisTable,
    caps: PlantCaps,
    latest: Arc<Mutex<Latest>>,
    publisher: r2r::Publisher<LowCommand>,
    /// 軸表の並び → `LowState.joint_states` の添字。最初の 1 通で解決する。
    index: Vec<Option<usize>>,
    /// `index` を作ったときの名前の並び。変わったら張り直す。
    index_for: Vec<String>,
    /// 直近に報告した「解決できなかった軸」。
    ///
    /// **同じことを何度も言わない。** 名前の並びが 2 通り以上のあいだで
    /// 揺れると（別のノードが同じトピックへ出しているなど）、毎周期
    /// 張り直しになってログが溢れる。張り直し自体は正しいので続けるが、
    /// 報告は結果が変わったときだけにする。
    reported: Option<Vec<String>>,
    /// 状態がこれより古ければ観測を信じない。
    timeout: Duration,
    started: Instant,
    /// spin スレッドを止める合図。
    ///
    /// **止めずに落とすと落ちる。** ROS のコンテキストが片付いたあとに
    /// spin が走ると、解放済みの領域を触って malloc が壊れる（実測）。
    stop: Arc<AtomicBool>,
    spin: Option<std::thread::JoinHandle<()>>,
}

impl Ros2Plant {
    pub fn connect(cfg: &AppConfig, axes: AxisTable) -> Result<Self, String> {
        let r = match &cfg.hardware {
            misa_hal::config::HardwareConfig::Ros2(r) => r.clone(),
            _ => {
                return Err("この Plant は kind = \"ros2\" のプロファイルでのみ使えます".into())
            }
        };

        // **0 のゲインで繋がない。** ブリッジは MIT しか持たないので、
        // kp が 0 だと τ が恒等的に 0 になる。位置を指令しているのに機体は
        // 脱力したまま崩れ、**ログには「指令どおり出している」と残る**ので、
        // 実機の前で原因を探すことになる。既定は 120 / 2.0。
        r.mit_gains.validate()?;
        log::info!(
            "MIT ゲイン kp {:?} / kd {:?}（hip, thigh, calf）",
            r.mit_gains.kp,
            r.mit_gains.kd
        );

        let ctx = r2r::Context::create().map_err(|e| format!("ROS 2 の初期化に失敗: {e}"))?;
        let mut node = r2r::Node::create(ctx, &format!("{}_plant", r.node_name), &r.namespace)
            .map_err(|e| format!("ノードを作れません: {e}"))?;

        let qos = r2r::QosProfile::default().best_effort().keep_last(1);
        let publisher = node
            .create_publisher::<LowCommand>(&r.command_topic, qos.clone())
            .map_err(|e| format!("{} へ publish できません: {e}", r.command_topic))?;
        let state_sub = node
            .subscribe::<LowState>(&r.state_topic, qos)
            .map_err(|e| format!("{} を購読できません: {e}", r.state_topic))?;

        let latest = Arc::new(Mutex::new(Latest {
            state: None,
            at: None,
        }));
        let spin_latest = Arc::clone(&latest);
        let stop = Arc::new(AtomicBool::new(false));
        let spin_stop = Arc::clone(&stop);
        let spin = std::thread::Builder::new()
            .name("ros2-plant".into())
            .spawn(move || {
                let mut pool = futures::executor::LocalPool::new();
                {
                    use futures::task::LocalSpawnExt;
                    let mut sub = state_sub;
                    pool.spawner()
                        .spawn_local(async move {
                            while let Some(msg) = sub.next().await {
                                let mut l =
                                    spin_latest.lock().unwrap_or_else(|e| e.into_inner());
                                l.state = Some(msg);
                                l.at = Some(Instant::now());
                            }
                        })
                        .expect("spawn low_state");
                }
                while !spin_stop.load(Ordering::Relaxed) {
                    node.spin_once(Duration::from_millis(2));
                    pool.run_until_stalled();
                }
            })
            .map_err(|e| format!("ROS 2 の spin スレッドを起こせません: {e}"))?;

        log::info!(
            "STM のブリッジへ ROS 2 で繋ぎます: 指令 {} / 状態 {}",
            r.command_topic,
            r.state_topic
        );

        let n = axes.len();
        Ok(Self {
            caps: PlantCaps {
                // **MIT だけ。** ブリッジが control_mode を見ず、常に MIT として
                // 下ろすので、position だけを名乗ると嘘になる。
                modes: vec![ControlMode::Impedance],
                has_imu: true,
                // 足裏センサは LowState に無い。
                has_contacts: false,
                driven: vec![true; n],
            },
            axes,
            latest,
            publisher,
            index: vec![None; n],
            index_for: Vec::new(),
            reported: None,
            timeout: Duration::from_millis(r.state_timeout_ms.max(5)),
            started: Instant::now(),
            stop,
            spin: Some(spin),
        })
    }

    /// 軸表の並び → 相手の並び を作り直す。
    ///
    /// **名前で取る。** 添字の一致を仮定すると、相手が 1 軸落としただけで
    /// 全部が 1 つずれる。
    fn reindex(&mut self, names: &[String]) {
        self.index = self
            .axes
            .axes()
            .iter()
            .map(|a| names.iter().position(|n| *n == a.name))
            .collect();
        self.index_for = names.to_vec();

        let missing: Vec<String> = self
            .axes
            .axes()
            .iter()
            .zip(&self.index)
            .filter(|(_, i)| i.is_none())
            .map(|(a, _)| a.name.clone())
            .collect();
        if self.reported.as_ref() == Some(&missing) {
            return;
        }
        if missing.is_empty() {
            log::info!("{} 軸すべてを low_state で解決しました", self.index.len());
        } else {
            // **黙って 0 のままにしない。** どの軸が来ていないかを言う。
            log::warn!(
                "low_state に無い関節が {} 本あります: {missing:?}（相手が出しているのは {names:?}）",
                missing.len()
            );
        }
        self.reported = Some(missing);
    }
}

impl Plant for Ros2Plant {
    fn axes(&self) -> &AxisTable {
        &self.axes
    }

    fn capabilities(&self) -> &PlantCaps {
        &self.caps
    }

    fn arm(&mut self) -> Result<(), String> {
        // 投入・切断という概念がこのインタフェースには無い。脱力は
        // kp = kd = torque = 0 の指令として毎周期出る。
        Ok(())
    }

    fn disarm(&mut self) -> Result<(), String> {
        Ok(())
    }

    fn status_line(&self) -> String {
        let l = self.latest.lock().unwrap_or_else(|e| e.into_inner());
        match l.at {
            Some(t) => format!("low_state {:.1}ms前", t.elapsed().as_secs_f64() * 1e3),
            None => "low_state 未受信".to_string(),
        }
    }

    fn exchange(&mut self, cmd: &Command, obs: &mut Observation) -> Result<(), String> {
        // ── 出す ──────────────────────────────────────────────
        let mut msg = LowCommand {
            timestamp: r2r::builtin_interfaces::msg::Time::default(),
            joint_name: Vec::with_capacity(self.axes.len()),
            num_motors: self.axes.len() as u32,
            motor_commands: Vec::with_capacity(self.axes.len()),
        };
        for (i, a) in self.axes.axes().iter().enumerate() {
            let c = cmd.get(AxisId::new(i as u16)).copied().unwrap_or_default();
            msg.joint_name.push(a.name.clone());
            // **脱力は kp = kd = torque = 0。** MIT しか無いので、モードを
            // 落とすという表現がそもそも無い。
            let limp = c.mode == ControlMode::Idle;
            msg.motor_commands.push(MotorCommand {
                // ブリッジは見ないが、記録と相手側のログのために入れておく。
                control_mode: c.mode as i32,
                joint_angle: c.position_rad,
                // **速度上限をここへ入れてはいけない。** `Command` の
                // `velocity_rad_s` はシリアルサーボ向けの「上限」だが、MIT の
                // `joint_angular_velocity` は**目標速度 q̇_d** で、
                // `τ = kp(q_d − q) + kd(q̇_d − q̇) + τ_ff` にそのまま入る。
                // 上限（keel は 8 rad/s）を渡すと、kd を入れた瞬間に全関節が
                // その速度で回ろうとする。位置保持の目標速度は 0。
                joint_angular_velocity: 0.0,
                joint_tau: if limp { 0.0 } else { c.torque_ff_nm },
                kp: if limp { 0.0 } else { c.kp_nm_per_rad },
                kd: if limp { 0.0 } else { c.kd_nm_s_per_rad },
            });
        }
        self.publisher
            .publish(&msg)
            .map_err(|e| format!("{} へ publish できません: {e}", "low_command"))?;

        // ── 受け取る ──────────────────────────────────────────
        obs.time = misa_core::Time::from_secs_f64(self.started.elapsed().as_secs_f64());

        let (state, age) = {
            let l = self.latest.lock().unwrap_or_else(|e| e.into_inner());
            match (&l.state, l.at) {
                (Some(s), Some(at)) => (s.clone(), at.elapsed()),
                // **まだ 1 通も来ていない。** 前の値を配り続けるより、
                // 未取得のまま返して上位に判断させる。
                _ => return Ok(()),
            }
        };

        if state.joint_names != self.index_for {
            self.reindex(&state.joint_names);
        }

        // 古すぎる状態は「古い」と印を付けて渡す。捨てない — 直前の姿勢が
        // 分からなくなると、復帰の始点が作れない。
        let stale = age > self.timeout;
        for (i, src) in self.index.iter().enumerate() {
            let Some(a) = obs.get_mut(AxisId::new(i as u16)) else {
                continue;
            };
            let Some(m) = src.and_then(|j| state.joint_states.get(j)) else {
                a.health.valid = false;
                continue;
            };
            a.position_rad = m.joint_angle;
            a.velocity_rad_s = m.joint_angular_velocity;
            a.torque_nm = m.motor_output_torque_valid.then_some(m.motor_output_torque);
            a.health.valid = true;
            a.health.age = age;
            a.health.fault_raw = m.motor_error as u32;
            a.health.temperature_c = m
                .motor_temperature_valid
                .then_some(m.motor_temperature as f64);
            // LowState は電圧を持たない。電流はあるが、ここには置き場が無い。
            a.health.voltage_v = None;
            if m.motor_warning != 0 && !stale {
                log::debug!("{} に警告 {:#x}", self.axes.name(AxisId::new(i as u16)).unwrap_or("?"), m.motor_warning);
            }
        }

        let q = &state.base_orientation.orientation;
        obs.imu = Some(Imu {
            rpy_rad: quat_to_rpy(q.w, q.x, q.y, q.z),
            gyro_rad_s: [
                state.base_orientation.angular_velocity.x,
                state.base_orientation.angular_velocity.y,
                state.base_orientation.angular_velocity.z,
            ],
            accel_m_s2: [
                state.base_orientation.linear_acceleration.x,
                state.base_orientation.linear_acceleration.y,
                state.base_orientation.linear_acceleration.z,
            ],
            age,
        });
        Ok(())
    }
}

/// クォータニオン → `[roll, pitch, yaw]`。ZYX（yaw-pitch-roll）順。
fn quat_to_rpy(w: f64, x: f64, y: f64, z: f64) -> [f64; 3] {
    let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    // **端で NaN にしない。** 真上・真下を向くと asin の引数が 1 を僅かに
    // 超えることがある。
    let s = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0);
    let pitch = s.asin();
    let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
    [roll, pitch, yaw]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_level_orientation_is_all_zeros() {
        let rpy = quat_to_rpy(1.0, 0.0, 0.0, 0.0);
        assert!(rpy.iter().all(|v| v.abs() < 1e-12), "{rpy:?}");
    }

    /// 真上を向いても NaN にしない（asin の引数が 1 を僅かに超える）。
    #[test]
    fn pointing_straight_up_does_not_produce_nan() {
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let rpy = quat_to_rpy(h, 0.0, h, 0.0);
        assert!(rpy.iter().all(|v| v.is_finite()), "{rpy:?}");
        assert!((rpy[1] - std::f64::consts::FRAC_PI_2).abs() < 1e-9, "{rpy:?}");
    }

    #[test]
    fn a_quarter_turn_in_yaw_reads_back() {
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let rpy = quat_to_rpy(h, 0.0, 0.0, h);
        assert!((rpy[2] - std::f64::consts::FRAC_PI_2).abs() < 1e-9, "{rpy:?}");
    }
}

/// `bridge` — ブリッジとの往復を確認する。**脱力の指令しか出さない。**
///
/// `legs` と同じ位置づけの診断で、繋がっているか・何軸解決できたか・
/// 状態がどれだけ古いかだけを見る。`kp = kd = torque = 0` を出し続けるので、
/// 実機に繋いでも動かない。
pub fn diagnose(cfg: &AppConfig, seconds: Option<f64>) -> Result<(), String> {
    let layout = crate::snapshot::axis_layout(cfg)?;
    let n = layout.table.len();
    let mut plant = Ros2Plant::connect(cfg, layout.table.clone())?;
    let cmd = Command::idle(n);
    let mut obs = Observation::empty(n, 4);

    let period = Duration::from_secs_f64(1.0 / cfg.control.rate_hz);
    let forever = seconds.is_none();
    let deadline = Instant::now() + Duration::from_secs_f64(seconds.unwrap_or(0.0));
    let mut last_print = Instant::now() - Duration::from_secs(1);
    let mut ticks: u64 = 0;
    let mut seen = 0u64;

    println!("ブリッジの往復を確認します（指令は脱力のまま）");
    println!("t[s]    受信   最古[ms]  異常  {}", layout.table.name(AxisId::new(0)).unwrap_or(""));

    let started = Instant::now();
    while forever || Instant::now() < deadline {
        plant.exchange(&cmd, &mut obs)?;
        ticks += 1;
        if !obs.any_unread() {
            seen += 1;
        }
        if last_print.elapsed() >= Duration::from_secs(1) {
            let faults: Vec<String> = obs
                .faulted()
                .map(|(id, s)| {
                    format!(
                        "{}={:#x}",
                        layout.table.name(id).unwrap_or("?"),
                        s.health.fault_raw
                    )
                })
                .collect();
            let q0 = obs.get(AxisId::new(0)).map(|a| a.position_rad).unwrap_or(f64::NAN);
            println!(
                "{:6.1}  {:5.1}%  {:8.1}  {:4}  {q0:+.4}",
                started.elapsed().as_secs_f64(),
                100.0 * seen as f64 / ticks.max(1) as f64,
                obs.worst_age().as_secs_f64() * 1e3,
                faults.len(),
            );
            if !faults.is_empty() {
                println!("        異常: {}", faults.join(" "));
            }
            last_print = Instant::now();
        }
        std::thread::sleep(period);
    }
    println!(
        "\n{ticks} 周期のうち {seen} 周期で全軸の状態が揃いました（{:.1}%）",
        100.0 * seen as f64 / ticks.max(1) as f64
    );
    if seen == 0 {
        return Err(format!(
            "{} から状態が来ていません。ブリッジが動いているか、トピック名と QoS を確認してください",
            match &cfg.hardware {
                misa_hal::config::HardwareConfig::Ros2(r) => r.state_topic.clone(),
                _ => String::new(),
            }
        ));
    }
    Ok(())
}

impl Drop for Ros2Plant {
    /// **spin を止めてから落とす。** ROS のコンテキストが片付いたあとに
    /// spin が走ると、解放済みの領域を触って malloc が壊れる。
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.spin.take() {
            let _ = h.join();
        }
    }
}
