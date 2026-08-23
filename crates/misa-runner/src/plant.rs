//! 実機を [`Plant`] にする。
//!
//! いまのところ namiashi の構成（CH348 上の RS485 脚バス ×4 + WitMotion IMU +
//! 補助軸のサーボ）だけ。keel の中間層 UDP と MuJoCo は同じトレイトの別実装に
//! なる。
//!
//! # S.BUS はここに入らない
//!
//! 操縦入力は観測ではなく**意図**なので、`Plant` ではなく [`crate::pilot`] の
//! 側。ポートの探索（`PortMap`）だけは両者で共有する — 探索はデバイスを
//! `open` するので、別々にやると 2 本目が自分自身の `EBUSY` で失敗する。

use misa_core::{
    AxisId, AxisTable, Command, ControlMode, Imu, Observation, Plant, PlantCaps,
};
use misa_hal::joint::{JointCommand, JointMode};

use crate::config::AppConfig;
use crate::runner::Hardware;
use crate::snapshot;

/// シリアルバス越しの実機。
pub struct SerialPlant {
    hw: Hardware,
    axes: AxisTable,
    caps: PlantCaps,
    /// 位置指令に添える軸の速度上限 [rad/s]。
    max_speed_rad_s: f64,
    /// 単調時刻の基準。**時刻は Plant が供給する**（`misa_core::Time`）。
    started: std::time::Instant,
}

impl SerialPlant {
    pub fn connect_with(cfg: &AppConfig, map: &misa_hal::ch348::PortMap) -> Result<Self, String> {
        let hw = Hardware::connect_with(cfg, map)?;
        let axes = snapshot::axis_table();

        // **腕は「繋がっている」と「こちらの指令で動く」が別。**
        // 受信機直結の腕は動いてはいるが、アプリの指令では動かない。
        let mut driven = vec![true; axes.len()];
        if let Some(last) = driven.last_mut() {
            *last = hw.arm.is_app_driven();
        }
        let caps = PlantCaps {
            // トルクの口は HAL に空いているが、実機で使ったことがない。
            // 使えると名乗るのは実際に回してから。
            modes: vec![ControlMode::Position],
            has_imu: true,
            // 足裏センサが無い。歩容の立脚相からの推定は観測ではないので
            // ここでは名乗らない。
            has_contacts: false,
            driven,
        };
        Ok(Self {
            hw,
            axes,
            caps,
            max_speed_rad_s: cfg.hardware.legs.default_max_speed_rad_s,
            started: std::time::Instant::now(),
        })
    }

    /// 状態表示のための直接アクセス。
    pub fn hw(&self) -> &Hardware {
        &self.hw
    }

    pub fn hw_mut(&mut self) -> &mut Hardware {
        &mut self.hw
    }
}

impl Plant for SerialPlant {
    fn axes(&self) -> &AxisTable {
        &self.axes
    }

    fn capabilities(&self) -> &PlantCaps {
        &self.caps
    }

    fn arm(&mut self) -> Result<(), String> {
        self.hw
            .legs
            .request_all(misa_hal::legs::BusRequest::Enable)
            .map_err(|e| e.to_string())
    }

    fn disarm(&mut self) -> Result<(), String> {
        self.hw
            .legs
            .request_all(misa_hal::legs::BusRequest::Disable)
            .map_err(|e| e.to_string())
    }

    fn exchange(&mut self, cmd: &Command, obs: &mut Observation) -> Result<(), String> {
        // ── 出す ──────────────────────────────────────────────
        let mut cmds = [[JointCommand::default(); 3]; 4];
        for leg in 0..4 {
            for k in 0..3 {
                let Some(a) = cmd.get(AxisId::new((leg * 3 + k) as u16)) else {
                    continue;
                };
                cmds[leg][k] = JointCommand {
                    mode: match a.mode {
                        ControlMode::Idle => JointMode::Idle,
                        _ => JointMode::Position,
                    },
                    position_rad: a.position_rad,
                    max_speed_rad_s: self.max_speed_rad_s,
                    torque_nm: a.torque_ff_nm,
                };
            }
        }
        self.hw.legs.set_all(&cmds);

        if self.hw.arm.is_app_driven() {
            if let Some(a) = cmd.get(AxisId::new(12)) {
                if a.mode != ControlMode::Idle {
                    if let Err(e) = self.hw.arm.set_position(a.position_rad) {
                        log::warn!("腕サーボへの指令に失敗: {e}");
                    }
                }
            }
        }

        // ── 受け取る ──────────────────────────────────────────
        obs.time = misa_core::Time::from_secs_f64(self.started.elapsed().as_secs_f64());
        let states = self.hw.legs.states();
        let mut status = [[misa_hal::legs::JointStatus::default(); 3]; 4];
        for (i, bus) in self.hw.legs.buses().iter().enumerate() {
            status[i] = bus.status();
        }
        let imu = self.hw.imu.sample_or_level();

        for leg in 0..4 {
            for k in 0..3 {
                let id = AxisId::new((leg * 3 + k) as u16);
                let s = &states[leg][k];
                let st = &status[leg][k];
                let Some(a) = obs.get_mut(id) else { continue };
                a.position_rad = s.position_rad;
                a.velocity_rad_s = s.velocity_rad_s;
                a.torque_nm = None;
                a.health.valid = s.ok;
                a.health.fault_raw = u32::from(st.error_raw);
                a.health.temperature_c = st.valid.then_some(st.temperature_c);
                a.health.voltage_v = st.valid.then_some(st.voltage_v);
            }
        }
        if let Some(a) = obs.get_mut(AxisId::new(12)) {
            a.position_rad = self.hw.arm.position();
            a.health.valid = true;
        }
        obs.imu = Some(Imu {
            rpy_rad: imu.rpy_rad,
            gyro_rad_s: imu.gyro_rad_s,
            accel_m_s2: imu.accel_m_s2,
            age: imu.stamp.elapsed(),
        });
        Ok(())
    }
}
