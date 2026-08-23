//! 意図を作るもの。
//!
//! **チャンネル番号を知っているのはここだけ。** ここから先は「歩けと言われた」
//! 「脱力しろと言われた」しか見えないので、プロポ・台本・ゲームパッド・
//! ネットワークが同じ穴に入る。シミュレータと CI が台本を流し込めるのは
//! この分離の直接の効果。

use std::time::Duration;

use misa_core::{Intent, Pilot, Time};
use misa_hal::ch348::PortMap;
use misa_hal::sbus::{SbusReceiver, SbusState};

use crate::config::AppConfig;
use crate::teleop::Teleop;

/// プロポ（Futaba S.BUS）。
pub struct SbusPilot {
    rx: SbusReceiver,
    teleop: Teleop,
    timeout: Duration,
    /// 受信機が**そもそも無い**ベンチ用。
    ///
    /// 受信断（受信していたのに切れた）と混ぜてはいけない。前者は
    /// 起立させたい、後者は活動度を上げてはいけない。かつて前者が後者の
    /// 経路に乗っていたため、**脱力中に受信が切れると立ち上がっていた**。
    allow_no_sbus: bool,
}

impl SbusPilot {
    pub fn connect_with(
        cfg: &AppConfig,
        map: &PortMap,
        allow_no_sbus: bool,
    ) -> Result<Self, String> {
        let rx = SbusReceiver::connect_with(&cfg.hardware.sbus, map).map_err(|e| e.to_string())?;
        log::info!("S.BUS → {}", rx.port());
        Ok(Self {
            rx,
            teleop: Teleop::new(
                cfg.teleop.clone(),
                &cfg.gait,
                &cfg.hardware.arm,
            ),
            timeout: Duration::from_millis(cfg.control.teleop_timeout_ms),
            allow_no_sbus,
        })
    }

    /// 受信を待つ。プロポが無い状態で起立させないための入口チェック。
    pub fn wait_ready(&self, timeout: Duration) -> Result<(), String> {
        self.rx.wait_ready(timeout).map(|_| ()).map_err(|e| e.to_string())
    }

    /// 生の受信状態。状態表示と診断用。
    pub fn state(&self) -> SbusState {
        self.rx.state()
    }
}

impl Pilot for SbusPilot {
    fn poll(&mut self, now: Time) -> Intent {
        let state = self.rx.state();
        let usable = state.is_usable(self.timeout);
        let mut intent = if !usable && self.allow_no_sbus {
            self.teleop.bench_stand()
        } else {
            self.teleop.update(&state, usable)
        };
        intent.time = now;
        intent
    }
}

/// 台本。シミュレータと CI が使う。
///
/// `sim` feature を外したビルドでは誰も構築しないので dead_code になる。
/// それは想定どおりで、他の警告が埋もれないように黙らせてある。
///
/// 実機の操縦を模すのではなく、**同じ意図を毎周期返すだけ**。速度を入れる
/// タイミングは呼び出し側が [`Self::set`] で決める（歩容へ入ってから、など）。
#[allow(dead_code)]
pub struct ScriptPilot {
    intent: Intent,
}

#[allow(dead_code)]
impl ScriptPilot {
    pub fn new(intent: Intent) -> Self {
        Self { intent }
    }

    pub fn set(&mut self, intent: Intent) {
        self.intent = intent;
    }

    pub fn intent_mut(&mut self) -> &mut Intent {
        &mut self.intent
    }
}

impl Pilot for ScriptPilot {
    fn poll(&mut self, now: Time) -> Intent {
        let mut intent = self.intent.clone();
        intent.time = now;
        intent
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use misa_core::{GaitSelect, ModeRequest, Velocity};

    #[test]
    fn a_script_pilot_stamps_the_time_it_was_polled() {
        let mut p = ScriptPilot::new(Intent {
            mode: ModeRequest::Walk,
            gait: GaitSelect::Trot,
            velocity: Velocity {
                vx_m_s: 0.2,
                ..Velocity::ZERO
            },
            ..Intent::default()
        });
        let i = p.poll(Time::from_secs_f64(1.5));
        assert_eq!(i.time, Time::from_secs_f64(1.5));
        assert_eq!(i.velocity.vx_m_s, 0.2);
        assert_eq!(i.mode, ModeRequest::Walk);
    }

    /// 台本を差し替えたら次の周期から効くこと。
    #[test]
    fn the_script_can_be_changed_between_ticks() {
        let mut p = ScriptPilot::new(Intent::default());
        assert_eq!(p.poll(Time::ZERO).mode, ModeRequest::Relax);
        p.intent_mut().mode = ModeRequest::Walk;
        assert_eq!(p.poll(Time::ZERO).mode, ModeRequest::Walk);
    }
}
