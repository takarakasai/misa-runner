//! 指令を書き換えてよい唯一の場所。
//!
//! Policy と Plant の間に 1 枚だけ挟む。それ自体は純関数で、状態は
//! 「直近に通した目標」だけを持つ。
//!
//! # なぜ 1 か所に集めるのか
//!
//! 制限そのものは各層に置いてあっても効く。効かないのは**説明**で、
//! スルーレート制限・可動域クランプ・受信断・異常ビットが 4 層に散っていると、
//! 「なぜこの指令になったか」を後から 1 か所で答えられない。ここを通した
//! 結果を [`SafetyVerdict`] として記録に残せば、ログだけで答えられる。
//!
//! # HAL の制限を置き換えるものではない
//!
//! 実機側（バススレッドのスルーレート制限と、ワイヤ直前の可動域クランプ）は
//! **最後の防波堤として残す**。あちらはバスごとの実測 dt で効いていて、
//! ここより下の層で最後に丸める。二重にかかるのは無駄ではなく、
//! この層を通らずに出た指令（校正コマンドなど）を拾うために要る。
//!
//! # 異常ビットでは何もしない
//!
//! 検出して [`SafetyVerdict::faulted`] に載せるだけで、指令には触らない。
//! **立っている四足を脱力させると崩れる**ので、止めるかどうかは operator が
//! 決める。自動で安全側に倒すことが安全でない場面がある。

use core::time::Duration;

use serde::{Deserialize, Serialize};

use crate::axis::AxisId;
use crate::command::{Command, ControlMode};
use crate::observation::Observation;

/// 1 軸に掛ける制限。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AxisLimits {
    /// モデル座標系での可動域 [rad]。
    pub min_rad: f64,
    pub max_rad: f64,
    /// **目標そのもの**が動いてよい速さ [rad/s]。`0` なら制限しない。
    ///
    /// モータ側の速度上限とは別物。あちらは「軸が何 rad/s で回るか」で、
    /// こちらは「目標が何 rad/s で動くか」。歩容の切り替えや IK のクランプで
    /// 目標が跳んだとき、あちらだけだと上限速度で追いに行ってしまう。
    pub max_target_rate_rad_s: f64,
    /// トルク指令の上限 [N·m]。`0` なら制限しない。
    pub max_torque_nm: f64,
}

impl AxisLimits {
    /// 何も制限しない軸。可動域は無限大。
    pub const UNLIMITED: AxisLimits = AxisLimits {
        min_rad: f64::NEG_INFINITY,
        max_rad: f64::INFINITY,
        max_target_rate_rad_s: 0.0,
        max_torque_nm: 0.0,
    };
}

/// ゲートの設定。並びは [`crate::axis::AxisTable`] に従う。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SafetyConfig {
    pub axes: Vec<AxisLimits>,
    /// 観測がこれより古ければ、目標を進めるのをやめて現状を保持する。
    pub max_observation_age: Duration,
}

/// この周期で何をしたか。**記録に残して後から説明するためのもの。**
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct SafetyVerdict {
    /// 可動域で丸めた軸。
    pub clamped: Vec<AxisId>,
    /// 目標の変化を鈍らせた軸。
    pub rate_limited: Vec<AxisId>,
    /// トルク指令を丸めた軸。
    pub torque_limited: Vec<AxisId>,
    /// 観測が古すぎて目標を進めなかったか。
    pub held_for_stale_observation: bool,
    /// 異常ビットが立っている軸。**指令には触っていない。**
    pub faulted: Vec<AxisId>,
}

impl SafetyVerdict {
    /// 何かに手を入れたか。ログを間引くのに使う。
    pub fn is_clean(&self) -> bool {
        self.clamped.is_empty()
            && self.rate_limited.is_empty()
            && self.torque_limited.is_empty()
            && !self.held_for_stale_observation
            && self.faulted.is_empty()
    }
}

/// 指令を安全側へ丸める。
pub struct SafetyGate {
    cfg: SafetyConfig,
    /// 直近に通した目標角。`None` は「まだ通していない」。
    ///
    /// 脱力に落ちたら `None` に戻す。前回の目標を覚えたままだと、
    /// **脱力中に手で動かされた分をいきなり戻しに行く**。
    issued: Vec<Option<f64>>,
}

impl SafetyGate {
    pub fn new(cfg: SafetyConfig) -> Self {
        let issued = vec![None; cfg.axes.len()];
        Self { cfg, issued }
    }

    pub fn limits(&self) -> &[AxisLimits] {
        &self.cfg.axes
    }

    /// 直近に通した目標を忘れる。次の位置指令は実測から出発する。
    pub fn forget(&mut self) {
        for s in &mut self.issued {
            *s = None;
        }
    }

    /// **指令を書き換えてよい唯一の入口。**
    ///
    /// `dt` は前回この関数を呼んでからの経過。呼び出し側が実測を渡すこと。
    /// 目標周期を渡すと、ループが遅れている間に目標だけ規定どおり進んで
    /// 制限の意味が無くなる。
    pub fn apply(&mut self, cmd: &mut Command, obs: &Observation, dt: Duration) -> SafetyVerdict {
        let mut v = SafetyVerdict::default();

        // 観測が古いときは目標を進めない。**モードは変えない。**
        // 見えていない相手に新しい目標を出すより、いまの姿勢で止まるほうが安全。
        // 脱力へ落とさないのは、荷重がかかった四足を脱力させると崩れるから。
        let stale = obs.worst_age() > self.cfg.max_observation_age;
        v.held_for_stale_observation = stale;

        for (id, state) in obs.faulted() {
            let _ = state;
            v.faulted.push(id);
        }

        let n = cmd.len().min(self.cfg.axes.len()).min(obs.len());
        for i in 0..n {
            let id = AxisId::new(i as u16);
            let lim = self.cfg.axes[i];
            let measured = obs.get(id).map(|s| s.position_rad).unwrap_or(0.0);
            let Some(a) = cmd.get_mut(id) else { continue };

            if a.mode == ControlMode::Idle {
                // 脱力中は何も丸めない。目標は残すが、通した記録は捨てる。
                self.issued[i] = None;
                continue;
            }

            if lim.max_torque_nm > 0.0 && a.torque_ff_nm.abs() > lim.max_torque_nm {
                a.torque_ff_nm = a.torque_ff_nm.clamp(-lim.max_torque_nm, lim.max_torque_nm);
                v.torque_limited.push(id);
            }

            // **可動域を先に、スルーレートを後に。**
            //
            // 逆順にすると、可動域の外を指した目標に向かって内部状態が
            // 制限レートで際限なく進み、目標が戻ってきたときに**外にいた
            // 時間ぶんだけ遅れて**追従する（積み上がり）。先に丸めておけば
            // 内部状態は常に可動域の内側に留まる。
            let want = a.position_rad.clamp(lim.min_rad, lim.max_rad);
            if want != a.position_rad {
                v.clamped.push(id);
            }

            // 位置制御に入った最初の 1 回は**実測位置から**出発する。
            let from = self.issued[i].unwrap_or(measured);
            let target = if stale {
                from
            } else if lim.max_target_rate_rad_s > 0.0 && !dt.is_zero() {
                let step = lim.max_target_rate_rad_s * dt.as_secs_f64();
                from + (want - from).clamp(-step, step)
            } else {
                want
            };
            if target != want {
                v.rate_limited.push(id);
            }
            self.issued[i] = Some(target);
            a.position_rad = target;
        }
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command::AxisCommand;
    use crate::observation::Observation;

    const DT: Duration = Duration::from_millis(10);

    fn gate(limits: AxisLimits, n: usize) -> SafetyGate {
        SafetyGate::new(SafetyConfig {
            axes: vec![limits; n],
            max_observation_age: Duration::from_millis(100),
        })
    }

    fn fresh_obs(n: usize, position_rad: f64) -> Observation {
        let mut o = Observation::empty(n, 1);
        for i in 0..n {
            let a = o.get_mut(AxisId::new(i as u16)).unwrap();
            a.position_rad = position_rad;
            a.health.valid = true;
        }
        o
    }

    fn position_cmd(n: usize, q: f64) -> Command {
        let mut c = Command::idle(n);
        for i in 0..n {
            *c.get_mut(AxisId::new(i as u16)).unwrap() = AxisCommand::position(q, 8.0);
        }
        c
    }

    #[test]
    fn a_target_outside_the_range_is_clamped() {
        let mut g = gate(
            AxisLimits {
                min_rad: -0.5,
                max_rad: 0.5,
                ..AxisLimits::UNLIMITED
            },
            1,
        );
        let mut cmd = position_cmd(1, 2.0);
        let v = g.apply(&mut cmd, &fresh_obs(1, 0.0), DT);
        assert_eq!(cmd.get(AxisId::new(0)).unwrap().position_rad, 0.5);
        assert_eq!(v.clamped, vec![AxisId::new(0)]);
    }

    /// 3 rad/s · 10 ms = 0.03 rad しか進まない。
    #[test]
    fn the_target_moves_no_faster_than_the_rate_limit() {
        let mut g = gate(
            AxisLimits {
                max_target_rate_rad_s: 3.0,
                ..AxisLimits::UNLIMITED
            },
            1,
        );
        let mut cmd = position_cmd(1, 1.5);
        let v = g.apply(&mut cmd, &fresh_obs(1, 0.0), DT);
        assert!((cmd.get(AxisId::new(0)).unwrap().position_rad - 0.03).abs() < 1e-12);
        assert_eq!(v.rate_limited, vec![AxisId::new(0)]);
    }

    /// **位置制御に入る最初の 1 回は実測から出発する。**
    ///
    /// 前回の目標を覚えたままだと、脱力中に手で動かされた分をいきなり
    /// 戻しに行く。
    #[test]
    fn re_entering_position_control_starts_from_the_measured_angle() {
        let mut g = gate(
            AxisLimits {
                max_target_rate_rad_s: 3.0,
                ..AxisLimits::UNLIMITED
            },
            1,
        );
        // 一度通してから脱力へ落とす。
        let mut cmd = position_cmd(1, 1.0);
        g.apply(&mut cmd, &fresh_obs(1, 0.0), DT);
        let mut relaxed = Command::idle(1);
        g.apply(&mut relaxed, &fresh_obs(1, 0.0), DT);

        // その間に手で 0.8 rad へ動かされた。
        let mut cmd = position_cmd(1, 1.0);
        g.apply(&mut cmd, &fresh_obs(1, 0.8), DT);
        // 0.03 ではなく 0.83 から始まる（0.8 + 0.03）。
        assert!((cmd.get(AxisId::new(0)).unwrap().position_rad - 0.83).abs() < 1e-12);
    }

    /// **可動域の外を指し続けても内部状態は積み上がらない。**
    ///
    /// 先に丸めずスルーレートを掛けると、内部状態が外へ際限なく進み、
    /// 目標が戻ったときに外にいた時間ぶん遅れて追従する。
    #[test]
    fn holding_a_target_outside_the_range_does_not_wind_up() {
        let mut g = gate(
            AxisLimits {
                min_rad: -0.5,
                max_rad: 0.5,
                max_target_rate_rad_s: 100.0,
                ..AxisLimits::UNLIMITED
            },
            1,
        );
        let obs = fresh_obs(1, 0.0);
        for _ in 0..100 {
            let mut cmd = position_cmd(1, 50.0);
            g.apply(&mut cmd, &obs, DT);
        }
        // 目標を可動域の内側へ戻したら、1 周期で追いつけること。
        let mut cmd = position_cmd(1, 0.4);
        g.apply(&mut cmd, &obs, DT);
        assert!((cmd.get(AxisId::new(0)).unwrap().position_rad - 0.4).abs() < 1e-12);
    }

    /// **観測が古いときは目標を進めず、その場で保持する。**
    ///
    /// モードは変えない。脱力へ落とすと荷重のかかった四足が崩れる。
    #[test]
    fn a_stale_observation_freezes_the_target_without_relaxing() {
        let mut g = gate(
            AxisLimits {
                max_target_rate_rad_s: 3.0,
                ..AxisLimits::UNLIMITED
            },
            1,
        );
        let mut cmd = position_cmd(1, 1.0);
        g.apply(&mut cmd, &fresh_obs(1, 0.0), DT);
        let held = cmd.get(AxisId::new(0)).unwrap().position_rad;

        let mut stale = fresh_obs(1, 0.0);
        stale.get_mut(AxisId::new(0)).unwrap().health.age = Duration::from_millis(500);
        let mut cmd = position_cmd(1, 1.0);
        let v = g.apply(&mut cmd, &stale, DT);

        assert!(v.held_for_stale_observation);
        assert_eq!(cmd.get(AxisId::new(0)).unwrap().position_rad, held);
        assert_eq!(cmd.get(AxisId::new(0)).unwrap().mode, ControlMode::Position);
    }

    /// **異常ビットは報告するだけ。指令には触らない。**
    #[test]
    fn a_fault_is_reported_but_never_relaxes_the_robot() {
        let mut g = gate(AxisLimits::UNLIMITED, 1);
        let mut obs = fresh_obs(1, 0.0);
        obs.get_mut(AxisId::new(0)).unwrap().health.fault_raw = 0x04;

        let mut cmd = position_cmd(1, 0.7);
        let v = g.apply(&mut cmd, &obs, DT);

        assert_eq!(v.faulted, vec![AxisId::new(0)]);
        assert_eq!(cmd.get(AxisId::new(0)).unwrap().mode, ControlMode::Position);
        assert_eq!(cmd.get(AxisId::new(0)).unwrap().position_rad, 0.7);
    }

    #[test]
    fn nothing_to_report_when_the_command_is_already_inside_every_limit() {
        let mut g = gate(
            AxisLimits {
                min_rad: -1.0,
                max_rad: 1.0,
                max_target_rate_rad_s: 100.0,
                max_torque_nm: 10.0,
            },
            2,
        );
        let mut cmd = position_cmd(2, 0.1);
        let v = g.apply(&mut cmd, &fresh_obs(2, 0.1), DT);
        assert!(v.is_clean(), "{v:?}");
    }
}
