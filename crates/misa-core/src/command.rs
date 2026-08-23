//! 全軸ぶんの指令。
//!
//! # なぜ位置しか使わないのにトルクの口を持つのか
//!
//! いま実機に流しているのは位置指令だけで、[`AxisCommand`] の
//! `torque_ff_nm` や `kp` は誰も見ていない。それでも最初から持たせるのは、
//! **後から足すと全レイヤに波及する**ため。Robstride は MIT
//! （`tau = kp·(q* − q) + kd·(dq* − dq) + tau_ff`）をネイティブに持ち、
//! WBC はトルクで出す。そのとき `Command` の形が変わると、Policy も
//! SafetyGate も Plant も記録の形式も一斉に直すことになる。
//!
//! 使わないフィールドが 0 で埋まっているコストは、その付け替えより安い。

use serde::{Deserialize, Serialize};

use crate::axis::AxisId;

/// 1 軸をどう動かすか。
///
/// **どのモードを使えるかは Plant の能力次第。** ベンダによってモード切替の
/// 作法が違う（コマンドごとに指定できるものと、レジスタを書き換えて
/// Disable/Enable を挟むものがある）ので、能力の照会と検査は Plant 側の仕事。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlMode {
    /// 脱力。**指令値は無視される。**
    #[default]
    Idle,
    /// 位置制御。`position_rad` と `velocity_rad_s`（速度上限）を見る。
    Position,
    /// 速度制御。`velocity_rad_s` を見る。
    Velocity,
    /// トルク制御。`torque_ff_nm` を見る。
    Torque,
    /// インピーダンス（MIT）。全フィールドを見る。
    Impedance,
}

/// 1 軸への指令。単位は出力軸の SI（rad, rad/s, N·m）。
///
/// 位置は**モデル座標系**。実機の符号とゼロ点の差は Plant が吸収する
/// （上位はモデルの関節角しか扱わない、という約束）。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AxisCommand {
    pub mode: ControlMode,
    pub position_rad: f64,
    /// [`ControlMode::Position`] では速度**上限**、
    /// [`ControlMode::Velocity`] と [`ControlMode::Impedance`] では目標速度。
    pub velocity_rad_s: f64,
    pub torque_ff_nm: f64,
    pub kp_nm_per_rad: f64,
    pub kd_nm_s_per_rad: f64,
}

impl Default for AxisCommand {
    fn default() -> Self {
        Self::idle()
    }
}

impl AxisCommand {
    pub const fn idle() -> Self {
        Self {
            mode: ControlMode::Idle,
            position_rad: 0.0,
            velocity_rad_s: 0.0,
            torque_ff_nm: 0.0,
            kp_nm_per_rad: 0.0,
            kd_nm_s_per_rad: 0.0,
        }
    }

    pub const fn position(position_rad: f64, max_speed_rad_s: f64) -> Self {
        Self {
            mode: ControlMode::Position,
            position_rad,
            velocity_rad_s: max_speed_rad_s,
            ..Self::idle()
        }
    }
}

/// 1 周期ぶんの、全軸への指令。並びは [`crate::axis::AxisTable`] に従う。
///
/// **毎周期作り直さずに使い回す。** `Plant::exchange` が `&Command` を取る
/// のはそのためで、確保は立ち上げのとき 1 回で済む。
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Command {
    axes: Vec<AxisCommand>,
}

impl Command {
    /// 全軸を脱力にした長さ `n` の指令。
    pub fn idle(n: usize) -> Self {
        Self {
            axes: vec![AxisCommand::idle(); n],
        }
    }

    pub fn len(&self) -> usize {
        self.axes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.axes.is_empty()
    }

    pub fn axes(&self) -> &[AxisCommand] {
        &self.axes
    }

    pub fn get(&self, id: AxisId) -> Option<&AxisCommand> {
        self.axes.get(id.index())
    }

    pub fn get_mut(&mut self, id: AxisId) -> Option<&mut AxisCommand> {
        self.axes.get_mut(id.index())
    }

    /// 全軸を脱力に戻す。長さは変えない。
    pub fn relax_all(&mut self) {
        for a in &mut self.axes {
            a.mode = ControlMode::Idle;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fresh_command_relaxes_every_axis() {
        let c = Command::idle(13);
        assert_eq!(c.len(), 13);
        assert!(c.axes().iter().all(|a| a.mode == ControlMode::Idle));
    }

    /// **脱力は「モードを落とす」であって「目標を 0 にする」ではない。**
    ///
    /// 目標角を 0 に潰すと、脱力から復帰した瞬間に全軸が 0 rad へ飛ぶ。
    /// namiashi が「脱力中の目標角は実測角」にしているのと同じ理由で、
    /// 直前の目標はそのまま残す。
    #[test]
    fn relaxing_keeps_the_last_targets() {
        let mut c = Command::idle(2);
        *c.get_mut(AxisId::new(0)).unwrap() = AxisCommand::position(1.25, 8.0);
        c.relax_all();
        let a = c.get(AxisId::new(0)).unwrap();
        assert_eq!(a.mode, ControlMode::Idle);
        assert_eq!(a.position_rad, 1.25);
    }
}
