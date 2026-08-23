//! 記録の 1 行。
//!
//! 毎周期 [`Frame`] を 1 つ追記する。入っているのは**その周期の全部**、
//! つまり意図・観測・指令・ゲートの判定。これだけあれば周期を再構成できる。
//!
//! # なぜこれを先に入れるのか
//!
//! これから触るのは制御ループそのもので、いちばん壊したくない場所。
//! 記録があると、**改修の前後で同じ入力に対する指令を差分**できる。
//! 1 bit も変わらなければ挙動を変えていないと言い切れるし、変わったなら
//! どの軸のどのフィールドがいつ変わったかまで出る。
//!
//! 逆に言うと、**記録は危ない改修より先に採っておかないと意味がない**。
//! 比較対象が無くなる。
//!
//! # ほかに効く場面
//!
//! - **現場の再現** — 倒れた瞬間のログを手元で同じ Policy に食わせる
//! - **シムと実機の比較** — 同じ形式なので直接差分できる
//! - **可視化** — いまの `--viz` は、この記録の出力先の 1 つに降格する

use serde::{Deserialize, Serialize};

use crate::axis::AxisId;
use crate::command::{AxisCommand, Command};
use crate::intent::Intent;
use crate::observation::Observation;
use crate::safety::SafetyVerdict;
use crate::time::Time;

/// 記録の書式。読み込み側が食い違いに気づけるように先頭に入れる。
pub const FORMAT_VERSION: u32 = 1;

/// 記録の先頭に 1 度だけ置く見出し。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Header {
    pub format_version: u32,
    /// ロボット名（プロファイルの `name`）。
    pub robot: String,
    /// 軸の並び。**これが無いと後から軸番号を関節名に戻せない。**
    pub axes: Vec<String>,
    /// 制御周期 [Hz]。
    pub rate_hz: f64,
}

/// 1 周期ぶんの記録。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Frame {
    pub seq: u64,
    pub time: Time,
    pub intent: Intent,
    pub observation: Observation,
    pub command: Command,
    pub verdict: SafetyVerdict,
}

/// 2 つの記録で指令が食い違った 1 点。
#[derive(Debug, Clone, PartialEq)]
pub struct Divergence {
    pub seq: u64,
    pub axis: AxisId,
    pub field: &'static str,
    pub left: f64,
    pub right: f64,
}

/// 1 軸ぶんの指令を比べる。**厳密な等値。**
///
/// 許容差を入れないのは、ここで見たいのが「数値が近いか」ではなく
/// 「同じ計算をしたか」だから。閾値を持たせると、少しずつずれていく
/// 改修が全部素通りする。
fn diff_axis(seq: u64, axis: AxisId, a: &AxisCommand, b: &AxisCommand, out: &mut Vec<Divergence>) {
    if a.mode != b.mode {
        out.push(Divergence {
            seq,
            axis,
            field: "mode",
            left: a.mode as u8 as f64,
            right: b.mode as u8 as f64,
        });
    }
    let mut push = |field, left: f64, right: f64| {
        if left != right {
            out.push(Divergence {
                seq,
                axis,
                field,
                left,
                right,
            });
        }
    };
    push("position_rad", a.position_rad, b.position_rad);
    push("velocity_rad_s", a.velocity_rad_s, b.velocity_rad_s);
    push("torque_ff_nm", a.torque_ff_nm, b.torque_ff_nm);
    push("kp_nm_per_rad", a.kp_nm_per_rad, b.kp_nm_per_rad);
    push("kd_nm_s_per_rad", a.kd_nm_s_per_rad, b.kd_nm_s_per_rad);
}

/// 2 つの指令列を比べ、食い違った点を古い順に返す。
///
/// `limit` は返す件数の上限。**1 周期ずれると以後の全周期が食い違う**ので、
/// 上限を置かないと出力が記録の長さぶん膨らむ。最初の数件が読めれば
/// たいてい原因は分かる。
pub fn diff_commands<'a>(
    left: impl IntoIterator<Item = &'a Frame>,
    right: impl IntoIterator<Item = &'a Frame>,
    limit: usize,
) -> Vec<Divergence> {
    let mut out = Vec::new();
    for (a, b) in left.into_iter().zip(right) {
        if out.len() >= limit {
            break;
        }
        let n = a.command.len().min(b.command.len());
        for i in 0..n {
            let id = AxisId::new(i as u16);
            let (Some(ca), Some(cb)) = (a.command.get(id), b.command.get(id)) else {
                continue;
            };
            diff_axis(a.seq, id, ca, cb, &mut out);
        }
    }
    out.truncate(limit);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command::ControlMode;

    fn frame(seq: u64, q: f64) -> Frame {
        let mut command = Command::idle(2);
        for i in 0..2 {
            *command.get_mut(AxisId::new(i)).unwrap() = AxisCommand::position(q, 8.0);
        }
        Frame {
            seq,
            time: Time::from_secs_f64(seq as f64 * 0.005),
            intent: Intent::default(),
            observation: Observation::empty(2, 1),
            command,
            verdict: SafetyVerdict::default(),
        }
    }

    #[test]
    fn identical_recordings_do_not_diverge() {
        let a: Vec<Frame> = (0..10).map(|i| frame(i, 0.5)).collect();
        let b = a.clone();
        assert!(diff_commands(&a, &b, 100).is_empty());
    }

    #[test]
    fn a_changed_target_is_reported_with_its_axis_and_field() {
        let a: Vec<Frame> = (0..3).map(|i| frame(i, 0.5)).collect();
        let mut b = a.clone();
        b[1].command.get_mut(AxisId::new(1)).unwrap().position_rad = 0.6;

        let d = diff_commands(&a, &b, 100);
        assert_eq!(d.len(), 1);
        assert_eq!(d[0].seq, 1);
        assert_eq!(d[0].axis, AxisId::new(1));
        assert_eq!(d[0].field, "position_rad");
        assert_eq!((d[0].left, d[0].right), (0.5, 0.6));
    }

    /// **許容差を持たない。** ここで見たいのは値が近いかではなく、
    /// 同じ計算をしたか。閾値を入れると少しずつずれる改修が素通りする。
    #[test]
    fn even_the_last_bit_counts_as_a_divergence() {
        let a: Vec<Frame> = (0..2).map(|i| frame(i, 0.5)).collect();
        let mut b = a.clone();
        b[0].command.get_mut(AxisId::new(0)).unwrap().position_rad = 0.5 + f64::EPSILON;
        assert_eq!(diff_commands(&a, &b, 100).len(), 1);
    }

    #[test]
    fn a_mode_change_is_a_divergence_too() {
        let a: Vec<Frame> = (0..2).map(|i| frame(i, 0.5)).collect();
        let mut b = a.clone();
        b[0].command.get_mut(AxisId::new(0)).unwrap().mode = ControlMode::Idle;
        let d = diff_commands(&a, &b, 100);
        assert_eq!(d[0].field, "mode");
    }

    /// **1 周期ずれると以後の全周期が食い違う。** 上限を置かないと出力が
    /// 記録の長さぶん膨らんで読めなくなる。
    #[test]
    fn the_number_of_reported_divergences_is_capped() {
        let a: Vec<Frame> = (0..1000).map(|i| frame(i, 0.5)).collect();
        let b: Vec<Frame> = (0..1000).map(|i| frame(i, 0.6)).collect();
        assert_eq!(diff_commands(&a, &b, 5).len(), 5);
    }

    /// 長さが違ってもパニックせず、短いほうまでで比べる。
    #[test]
    fn recordings_of_different_lengths_compare_up_to_the_shorter_one() {
        let a: Vec<Frame> = (0..10).map(|i| frame(i, 0.5)).collect();
        let b: Vec<Frame> = (0..3).map(|i| frame(i, 0.5)).collect();
        assert!(diff_commands(&a, &b, 100).is_empty());
    }
}
