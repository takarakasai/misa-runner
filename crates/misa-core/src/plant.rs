//! 動くもの。実機・シミュレータ・ログ再生を同じ穴に通す。
//!
//! 粒度は**全軸・1 tick**。書き込みと読み出しを別々の呼び出しにせず
//! [`Plant::exchange`] 1 本にしてあるのは、3 つの配備先が揃ってこの形を
//! しているため。
//!
//! | 配備先 | 実体 |
//! |---|---|
//! | 中間層 MCU（namiashi2） | 周期 Down/Up が `seq` で 1:1 |
//! | MuJoCo | `ctrl` を書いて `step` して状態を読む |
//! | Unitree Go2 | `rt/lowcmd` を送って `rt/lowstate` を受ける |
//! | ログ再生 | 記録した観測を返すだけ |
//!
//! # 1 周期ぶんの観測の古さが構造に入る
//!
//! **これは実装の都合ではなく、要求応答の性質そのもの。** 指令を送って
//! 状態が返るまでが 1 往復なので、`tick` が見る観測は「1 つ前の
//! `exchange` が持ち帰ったもの」になる。ループはこう回る:
//!
//! ```text
//! loop {
//!     let cmd = policy.tick(&obs);      // obs は前周期に持ち帰ったもの
//!     plant.exchange(&cmd, &mut obs);   // cmd を出し、新しい obs を受け取る
//!     sleep(period);
//! }
//! ```
//!
//! 指令の遅れは無い（`tick` の直後に出る）。観測が 1 周期ぶん古くなる。
//! 200 Hz なら 5 ms で、[`crate::observation::AxisHealth::age`] に載るので
//! 上位からも見える。
//!
//! namiashi の実機のように**読み出しが只**（バススレッドが共有スロットへ
//! 置き続ける）構成では、読みを `tick` の直前に置けばこの 1 周期は消せる。
//! それでも `exchange` に揃えてあるのは、消せない配備先が混ざる以上、
//! **一番厳しい側に合わせておかないと移したときに壊れる**ため。

use crate::axis::{AxisId, AxisTable};
use crate::command::{Command, ControlMode};
use crate::observation::Observation;

/// Plant が何をできるか。**起動時に照会して、足りなければその場で失敗する。**
///
/// namiashi の腕サーボで「繋がっている」と「こちらの指令で動く」を別の
/// 述語に分けたのは正しい判断だった（受信機直結の腕は動いてはいるが、
/// アプリの指令では動かない）。それを全能力へ広げたもの。
/// **黙って劣化させないこと。**
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct PlantCaps {
    /// 受け付ける制御モード。ここに無いモードの指令は拒否される。
    pub modes: Vec<ControlMode>,
    /// 胴体の IMU を観測できるか。
    pub has_imu: bool,
    /// 足の接地を観測できるか。実機は足裏センサが要る。
    pub has_contacts: bool,
    /// 軸ごとに、**こちらの指令で動くか**。並びは [`AxisTable`] に従う。
    /// false の軸は観測だけができる。
    pub driven: Vec<bool>,
}

impl PlantCaps {
    pub fn accepts(&self, mode: ControlMode) -> bool {
        mode == ControlMode::Idle || self.modes.contains(&mode)
    }

    /// 指令が能力の範囲に収まっているか。**起動時に 1 回確かめる用。**
    ///
    /// 毎周期呼ぶものではない。使えないモードを黙って捨てると、
    /// 「指令しているのに動かない」が原因不明のまま残る。
    pub fn check(&self, cmd: &Command) -> Result<(), String> {
        for (i, a) in cmd.axes().iter().enumerate() {
            if !self.accepts(a.mode) {
                return Err(format!("軸 {i} の {:?} をこの Plant は受け付けません", a.mode));
            }
            if a.mode != ControlMode::Idle && !self.driven.get(i).copied().unwrap_or(false) {
                return Err(format!("軸 {i} はこちらの指令では動きません"));
            }
        }
        Ok(())
    }
}

/// 実機・シミュレータ・再生を同じ形にしたもの。
pub trait Plant {
    /// 軸の並び。[`Command`] と [`Observation`] はこれに従う。
    fn axes(&self) -> &AxisTable;

    fn capabilities(&self) -> &PlantCaps;

    /// 閉ループを入れる。
    fn arm(&mut self) -> Result<(), String>;

    /// 閉ループを切る。軸は脱力する。
    fn disarm(&mut self) -> Result<(), String>;

    /// **こちらの指令で動かない軸の観測値**を外から教える。
    ///
    /// 受信機直結の腕がこれ。プロポのチャンネルから割り出した角度を
    /// 入れると、ログ・可視化・モデル状態に実際の角度が載る。駆動する軸
    /// しか無い Plant では何もしなくてよい。
    fn observe_aux(&mut self, _axis: AxisId, _value_rad: f64) {}

    /// 状態表示に添える 1 行。バスの実効周期など、**この Plant にしか
    /// 分からないこと**を書く。既定は空。
    fn status_line(&self) -> String {
        String::new()
    }

    /// 異常ビットの生値を人が読める形にする。
    ///
    /// **生値の `0x01` だけ出しても現場では何も分からない。** ビットごとに
    /// 意味も対処も違う（低電圧は電源、過熱は冷却待ち）ので、ベンダを知って
    /// いる実装が名前を付ける。
    fn describe_fault(&self, raw: u32) -> String {
        format!("{raw:#010x}")
    }

    /// **指令を渡し、観測を受け取る。これが 1 tick の全部。**
    ///
    /// `obs` は使い回す。毎周期作り直さないのは、200 Hz で確保を繰り返すと
    /// ジッタの原因になるため。長さは [`Self::axes`] と揃っていること。
    fn exchange(&mut self, cmd: &Command, obs: &mut Observation) -> Result<(), String>;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::axis::{Axis, AxisId, AxisRole};
    use crate::command::AxisCommand;

    fn caps(driven: Vec<bool>) -> PlantCaps {
        PlantCaps {
            modes: vec![ControlMode::Position],
            has_imu: true,
            has_contacts: false,
            driven,
        }
    }

    #[test]
    fn idle_is_always_accepted() {
        let c = caps(vec![false]);
        assert!(c.accepts(ControlMode::Idle));
        assert!(!c.accepts(ControlMode::Torque));
    }

    /// **使えないモードは起動時に弾く。** 黙って捨てると
    /// 「指令しているのに動かない」が原因不明のまま残る。
    #[test]
    fn a_mode_the_plant_cannot_do_is_refused_up_front() {
        let mut cmd = Command::idle(1);
        cmd.get_mut(AxisId::new(0)).unwrap().mode = ControlMode::Torque;
        let e = caps(vec![true]).check(&cmd).unwrap_err();
        assert!(e.contains("Torque"), "{e}");
    }

    /// **駆動していない軸へ指令を出そうとしたら弾く。**
    ///
    /// 受信機直結の腕がこれ。動いてはいるがこちらの指令では動かないので、
    /// 指令を出す構成にした時点で気づけないと、演出が黙って無効になる。
    #[test]
    fn commanding_an_axis_the_plant_does_not_drive_is_refused() {
        let mut cmd = Command::idle(2);
        *cmd.get_mut(AxisId::new(1)).unwrap() = AxisCommand::position(0.0, 1.0);
        let e = caps(vec![true, false]).check(&cmd).unwrap_err();
        assert!(e.contains("軸 1"), "{e}");
    }

    /// 観測だけの軸でも、脱力のままなら通る。
    #[test]
    fn leaving_an_undriven_axis_relaxed_is_fine() {
        let cmd = Command::idle(2);
        assert!(caps(vec![true, false]).check(&cmd).is_ok());
    }

    #[test]
    fn the_axis_table_and_the_caps_line_up() {
        let t = AxisTable::new(vec![Axis {
            name: "FL_hip_joint".into(),
            role: AxisRole::Leg { leg: 0, joint: 0 },
        }])
        .unwrap();
        assert_eq!(t.len(), caps(vec![true]).driven.len());
    }
}
