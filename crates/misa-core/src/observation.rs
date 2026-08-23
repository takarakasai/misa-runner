//! 1 周期ぶんの観測。
//!
//! # 古さを構造として持つ
//!
//! 実機の値は「いま読んだ値」ではなく「最後に読めた値」で、バスが詰まれば
//! 古いまま残る。古さを型に出しておかないと、上位は**止まった値を現在値だと
//! 信じて**制御を続ける。namiashi では無応答のモータが 1 台いるだけでバスが
//! 16 Hz まで落ちたことがあり、そこで古い角度を新しいと読むと目標が跳ねる。
//!
//! だから [`AxisState`] と [`Imu`] は `age` を持ち、埋めるのは Plant の
//! 責務にしてある。フェイルセーフの判断は SafetyGate がこれを見て決める。

use core::time::Duration;

use serde::{Deserialize, Serialize};

use crate::axis::AxisId;
use crate::time::Time;

/// 軸の健全性。値そのものではなく、値をどこまで信じてよいか。
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct AxisHealth {
    /// 一度でも読めたか。**false の値は意味を持たない。**
    ///
    /// 起動直後の 0 rad と、実際に 0 rad にいる軸を区別するために要る。
    /// ここを見ずに使うと、読み戻しが済む前の姿勢で可視化とログが埋まる。
    pub valid: bool,
    /// この値が最後に更新されてからの経過。
    pub age: Duration,
    /// ベンダ固有の異常ビットの生値。0 なら異常なし。意味はドライバが知る。
    pub fault_raw: u32,
    pub temperature_c: Option<f64>,
    pub voltage_v: Option<f64>,
}

impl AxisHealth {
    pub fn faulted(&self) -> bool {
        self.valid && self.fault_raw != 0
    }
}

/// 1 軸の観測。位置は**モデル座標系** [rad]。
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct AxisState {
    pub position_rad: f64,
    pub velocity_rad_s: f64,
    /// 取れる機体でだけ入る。位置制御だけの構成では `None`。
    pub torque_nm: Option<f64>,
    pub health: AxisHealth,
}

/// 胴体の慣性計測。
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Imu {
    /// 姿勢角 [rad]。`[roll, pitch, yaw]`、取付補正済み。
    pub rpy_rad: [f64; 3],
    /// 角速度 [rad/s]、胴体座標系。
    pub gyro_rad_s: [f64; 3],
    /// 加速度 [m/s²]、重力込み、胴体座標系。
    pub accel_m_s2: [f64; 3],
    pub age: Duration,
}

/// 足の接地。
///
/// `Option` にしているのは、**「接地していない」と「分からない」は別物**
/// だから。シミュレータは接触を知っているが、実機は足裏センサが無ければ
/// 推定するしかない。潰すと、推定を持たない実機で「全脚が浮いている」と
/// 読まれる。
pub type Contact = Option<bool>;

/// 1 周期ぶんの観測。並びは [`crate::axis::AxisTable`] に従う。
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Observation {
    /// この観測の時刻。**Plant が入れる。**
    pub time: Time,
    axes: Vec<AxisState>,
    /// 積んでいない構成では `None`。
    pub imu: Option<Imu>,
    /// 脚ごと。並びは [`crate::axis::AxisTable::legs`] と同じ。
    pub contacts: Vec<Contact>,
}

impl Observation {
    /// 軸 `n` 本・脚 `legs` 本ぶんの入れ物。値はすべて未取得（`valid = false`）。
    pub fn empty(n: usize, legs: usize) -> Self {
        Self {
            time: Time::ZERO,
            axes: vec![AxisState::default(); n],
            imu: None,
            contacts: vec![None; legs],
        }
    }

    pub fn len(&self) -> usize {
        self.axes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.axes.is_empty()
    }

    pub fn axes(&self) -> &[AxisState] {
        &self.axes
    }

    pub fn get(&self, id: AxisId) -> Option<&AxisState> {
        self.axes.get(id.index())
    }

    pub fn get_mut(&mut self, id: AxisId) -> Option<&mut AxisState> {
        self.axes.get_mut(id.index())
    }

    /// まだ一度も読めていない軸があるか。
    ///
    /// 起立へ移る前にこれが false になっているのを確かめる、という使い方を
    /// 想定している。1 軸でも未取得のまま目標を組むと、その軸だけ 0 rad を
    /// 基準にした値が混じる。
    pub fn any_unread(&self) -> bool {
        self.axes.iter().any(|a| !a.health.valid)
    }

    /// 異常ビットが立っている軸。
    ///
    /// **自動で脱力はしない。** 立っている四足を脱力させると崩れるので、
    /// 止めるかどうかは operator が決める。ここは検出だけを担う。
    pub fn faulted(&self) -> impl Iterator<Item = (AxisId, &AxisState)> {
        self.axes
            .iter()
            .enumerate()
            .filter(|(_, a)| a.health.faulted())
            .map(|(i, a)| (AxisId::new(i as u16), a))
    }

    /// もっとも古い軸の古さ。1 軸も無ければ 0。
    pub fn worst_age(&self) -> Duration {
        self.axes
            .iter()
            .map(|a| a.health.age)
            .max()
            .unwrap_or(Duration::ZERO)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fresh_observation_has_nothing_read_yet() {
        let o = Observation::empty(13, 4);
        assert_eq!(o.len(), 13);
        assert_eq!(o.contacts.len(), 4);
        assert!(o.any_unread());
        // **接地は「不明」で始まる。**「浮いている」ではない。
        assert!(o.contacts.iter().all(|c| c.is_none()));
    }

    /// **古さは一番悪い軸で代表する。** 平均や先頭ではない。
    ///
    /// 12 軸のうち 1 軸だけ 200 ms 遅れている状態は、平均を取ると 17 ms に
    /// 見えて健全に読めてしまう。フェイルセーフに使う値は最悪値であること。
    #[test]
    fn staleness_is_reported_by_the_worst_axis() {
        let mut o = Observation::empty(3, 1);
        o.get_mut(AxisId::new(0)).unwrap().health.age = Duration::from_millis(5);
        o.get_mut(AxisId::new(1)).unwrap().health.age = Duration::from_millis(200);
        o.get_mut(AxisId::new(2)).unwrap().health.age = Duration::from_millis(5);
        assert_eq!(o.worst_age(), Duration::from_millis(200));
    }

    #[test]
    fn faults_are_listed_but_only_for_axes_that_were_read() {
        let mut o = Observation::empty(2, 1);
        // 未取得の軸のビットは意味を持たないので、拾ってはいけない。
        o.get_mut(AxisId::new(0)).unwrap().health.fault_raw = 0x04;
        let a1 = o.get_mut(AxisId::new(1)).unwrap();
        a1.health.valid = true;
        a1.health.fault_raw = 0x08;

        let ids: Vec<AxisId> = o.faulted().map(|(id, _)| id).collect();
        assert_eq!(ids, vec![AxisId::new(1)]);
    }
}
