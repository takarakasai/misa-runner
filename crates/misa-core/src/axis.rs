//! 軸の並び。
//!
//! **軸数を型に焼き込まない。** namiashi は脚 12 + 腕 1 の 13 軸、keel は
//! 脚 12 + 車輪 4 の 16 軸、go2 は 12 軸。`[[f64; 3]; 4]` のような形は
//! 補助軸を持つ機体を弾くし、`Legs<4, 3>` のような型パラメータは 2 脚を弾く。
//! ここでは可変長の索引表にして、並びは実行時のデータとして持つ。
//!
//! 並び自体はモデル（`.misa`）とロボットのプロファイルが決める。この crate は
//! 「順序つきの名前の列」以上のことを知らない。

use serde::{Deserialize, Serialize};

/// [`AxisTable`] への添字。
///
/// 生の `usize` にしないのは、脚番号・関節番号・チャンネル番号と取り違えても
/// 型が黙って通してしまうため。namiashi では「片脚だけ挙動がおかしい」という
/// 形で出る類の事故を、ここで 1 段止める。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct AxisId(u16);

impl AxisId {
    pub const fn new(index: u16) -> Self {
        Self(index)
    }

    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// 軸の役割。歩容が触ってよい軸と、そうでない軸を分けるためのもの。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AxisRole {
    /// 脚の関節。`leg` は脚番号、`joint` は脚内の並び（根元から先端へ）。
    Leg { leg: u8, joint: u8 },
    /// 脚以外の軸（腕、車輪、ヘッドなど）。歩容は触らない。
    Aux,
}

/// 1 軸の素性。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Axis {
    /// モデル（`.misa` / URDF）の関節名。ここが唯一の照合キー。
    pub name: String,
    pub role: AxisRole,
}

/// 軸の順序つき表。`Observation` と `Command` の並びはこれに従う。
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct AxisTable {
    axes: Vec<Axis>,
}

impl AxisTable {
    /// 重複する関節名があれば拒否する。
    ///
    /// **同じ名前が 2 度出ると、名前引きが片方だけを指す。** 一方への書き込みが
    /// 黙って捨てられ、症状は「その軸だけ動かない」になる。組み立て時に弾く。
    pub fn new(axes: Vec<Axis>) -> Result<Self, String> {
        for (i, a) in axes.iter().enumerate() {
            if let Some(j) = axes[..i].iter().position(|b| b.name == a.name) {
                return Err(format!(
                    "関節名 {:?} が {j} 番と {i} 番で重複しています",
                    a.name
                ));
            }
        }
        Ok(Self { axes })
    }

    pub fn len(&self) -> usize {
        self.axes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.axes.is_empty()
    }

    pub fn axes(&self) -> &[Axis] {
        &self.axes
    }

    pub fn get(&self, id: AxisId) -> Option<&Axis> {
        self.axes.get(id.index())
    }

    pub fn name(&self, id: AxisId) -> Option<&str> {
        self.get(id).map(|a| a.name.as_str())
    }

    /// 関節名から添字を引く。線形探索。
    ///
    /// **制御ループでは使わない。** 呼ぶのは組み立てのとき（モデルの姿勢を
    /// 軸ベクトルへ写すなど）だけで、そこは毎周期ではない。索引を先に
    /// 解決しておき、ループでは [`AxisId`] を持ち回ること。
    pub fn id_of(&self, name: &str) -> Option<AxisId> {
        self.axes
            .iter()
            .position(|a| a.name == name)
            .map(|i| AxisId::new(i as u16))
    }

    /// 脚の軸だけを、脚番号ごとにまとめて返す。
    pub fn legs(&self) -> Vec<Vec<AxisId>> {
        let mut out: Vec<Vec<AxisId>> = Vec::new();
        for (i, a) in self.axes.iter().enumerate() {
            if let AxisRole::Leg { leg, .. } = a.role {
                let leg = leg as usize;
                if out.len() <= leg {
                    out.resize(leg + 1, Vec::new());
                }
                out[leg].push(AxisId::new(i as u16));
            }
        }
        out
    }

    /// 脚以外の軸。
    pub fn aux(&self) -> Vec<AxisId> {
        self.axes
            .iter()
            .enumerate()
            .filter(|(_, a)| a.role == AxisRole::Aux)
            .map(|(i, _)| AxisId::new(i as u16))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn leg(name: &str, leg: u8, joint: u8) -> Axis {
        Axis {
            name: name.into(),
            role: AxisRole::Leg { leg, joint },
        }
    }

    fn aux(name: &str) -> Axis {
        Axis {
            name: name.into(),
            role: AxisRole::Aux,
        }
    }

    fn namiashi_like() -> AxisTable {
        AxisTable::new(vec![
            leg("FL_hip_joint", 0, 0),
            leg("FL_thigh_joint", 0, 1),
            leg("FL_calf_joint", 0, 2),
            leg("FR_hip_joint", 1, 0),
            leg("FR_thigh_joint", 1, 1),
            leg("FR_calf_joint", 1, 2),
            aux("arm_pitch_joint"),
        ])
        .unwrap()
    }

    #[test]
    fn names_resolve_to_indices() {
        let t = namiashi_like();
        assert_eq!(t.id_of("FR_thigh_joint"), Some(AxisId::new(4)));
        assert_eq!(t.name(AxisId::new(4)), Some("FR_thigh_joint"));
        assert_eq!(t.id_of("nope"), None);
    }

    #[test]
    fn legs_and_aux_are_separated() {
        let t = namiashi_like();
        let legs = t.legs();
        assert_eq!(legs.len(), 2);
        assert_eq!(legs[0].len(), 3);
        assert_eq!(t.aux(), vec![AxisId::new(6)]);
    }

    /// **同じ関節名が 2 度出る表を作れてはいけない。**
    ///
    /// 通してしまうと名前引きが片方だけを指し、もう片方への書き込みが黙って
    /// 捨てられる。症状は「その軸だけ動かない」で、原因に辿り着きにくい。
    #[test]
    fn a_duplicated_joint_name_is_rejected() {
        let e = AxisTable::new(vec![leg("FL_hip_joint", 0, 0), leg("FL_hip_joint", 1, 0)])
            .unwrap_err();
        assert!(e.contains("FL_hip_joint"), "{e}");
    }
}
