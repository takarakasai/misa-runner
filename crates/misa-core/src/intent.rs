//! 操縦者の意図。
//!
//! **チャンネル番号ではなく意味を運ぶ。** どのスイッチが何段かを知っている
//! のは入力側（S.BUS のマッピング）だけで、ここから先は「歩けと言われた」
//! 「脱力しろと言われた」しか見えない。こうしておくと、プロポ・ゲームパッド・
//! キーボード・スクリプト・ネットワークが同じ穴に入り、シミュレータや CI が
//! 台本を流し込めるようになる。

use serde::{Deserialize, Serialize};

use crate::time::Time;

/// 胴体速度の指令。既に実単位へスケール済み。
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Velocity {
    pub vx_m_s: f64,
    pub vy_m_s: f64,
    pub wz_rad_s: f64,
}

impl Velocity {
    pub const ZERO: Velocity = Velocity {
        vx_m_s: 0.0,
        vy_m_s: 0.0,
        wz_rad_s: 0.0,
    };

    /// 厳密にゼロか。
    ///
    /// **等値比較であることに意味がある。** 歩容はちょうど 0 になった瞬間に
    /// 全脚を接地へスナップさせるので、スティックが中立を通過するたびに
    /// 立脚静止へ落ちる。それを鈍らせるのは速度ランプ（時間的ヒステリシス）の
    /// 仕事で、ここで閾値を持たせて誤魔化す場所ではない。
    pub fn is_zero(&self) -> bool {
        self.vx_m_s == 0.0 && self.vy_m_s == 0.0 && self.wz_rad_s == 0.0
    }
}

/// どこまで動いてよいかの要求。
///
/// 並び順が**活動度の低い順**になっていることに意味がある。受信が切れた
/// ときのフェイルセーフは、この順序で**直前より上へ行かない**ことを保証する
/// （[`Self::capped_for_failsafe`]）。
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default, Serialize, Deserialize,
)]
#[serde(rename_all = "snake_case")]
pub enum ModeRequest {
    /// 脱力。
    #[default]
    Relax,
    /// 初期姿勢で保持。
    Stand,
    /// 歩行。
    Walk,
}

impl ModeRequest {
    /// 受信が切れたときに落とし込む先。**活動度を上げない。**
    ///
    /// | 直前 | 受信断後 | 理由 |
    /// |---|---|---|
    /// | `Relax` | `Relax` | **脱力中に受信が切れて立ち上がるのは危ない** |
    /// | `Stand` | `Stand` | 初期姿勢のまま保持 |
    /// | `Walk` | `Walk` | **速度だけ 0 にして、その場で立ったまま**保持 |
    ///
    /// つまり**モードは変えない**。速度をゼロにするのは [`Intent::failsafe`]
    /// の側。
    ///
    /// `Walk` を `Stand` へ丸めてはいけない。中段は「初期姿勢で保持」なので、
    /// 丸めると**歩行中に受信が切れた瞬間に初期姿勢へしゃがみ込む**。
    /// 求めているのは「速度 0・その場起立」。
    ///
    /// **脱力へ落とすのも禁止。** 荷重がかかった四足を脱力させると崩れる。
    pub fn capped_for_failsafe(self) -> Self {
        self
    }
}

/// 歩容の選択。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GaitSelect {
    #[default]
    Crawl,
    Walk,
    Trot,
}

impl GaitSelect {
    pub fn label(self) -> &'static str {
        match self {
            GaitSelect::Crawl => "Crawl",
            GaitSelect::Walk => "Walk",
            GaitSelect::Trot => "Trot",
        }
    }
}

/// 再生するポーズの枠。実際にどのポーズ名かはプロファイルが決める。
///
/// 番号にしているのは、`greeting` / `greeting_alt` のような**機体固有の名前を
/// この層に持ち込まない**ため。意味づけはプロファイルの仕事。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct PoseSlot(pub u8);

/// 1 周期ぶんの意図。
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Intent {
    /// この意図の時刻。入力の古さの判定に使う。
    pub time: Time,
    pub velocity: Velocity,
    /// 胴体の目標姿勢 `[roll, pitch, yaw]` [rad]。
    pub body_attitude_rad: [f64; 3],
    /// 立ち高さの公称値からの差分 [m]。
    pub height_offset_m: f64,
    pub mode: ModeRequest,
    pub gait: GaitSelect,
    /// 立ち上がりで 1 回だけ効く、ポーズ再生の要求。
    pub play_pose: bool,
    /// どの枠を再生するか。
    ///
    /// **再生要求が無くても読める。** 操縦者は押す前に選択を確かめたいので
    /// （`misa-run sbus` の表示がそれ）、`play_pose` に畳んではいけない。
    pub pose_slot: PoseSlot,
    /// 胴体の傾きを打ち消すようにヘッド軸を動かすか（チキンヘッド）。
    pub stabilize_head: bool,
    /// 補助軸への要求または観測 [rad]。並びは
    /// [`crate::axis::AxisTable::aux`] と同じ。**駆動していない軸には
    /// 観測値が入る**（受信機直結の腕など）。
    pub aux_rad: Vec<Option<f64>>,
    /// 操縦入力が生きているか。false なら [`Self::failsafe`] を通す。
    pub link_ok: bool,
}

impl Intent {
    /// 補助軸の要求または観測。並びは [`crate::axis::AxisTable::aux`] と同じ。
    ///
    /// 駆動していない軸には**観測値**が入る（受信機直結の腕など）ので、
    /// 「指令が無い」と「その軸が無い」の区別はここではなく
    /// [`crate::plant::PlantCaps::driven`] が持つ。
    pub fn aux(&self, index: usize) -> Option<f64> {
        self.aux_rad.get(index).copied().flatten()
    }

    /// 受信が切れたときの意図。
    ///
    /// **モードは変えず、速度と姿勢要求だけを落とす。** 立っているなら
    /// 立ったまま、脱力なら脱力のまま。トリガも落とすのは、切れた瞬間の
    /// 立ち上がりを演出の開始と誤読しないため。
    pub fn failsafe(&self) -> Self {
        Self {
            velocity: Velocity::ZERO,
            body_attitude_rad: [0.0; 3],
            mode: self.mode.capped_for_failsafe(),
            play_pose: false,
            stabilize_head: false,
            link_ok: false,
            ..self.clone()
        }
    }
}

/// 意図を作るもの。
///
/// **チャンネル番号を知っているのは実装だけ。** プロポ・ゲームパッド・
/// キーボード・台本・ネットワークが同じ穴に入るので、シミュレータや CI は
/// 台本を流し込める。
pub trait Pilot {
    /// この周期の意図。`now` は [`crate::plant::Plant`] が供給した時刻。
    ///
    /// **入力が切れていても意図は返す。** 何も返さない選択肢を作ると、
    /// 呼び出し側が「前回の意図を使い回す」ことになり、受信断で速度が
    /// 残り続ける。切れたことは [`Intent::link_ok`] で伝える。
    fn poll(&mut self, now: Time) -> Intent;

    /// 状態表示に添える 1 行。受信の生きの良さなど、**この入力にしか
    /// 分からないこと**を書く。既定は空。
    fn status_line(&self) -> String {
        String::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn walking() -> Intent {
        Intent {
            velocity: Velocity {
                vx_m_s: 0.2,
                vy_m_s: 0.1,
                wz_rad_s: 0.3,
            },
            body_attitude_rad: [0.1, 0.2, 0.3],
            mode: ModeRequest::Walk,
            play_pose: true,
            pose_slot: PoseSlot(1),
            stabilize_head: true,
            link_ok: true,
            ..Intent::default()
        }
    }

    /// **受信断でモードを変えない。**
    ///
    /// かつて一律「起立」を返しており、脱力中に受信が切れると立ち上がって
    /// いた。逆に `Stand` へ丸めるのも誤りで、歩行中の受信断でしゃがみ込む。
    #[test]
    fn losing_the_link_stops_the_robot_without_changing_its_mode() {
        let f = walking().failsafe();
        assert_eq!(f.mode, ModeRequest::Walk);
        assert!(f.velocity.is_zero());
        assert_eq!(f.body_attitude_rad, [0.0; 3]);
        assert!(!f.link_ok);
    }

    #[test]
    fn losing_the_link_does_not_fire_a_pose() {
        let f = walking().failsafe();
        assert!(!f.play_pose);
        // どの枠が選ばれているかは残る。落とすのは「いま撃て」のほうだけ。
        assert_eq!(f.pose_slot, PoseSlot(1));
        assert!(!f.stabilize_head);
    }

    #[test]
    fn the_failsafe_never_raises_activity() {
        for m in [ModeRequest::Relax, ModeRequest::Stand, ModeRequest::Walk] {
            let before = Intent {
                mode: m,
                ..Intent::default()
            };
            assert!(before.failsafe().mode <= m, "{m:?} で活動度が上がった");
        }
    }

    /// 速度ゼロの判定は厳密な等値であること。歩容がそれを前提にしている。
    #[test]
    fn zero_velocity_is_an_exact_comparison() {
        assert!(Velocity::ZERO.is_zero());
        assert!(!Velocity {
            vx_m_s: 1e-12,
            ..Velocity::ZERO
        }
        .is_zero());
    }
}
