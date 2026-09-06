//! アプリ設定（TOML）。実機設定 (`misa_hal::config`) と同じファイルに同居する。
//!
//! ファイル 1 枚に `[hardware]` と `[control] [gait] [teleop] [poses]` を並べる
//! 形にしてある。配線とチューニングを別ファイルに分けると、現場で片方だけ
//! 持ち出して食い違う。

use misa_hal::config::HardwareConfig;
use serde::{Deserialize, Serialize};

use crate::teleop::TeleopConfig;

/// 設定全体。
///
/// **`name` はファイルの先頭に置くこと。** TOML ではテーブル見出しより後ろに
/// 書いた素のキーは、そのテーブルの中身として読まれる。フィールド順が
/// そのまま `to_toml` の出力順になるので、ここの並びが仕様になる。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AppConfig {
    /// ロボット名。プロファイル 1 枚 = ロボット 1 台の識別子。
    ///
    /// ログの見出しに出るほか、**実行時の資源名に入る**（脚バスの flock、
    /// マルチターン原点の目印）。同一ホストで 2 台動かしたときに、
    /// 一方のロックがもう一方を弾かないようにするため。
    /// パスに入るので [`AppConfig::validate`] で文字種を縛っている。
    #[serde(default = "default_robot_name")]
    pub name: String,
    #[serde(default)]
    pub control: ControlConfig,
    #[serde(default)]
    pub gait: GaitTuning,
    #[serde(default)]
    pub teleop: TeleopConfig,
    #[serde(default)]
    pub poses: PoseConfig,
    #[serde(default)]
    pub hardware: HardwareConfig,
    /// 全身制御（WBC）。**既定は無効**で、書かなければ従来どおり歩容の
    /// IK 出力をそのまま位置制御で流す。
    #[serde(default)]
    pub wbc: WbcConfig,
    /// 脚以外の軸。**機体ごとに数も役割も違う。**
    ///
    /// namiashi は腕 1 軸、namiashi2 は車輪 4 軸。歩容は触らないので、ここに
    /// 書いてあるかどうかで軸表の長さと `Command` / `Observation` の長さが
    /// 決まる。書かなければ補助軸なしの機体になる。
    #[serde(default = "default_aux_axes")]
    pub aux: Vec<AuxAxis>,
}

/// 脚以外の軸 1 本。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AuxAxis {
    /// モデル（`.misa`）の関節名。軸表の照合キー。
    pub joint: String,
    #[serde(default)]
    pub role: AuxRole,
}

/// 補助軸の役割。**何に使う軸かは制御則の側の関心事**なので、
/// ここで名指しする（機体固有の名前は持ち込まない）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuxRole {
    /// 胴体の傾きを打ち消す軸（チキンヘッド）。**1 台に 1 本まで。**
    Head,
    /// 車輪。歩容もチキンヘッドも触らない。
    Wheel,
    /// それ以外。観測だけする。
    #[default]
    Other,
}

/// namiashi の腕。プロファイルが `[[aux]]` を書かなかったときの既定。
fn default_aux_axes() -> Vec<AuxAxis> {
    vec![AuxAxis {
        joint: misa_hal::joint::ARM_JOINT_NAME.into(),
        role: AuxRole::Head,
    }]
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            name: default_robot_name(),
            control: ControlConfig::default(),
            gait: GaitTuning::default(),
            teleop: TeleopConfig::default(),
            poses: PoseConfig::default(),
            hardware: HardwareConfig::default(),
            wbc: WbcConfig::default(),
            aux: default_aux_axes(),
        }
    }
}

/// プロファイルが名前を書いていないときの既定。
///
/// 実機を持つプロファイルは必ず自分の名前を書く前提だが、`--robot` を
/// 付けずに既定値で立ち上げる経路があるので、そこでも資源名が決まるように
/// しておく。
fn default_robot_name() -> String {
    "robot".into()
}

impl AppConfig {
    pub fn from_toml(text: &str) -> Result<Self, String> {
        // **`[hardware]` が無いファイルは設定として受け取らない。**
        //
        // 全フィールドに serde の既定値が入っているので、**TOML なら何でも
        // 「設定」として通ってしまう**。実際、モデルの `.misa` を
        // `--config` に渡すと「設定: OK」と出て、ゼロ点 0 / 符号 +1 の
        // **未校正の既定値**で実機を動かせてしまった。
        //
        // 既定値で走らせたい正規の入口は `--config` を**付けない**こと。
        // ファイルを指定した以上、その中身が設定であることを確かめる。
        let raw: toml::Value =
            toml::from_str(text).map_err(|e| format!("TOML の解析に失敗: {e}"))?;
        if raw.get("hardware").is_none() {
            let looks_like_model = raw.get("pose").is_some() || raw.get("joint").is_some();
            return Err(format!(
                "設定ファイルに [hardware] がありません{}。\
                 既定値で走らせたいなら --config を付けないでください",
                if looks_like_model {
                    "（`[[pose]]` があります — **モデルファイルを渡していませんか**）"
                } else {
                    ""
                }
            ));
        }
        let cfg: AppConfig = toml::from_str(text).map_err(|e| format!("TOML の解析に失敗: {e}"))?;
        cfg.validate()?;
        Ok(cfg)
    }

    pub fn load(path: impl AsRef<std::path::Path>) -> Result<Self, String> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("{} を読めません: {e}", path.display()))?;
        Self::from_toml(&text)
    }

    /// トルク指令の単位が実機と食い違っていれば、その理由を返す。
    ///
    /// **`torque_constant_nm_per_a` を書いていないシリアル構成では、
    /// `set_torque` に渡した数がそのまま電流 (A) として線に乗る**
    /// （`lkmotor_driver::MotorConfig::current_units` が `Kt = 1/減速比` を
    /// 選ぶため）。WBC が出すのは N·m なので、そのまま流すと 12 軸ぶんの
    /// N·m が A に読み替えられる。namiashi の定格 1.5 N·m なら 1 軸 1.5 A、
    /// 12 軸で 18 A — 電源の電流制限 5 A（`doc/motor_map.md`）を大きく超えて
    /// レールが崩壊する。**復帰できても、その瞬間に 12 軸が同時に脱力する。**
    ///
    /// **これは実機（シリアル）だけの話。** `sim` のトルクは MuJoCo へ行く
    /// ので単位は N·m のまま正しい。したがって設定の検証では落とさず、
    /// 実機の [`misa_core::PlantCaps`] から `Torque` を外す形で止める
    /// （[`crate::plant::SerialPlant`]）。ここはその理由を人に見せるため。
    pub fn torque_unit_mismatch(&self) -> Option<String> {
        let serial = self.hardware.serial().ok()?;
        if serial.legs.torque_constant_nm_per_a.is_some() {
            return None;
        }
        Some(
            "[hardware.legs] に torque_constant_nm_per_a がないので、\
             トルク指令は N·m ではなく電流 (A) として線に乗ります"
                .into(),
        )
    }

    pub fn to_toml(&self) -> Result<String, String> {
        toml::to_string_pretty(self).map_err(|e| format!("TOML の生成に失敗: {e}"))
    }

    /// 傾きを報告するしきい値 [rad]。`0` なら無効。
    ///
    /// 省略時は**意図して傾ける量 + 0.3 rad**（最低 0.5 rad = 29°）。
    /// `sim` が転倒と判定するのは 1 rad なので、その内側で先に声を上げる。
    pub fn max_tilt_rad(&self) -> f64 {
        match self.control.max_tilt_rad {
            Some(v) => v,
            None => (self.gait.body_attitude_max_rad + 0.3).max(0.5),
        }
    }

    pub fn validate(&self) -> Result<(), String> {
        // **名前はファイル名になる。** flock と目印のパスに入るので、
        // `/` や `..` が混ざると別のディレクトリを触りに行く。
        if self.name.is_empty() {
            return Err("name が空です。ロボット名を書いてください".into());
        }
        if !self
            .name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
        {
            return Err(format!(
                "name {:?} に使えない文字があります。\
                 英数字と - _ だけにしてください（実行時のパスに入るため）",
                self.name
            ));
        }
        self.hardware.validate().map_err(|e| e.to_string())?;
        if self.control.rate_hz <= 0.0 {
            return Err("control.rate_hz は正の値が必要です".into());
        }
        // 制御周期がバス周期より速いと、同じ指令を 2 回送るだけで意味がない。
        if let Some(bus_hz) = self.hardware.max_control_rate_hz() {
        if self.control.rate_hz > bus_hz {
            return Err(format!(
                "control.rate_hz ({}) が legs.bus_rate_hz ({}) を超えています。\
                 バスが追いつかないので制御周期を落とすか、バス周期を上げてください",
                self.control.rate_hz, bus_hz
            ));
        }
        }
        // **チキンヘッドの相手は 1 本まで。** 2 本あるとどちらを動かすか
        // 決まらず、片方が黙って無視される。
        let heads = self.aux.iter().filter(|a| a.role == AuxRole::Head).count();
        if heads > 1 {
            return Err(format!("role = \"head\" の補助軸が {heads} 本あります。1 本までです"));
        }
        self.teleop.validate()?;
        self.wbc.validate()?;
        if self.gait.max_vx_m_s <= 0.0
            || self.gait.max_vy_m_s <= 0.0
            || self.gait.max_wz_rad_s <= 0.0
        {
            return Err("gait の速度上限は正の値が必要です".into());
        }
        // **傾きの報告は、意図して傾ける量より外側でしか意味を持たない。**
        // 指令どおり傾けただけで「転倒しかけ」と出ると、本当の転倒と
        // 区別が付かなくなり、そのうち誰も読まなくなる。省略時は導出値なので
        // この矛盾は起こらない。明示的に書いたときだけ突き合わせる。
        if let Some(v) = self.control.max_tilt_rad {
            if v < 0.0 {
                return Err("control.max_tilt_rad は 0 以上（0 で無効）".into());
            }
            if v > 0.0 && v <= self.gait.body_attitude_max_rad {
                return Err(format!(
                    "control.max_tilt_rad ({}) が gait.body_attitude_max_rad ({}) 以下です。\
                     指令どおり傾けただけで転倒と報告されるので、姿勢の上限より\
                     大きく取ってください",
                    v, self.gait.body_attitude_max_rad
                ));
            }
        }
        Ok(())
    }
}

/// 膝の曲がる向き。`quadruped_gait::KneePattern` に 1 対 1 で対応する。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum KneeShape {
    /// `<<` 4 脚とも後ろ向き。
    #[default]
    BothBack,
    /// `<>` 前が後ろ向き・後ろが前向き（哺乳類型）。
    MammalianForward,
    /// `><` 前が前向き・後ろが後ろ向き。
    MammalianReverse,
    /// `>>` 4 脚とも前向き。
    BothForward,
}

/// WBC（全身制御）の出力の出し方。
///
/// **1 回の QP の解 `(q̈, f_GRF, τ)` を、どの量にして実機へ出すか**という
/// 選択で、解そのものは 3 つとも同じ。τ をそのまま出すのが素直だが、
/// トルク制御はモータ側の電流ループとゲインの素性が要る。位置・速度は
/// q̈ を 1 回 / 2 回積分して参照にするので、**既存の位置制御の口をそのまま
/// 使いながら WBC の解を通せる**（実機へ持っていくときの中間段）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WbcOutput {
    /// τ をそのまま出す（[`misa_core::ControlMode::Torque`]）。
    ///
    /// **WBC 本来の出し方。** 接触力と姿勢が同じ QP の中で釣り合っている
    /// ので、位置・速度へ積分し直したときのような時間遅れが入らない。
    Torque,
    /// `q̇* = q̇ + q̈·dt` を出す（[`misa_core::ControlMode::Velocity`]）。
    Velocity,
    /// `q* = q + q̇·dt + ½·q̈·dt²` を出す（[`misa_core::ControlMode::Position`]）。
    ///
    /// **既定。** 実機で唯一実績のある口で、τ は `torque_ff_nm` に載せる
    /// だけなので、MIT を持つ機体では前置トルクとして効き、持たない機体
    /// では無視される。
    #[default]
    Position,
}

impl WbcOutput {
    pub fn label(self) -> &'static str {
        match self {
            WbcOutput::Torque => "トルク",
            WbcOutput::Velocity => "速度",
            WbcOutput::Position => "位置",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "torque" | "trq" => Some(WbcOutput::Torque),
            "velocity" | "vel" => Some(WbcOutput::Velocity),
            "position" | "pos" => Some(WbcOutput::Position),
            _ => None,
        }
    }
}

/// WBC（階層 QP による全身制御）の設定。
///
/// # 何を解いているのか
///
/// `quadruped_gait::wbc` の 3 優先度 HoQP に、毎周期
/// `x = [q̈ | f_GRF | τ]` を解かせる。優先度 0 は**物理の制約**（浮遊ベースの
/// 運動方程式・摩擦錐・トルク上限・立脚足が滑らないこと）、優先度 1 が
/// 胴体加速度と遊脚の追従、優先度 2 が接地力と重力補償トルクへの寄せ。
///
/// # 参照は歩容ではなくここで作る
///
/// **この機体の歩容（CHAMP 系）は MPC を持たない**ので、`predicted_grfs()` は
/// 常に `None`。したがって WBC が要る `a_base_des` と `f_grf_des` は
/// [`crate::wbc`] が自前で作る: 接地足は歩容の立脚フラグ、接地力は体重の
/// 静的配分、胴体加速度は IMU 姿勢への PD。**準静的な参照**なので、
/// 立位と低速の crawl では成立するが、跳ぶ・走る歩容には足りない。
/// MPC 歩容を入れたらここを差し替える（層は分けてある）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct WbcConfig {
    /// **既定は無効。** 有効にすると脚 12 軸の指令が WBC の解に変わる。
    /// 実機でいきなり入れるものではないので、明示的に立てさせる。
    pub enabled: bool,
    /// 解をどの量にして出すか。
    pub output: WbcOutput,
    /// 接地摩擦係数。摩擦錐（優先度 0 の硬い制約）に入る。
    pub friction_mu: f64,
    /// 立脚と計画した足に要求する最小垂直力 [N]。
    ///
    /// 0 だと「押すだけ」の錐になり、3 点接地の計画を 2 点で満たす解が
    /// 通ってしまう。**硬い制約なので、1 脚あたりの実際の分担より十分
    /// 小さく取る**（大きいと着地の過渡で解なしになる）。
    pub f_min_stance_n: f64,
    /// モデルの `[joint.limit] effort` に掛ける係数。**軸ごとの比を保つ。**
    ///
    /// # なぜ係数なのか
    ///
    /// `.misa` の `effort` は**連続定格**（namiashi は hip / thigh 1.5、
    /// calf 2.205 N·m。減速比で割るとどちらもモータ軸 0.14〜0.15 N·m で、
    /// 同じ 1 個の数字が比で配られているのが分かる）。モータは瞬間的には
    /// それより出るので、WBC に連続定格を渡すと出せるはずの力を使わない。
    ///
    /// 絶対値で 1 本書くと軸ごとの比（calf は hip の 1.47 倍）が潰れるので、
    /// **係数で持つ**。`2.0` なら「瞬間は連続定格の 2 倍まで」。
    ///
    /// **namiashi の実機（LKMTech MG4005E-i10）は 24 V で瞬時 2.5 N·m**
    /// （減速機出力。連続定格 1.5 N·m）なので **1.667**。同梱の
    /// `robots/namiashi.toml` に書いてある。
    ///
    /// **上げれば歩けるようになる、という話ではない**
    /// （[`crate::wbc`] の「トルク定格は前進速度を縛っていない」）。
    /// 上げるのは、飽和が原因で姿勢を戻せていないと分かったときだけ。
    pub torque_scale: f64,
    /// トルク上限の絶対値 [N·m]。`0` で無効。
    ///
    /// **モデル × [`Self::torque_scale`] より小さいときだけ効く（頭打ち）。**
    /// 実機の電流リミットや電源の都合をモデルより手前へ置きたいときに使う。
    /// ここを大きくしてもモデルの上には行かない — 上げるなら
    /// `torque_scale` のほう。
    pub max_torque_nm: f64,
    /// 遊脚の**直交空間** PD ゲイン。単位は 1/s² と 1/s で、**足先の位置
    /// 誤差 [m] に掛かる**（関節角ではない）。
    ///
    /// legged_control の `formulateSwingLegTask` と同じ形:
    ///
    /// ```text
    ///   J_足 · q̈ = kp·(p* − p) + kd·(ṗ* − ṗ) − J̇·v
    /// ```
    ///
    /// **関節空間の PD ではないので桁が違う。** legged_control の既定は
    /// 350 / 37（ζ ≈ 1.0）で、そのまま使っている。関節空間で書くと脚の
    /// 姿勢によってヤコビアンのぶん効きが変わり、伸び切った脚で効かなく
    /// なる — 直交空間なら足先で一定。
    pub swing_kp: f64,
    pub swing_kd: f64,
    /// 補助軸（腕）の**関節空間** PD ゲイン。
    ///
    /// 腕は足ではないので直交空間のタスクが書けない。実機では位置で
    /// 保持されているので、WBC にも「そこに留まる」と関節空間で伝える。
    pub aux_kp: f64,
    pub aux_kd: f64,
    /// τ と一緒に出す関節 PD のゲイン [N·m/rad, N·m·s/rad]。
    /// **`output = "torque"` のときだけ効く。**
    ///
    /// # なぜ純粋なトルクではないのか
    ///
    /// legged_control は `setCommand(pos_des, vel_des, kp=5, kd=3, torque)`
    /// で、**トルクに弱い関節 PD を添えて**出す（hybrid joint）。WBC が出す
    /// のは加速度であって位置ではなく、位置の誤差を戻す積分器がどこにも
    /// 無いので、τ だけだとモデル誤差のぶん関節がずれ続ける。この PD が
    /// その錨になる。
    ///
    /// **位置サーボのゲインとは桁が違う**（あちらは 60〜300）。強くすると
    /// PD が主役になって WBC の解が埋もれる。
    ///
    /// LKMTech は MIT を持たないので、**ホスト側で足して 1 本のトルクに
    /// してから出す**（`τ = τ_WBC + kp·(q*−q) + kd·(q̇*−q̇)`）。
    /// 両方 0 にすれば純粋なトルクになる。
    ///
    /// # `joint_kd` は legged_control の 3 をそのまま使ってはいけない
    ///
    /// あちらの kd=3 は**モータ内 2.5 kHz** の PD のもの。ここは制御周期
    /// （200 Hz）で回るホスト側の PD なので、脚リンクの慣性（1e-3 kg·m²
    /// 程度）に対して kd·dt/I が 2 を超えて**離散の微分項が発振する**。
    /// 実測（MuJoCo、立ち止まり）: kd=3 では起動した最初の周期から τ が
    /// ±18 N·m で毎周期符号を替え、上限で削られた結果として胴体が 0.20 →
    /// 0.03 m まで沈んだ。kd=0.3 以下なら τ_WBC が Plant の実トルクと
    /// 一致して 0.200 m に立つ。5 走行の平均追従率は kd=0 で 1.21（横ずれ
    /// 0.83 m・ヨー 4.8°）、kd=0.3 で 1.18（0.46 m・0.4°）、kd=1.0 で 0.96
    /// （ヨー −44°）。**既定は 0.3。** モータ側に PD がある機体（MIT モード）
    /// で初めて 3 に戻せる。
    pub joint_kp: f64,
    pub joint_kd: f64,
    /// 立脚の関節空間 PD ゲイン。**既定は 0（＝タスクを置かない）。**
    ///
    /// legged_control は立脚に関節タスクを置かない（遊脚だけ）。接地拘束の
    /// 下では胴体の加速度を決めれば立脚の関節加速度も決まるので、ここへ
    /// タスクを置くと `base_accel` と同じ自由度を取り合うことになる。
    ///
    ///
    /// # `base_accel` と同じ自由度を取り合っていること
    ///
    /// 4 脚接地のとき、足が滑らないという制約（優先度 0、12 式）は脚 12
    /// 関節と胴体 6 自由度を結び付ける。したがって**胴体の加速度を決めれば
    /// 立脚の関節加速度も決まる**（自由度が重なっている）。立脚に関節
    /// タスクを置くと、`base_accel` と同じ 6 自由度を別の言葉で取り合う
    /// ことになり、勝つのは重みの大きいほう。
    ///
    /// **どちらか一方では足りない**（MuJoCo の LinearCrawl・16 秒・
    /// 前進 0.05 で実測）:
    ///
    /// | 立脚タスク | 関節重み / 胴体重み | 進む量 |
    /// |---|---|---|
    /// | 無し | — / 200 | +0.078 m |
    /// | 60 / 6 | 20 / 200 | +0.123 m |
    /// | **60 / 6** | **200 / 50** | **+0.315 m** |
    /// | 200 / 20 | 400 / 20 | +0.283 m（ヨーが −36°へ流れる）|
    ///
    /// 胴体側だけだと「支えるが進まない」（前へ送るには接地力で胴体を
    /// 加速するしかなく、この機体のトルク定格では足りない）。関節側だけを
    /// 強くすると姿勢を戻す力が消えてヨーが流れる。既定はその中間。
    pub stance_kp: f64,
    pub stance_kd: f64,
    /// 胴体姿勢（roll / pitch）の PD ゲイン。`a_base_des` の角加速度に入る。
    pub attitude_kp: f64,
    pub attitude_kd: f64,
    /// ヨーの PD ゲイン。**別に持つのは効き方が違うから**で、roll/pitch は
    /// 転倒に直結するが yaw は向きが変わるだけ。既定は弱い。
    pub yaw_kp: f64,
    pub yaw_kd: f64,
    /// 胴体高さの比例ゲイン。`a_base_des` の並進 z に入る。
    pub height_kp: f64,
    pub height_kd: f64,
    /// **支持多角形に対する胴体の水平位置**の比例ゲイン。
    ///
    /// # なぜ水平位置に帰還が要るか
    ///
    /// crawl は 1 本ずつ脚を上げるので、支持は 3 点になる。倒れないため
    /// には胴体（重心）が支持三角形の内側へ寄っている必要があり、CHAMP は
    /// それを**足の置き場所**で作る — 位置制御なら脚が動けば胴体も付いて
    /// くるので、それで足りる。トルク制御ではそうならない: 胴体は接地力で
    /// しか動かず、「寄れ」と言わなければ寄らない。実際、これが無いと
    /// MuJoCo の crawl で最初の遊脚が上がった瞬間に横へ倒れた。
    ///
    /// 誤差は**歩容の計画した足位置と実測の足位置の差**として測る（接地足
    /// の平均）。世界座標の絶対位置は測れないが、支持足に対する相対位置は
    /// 測れて、balance に効くのはそちらだけ。
    pub position_kp: f64,
    /// 胴体の並進速度を寄せるゲイン [1/s]。水平は速度指令へ、鉛直は 0 へ。
    /// 速度は接地足の FK から測る（脚オドメトリ）。
    pub velocity_kd: f64,
    /// 機体質量 [kg]。`0` ならモデルのリンク質量の総和を使う。
    /// 接地力の静的配分（`m·g / 立脚数`）に効く。
    pub mass_kg: f64,
    /// 優先度 1 の 2 つのタスクの重み。**胴体姿勢と関節追従の綱引き。**
    ///
    /// `joint_track` は**遊脚の直交空間タスクと補助軸の関節タスク**の重み。
    ///
    /// **効くのは比だけ。** legged_control は優先度の中で重みを付けない
    /// （どのタスクも 1）。MuJoCo の trot（前進 0.80、トルク出力）で振ると:
    ///
    /// | base_accel / joint_track | 進む量 | ヨーのずれ |
    /// |---|---|---|
    /// | 200 / 1 | +1.94 m | +102° |
    /// | 200 / 20 | +0.62 m | +107° |
    /// | 200 / 200 | +4.36 m | +8.0° |
    /// | **50 / 200（既定）** | +4.07 m | +42.6° |
    /// | 1 / 1 | +3.32 m | +16.0° |
    ///
    /// **片方だけを強くすると、もう片方が担っていた自由度が投げ出される。**
    /// trot だけなら 200/200 が最良だが、walk と crawl では落ちる（6 条件の
    /// 合計で 50/200 が上）。**この値は歩容と速度に依るので、詰めるときは
    /// 1 条件で決めないこと。**
    pub weight_base_accel: f64,
    pub weight_joint_track: f64,
    /// 優先度 2 の重み。接地力を参照へ寄せる強さと、τ を重力補償値へ
    /// 寄せる強さ（`τ ≈ 0` の退化解を止める錨）。
    pub weight_contact_force: f64,
    pub weight_tau_gravity: f64,
    /// **参照に積む関節加速度**の頭打ち [rad/s²]。`0` で無効。
    ///
    /// 位置・速度出力は WBC の `q̈` を 1 周期ぶん積んで参照にする。MPC の
    /// 参照では姿勢が崩れ始めた瞬間に `q̈` が 600 rad/s² 級になり、速度
    /// 出力ではそれが 3 rad/s の跳びとして脚に出る。**トルク出力には
    /// 掛からない**（あちらが出すのは τ で、q̈ は参照にしか使わない）。
    pub max_joint_accel: f64,
    /// `a_base_des` の頭打ち。並進 [m/s²] と角 [rad/s²]。`0` で無効。
    ///
    /// # なぜ要るか
    ///
    /// **機械が出せない加速度を要求しても、QP はほかのタスクを犠牲にする
    /// だけ。** MPC の角加速度は `α = I⁻¹·(Σr×f − ω×Iω)` で作られるが、
    /// namiashi の慣性は `diag(0.008, 0.034, 0.034) kg·m²` と小さいので、
    /// 姿勢が崩れ始めると 600 rad/s² といった値がすぐ出る。それを 1 周期
    /// 積分して速度指令にすると 3 rad/s の跳びになり、速度出力では脚が
    /// そのまま飛ぶ（実測で MuJoCo の trot が後ろへ 1 m 走った）。
    ///
    /// 既定は namiashi で出せる範囲から: 並進は摩擦（μ=0.7）で 7 m/s²
    /// 前後、角は接地力の付け替えで作れるピッチが 100 rad/s² 前後。
    pub max_base_accel_lin: f64,
    pub max_base_accel_ang: f64,
    /// 実測の接地で歩容の立脚フラグを上書きするか。**既定は無効。**
    ///
    /// # なぜ既定で無効なのか
    ///
    /// 考え方は正しい — WBC の「立脚足が滑らない」は硬い制約なので、着地が
    /// 計画より早い相を渡せないと trot は壊れる（articara の WBC 検証も
    /// `ContactDrivenPhase` で同じことをしている）。
    ///
    /// **足りなかったのは力の閾値。** あちらは 5 N を超えたときだけ立脚へ
    /// 倒すが、[`misa_core::Observation::contacts`] は真偽値しか持たないので、
    /// 幾何の接触をそのまま入れると遊脚がかすっただけでも立脚に倒れた
    /// （MuJoCo の trot で進む量 9.50 → 8.43 m、ヨー 29° → 61°）。
    /// 2026-09-06 から MuJoCo の Plant は [`Self::contact_force_threshold_n`]
    /// で切った真偽値を報告するので、この問題は無い。既定が無効なのは、
    /// 実機に足裏センサが無く（`None` が並んで計画がそのまま通る）、
    /// sim と実機で挙動を揃えるため。
    pub use_measured_contact: bool,
    /// 足を接地と報告する垂直力の閾値 [N]。**力を測れる Plant（MuJoCo）
    /// だけが使う。** articara の `ContactDrivenPhase` と同じ 5 N。
    pub contact_force_threshold_n: f64,
    /// MPC の接地力の予測を鈍らせる係数（`鈍 = α·新 + (1−α)·前`）。
    /// `1.0` で鈍らせない。**MPC 歩容のときだけ効く。**
    ///
    /// QP は広い零空間から周期ごとに少しずつ違う最適解を拾うので、生のまま
    /// 参照にすると接地力のタスクが震える（articara の実測で 13 → 68 →
    /// 47 N）。鈍らせるのは**参照だけ**で、τ の前置きに使う生の値には
    /// 触らない。
    pub grf_smoothing: f64,
    /// QP の warm start の重み。0 で毎周期コールドスタート。
    ///
    /// 解の空間が広いので、何もしないと同じ姿勢でも周期ごとに違う解を
    /// 拾って指令が震える。`1e-3` 付近から。
    pub prox_weight: f64,
    /// 速度出力で位置誤差を潰す比例ゲイン [1/s]。
    ///
    /// **速度指令だけでは関節位置が漂う。** 速度制御は位置のループを
    /// 持たないので、モデル誤差と外乱のぶんだけ積分されて歩容の目標から
    /// 離れていく。`q̇* = q̇_計画 + q̈·dt + kp·(q_計画 − q_実測)` の kp。
    /// 位置・トルク出力では使わない。
    pub velocity_track_kp: f64,
    /// 遊脚の**加速度誤差積分**（Grandia 2022 式 39–40）のゲイン
    /// [N·m·s/rad]。**0 で無効（既定）。`output = "torque"` のときだけ効く。**
    ///
    /// WBC は加速度 q̈ を出すが、モデル誤差・摩擦のぶん脚は解いた通りに
    /// 加速しない。離地の瞬間の q̇ に WBC の q̈ を積んだ「解通りなら今こう
    /// 動いているはず」の速度と実測との差にゲインを掛けて τ に足す:
    ///
    /// ```text
    ///   τ_i += -K · (q̇_i − q̇_i(t_sw) − ∫_{t_sw}^{t} q̈_i,wbc dt)
    /// ```
    ///
    /// 遊脚だけ・離地でリセット・飽和付き。立脚には置かない（接地拘束の
    /// 下で速度を追わせると接地力と喧嘩する）。
    pub swing_accel_integral_k: f64,
    /// 上の項の飽和 [N·m]。積分なので外れ値を溜め込む。0.5 N·m は hip の
    /// 定格 1.5 の 1/3。
    pub swing_accel_integral_sat_nm: f64,
}

impl Default for WbcConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            output: WbcOutput::default(),
            friction_mu: 0.7,
            f_min_stance_n: 0.5,
            torque_scale: 1.0,
            max_torque_nm: 0.0,
            swing_kp: 350.0,
            swing_kd: 37.0,
            aux_kp: 100.0,
            aux_kd: 10.0,
            joint_kp: 5.0,
            joint_kd: 0.3,
            stance_kp: 0.0,
            stance_kd: 0.0,
            attitude_kp: 60.0,
            attitude_kd: 8.0,
            yaw_kp: 10.0,
            yaw_kd: 2.0,
            height_kp: 100.0,
            height_kd: 20.0,
            position_kp: 100.0,
            velocity_kd: 10.0,
            mass_kg: 0.0,
            weight_base_accel: 50.0,
            weight_joint_track: 200.0,
            weight_contact_force: 5.0,
            weight_tau_gravity: 5.0,
            max_joint_accel: 200.0,
            max_base_accel_lin: 10.0,
            max_base_accel_ang: 100.0,
            use_measured_contact: false,
            contact_force_threshold_n: 5.0,
            grf_smoothing: 0.3,
            prox_weight: 1e-3,
            velocity_track_kp: 20.0,
            swing_accel_integral_k: 0.0,
            swing_accel_integral_sat_nm: 0.5,
        }
    }
}

impl WbcConfig {
    /// その軸のトルク上限 [N·m]。`declared` はモデルの `effort`。
    ///
    /// **モデル × 係数を、絶対値の上限で頭打ちにする。** どちらも無ければ
    /// `0`（＝制限しない）で、[`crate::wbc::WbcLayer::new`] はそれを拒む。
    pub fn torque_ceiling(&self, declared: f64) -> f64 {
        let scaled = declared * self.torque_scale;
        match (scaled > 0.0, self.max_torque_nm > 0.0) {
            (true, true) => scaled.min(self.max_torque_nm),
            (true, false) => scaled,
            (false, true) => self.max_torque_nm,
            (false, false) => 0.0,
        }
    }

    pub fn validate(&self) -> Result<(), String> {
        if !self.enabled {
            return Ok(());
        }
        if self.friction_mu <= 0.0 {
            return Err("wbc.friction_mu は正の値が必要です".into());
        }
        if self.f_min_stance_n < 0.0 {
            return Err("wbc.f_min_stance_n は 0 以上が必要です".into());
        }
        if self.max_joint_accel < 0.0 {
            return Err("wbc.max_joint_accel は 0 以上が必要です".into());
        }
        if self.max_base_accel_lin < 0.0 || self.max_base_accel_ang < 0.0 {
            return Err("wbc.max_base_accel_* は 0 以上が必要です".into());
        }
        if !(0.0..=1.0).contains(&self.grf_smoothing) {
            return Err("wbc.grf_smoothing は 0〜1 が必要です".into());
        }
        if self.torque_scale <= 0.0 {
            return Err("wbc.torque_scale は正の値が必要です".into());
        }
        if self.mass_kg < 0.0 {
            return Err("wbc.mass_kg は 0 以上が必要です（0 ならモデルから取ります）".into());
        }
        if self.velocity_track_kp < 0.0 {
            return Err("wbc.velocity_track_kp は 0 以上が必要です".into());
        }
        Ok(())
    }
}

/// どの歩容コントローラを使うか。**歩容の型（Crawl / Walk / Trot、＝踏み
/// 替えの並び）とは別の軸**で、そちらはプロポの CH6 が選ぶ。
///
/// | | 接地力の予測 | 観測 | 備考 |
/// |---|---|---|---|
/// | `champ` | 無し | 見ない | 既定。開ループの運動学 |
/// | `linear_crawl` | 無し | 見ない | 胴体を +X 直線に載せる。**横移動と旋回を受け付けない** |
/// | `mpc` | SRBD | 速度 | 胴体を 1 剛体と見た MPC |
/// | `centroidal` | 重心 SRBD | 速度 | 重心のずれと慣性を持つ版 |
///
/// **MPC 系は `predicted_grfs()` を出す**ので、WBC の参照が準静的な自前の
/// ものから MPC の予測に変わる（[`crate::wbc`]）。そこが入る唯一の効き目で、
/// WBC を無効にしたまま MPC を選んでも、接地力の予測は前置トルクにしか
/// 使われない。
/// 胴体の状態推定の方式。[`GaitTuning::estimator`]。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EstimatorKind {
    #[default]
    LegOdometry,
    Kalman,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GaitControllerKind {
    /// **既定。** `crawl_use_linear` の従来どおりの解釈（Crawl だけ
    /// `LinearCrawl`、ほかは CHAMP）。
    #[default]
    Auto,
    Champ,
    LinearCrawl,
    Mpc,
    Centroidal,
}

impl GaitControllerKind {
    pub fn label(self) -> &'static str {
        match self {
            GaitControllerKind::Auto => "自動",
            GaitControllerKind::Champ => "CHAMP",
            GaitControllerKind::LinearCrawl => "LinearCrawl",
            GaitControllerKind::Mpc => "MPC (SRBD)",
            GaitControllerKind::Centroidal => "MPC (重心)",
        }
    }

    /// 接地力の予測を出すか。**WBC の参照がこれで変わる。**
    pub fn has_mpc(self) -> bool {
        matches!(self, GaitControllerKind::Mpc | GaitControllerKind::Centroidal)
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "auto" => Some(GaitControllerKind::Auto),
            "champ" => Some(GaitControllerKind::Champ),
            "linear_crawl" | "linear" => Some(GaitControllerKind::LinearCrawl),
            "mpc" | "srbd" => Some(GaitControllerKind::Mpc),
            "centroidal" => Some(GaitControllerKind::Centroidal),
            _ => None,
        }
    }
}

/// 制御ループ全体の設定。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlConfig {
    /// ロボットモデル (`.misa`)。ポーズ・シーケンスもここから読む。
    #[serde(default = "default_model_path")]
    pub model: String,
    /// **シムと机上再生の初期姿勢。実機では使わない。**
    ///
    /// **脱力した姿勢は一意に決まらない**（どう置いたか、どこで摩擦が
    /// 止まるかで変わる）ので、実機はここを見ず毎周期の実測を始点にする。
    /// ここが要るのは、実測できる相手がいないシムと `dump` だけ。
    ///
    /// 省略時はシリアル構成なら `zero_pose_rad`、それ以外はモデルの home。
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rest_pose: Option<String>,
    /// 制御ループの周期 (Hz)。バス周期以下であること。
    #[serde(default = "default_rate_hz")]
    pub rate_hz: f64,
    /// 姿勢を切り替えるときの既定の遷移時間 (s)。
    #[serde(default = "default_transition_s")]
    pub transition_s: f64,
    /// 起動直後に取る姿勢の名前（`.misa` の `[[pose]]`）。
    ///
    /// **250×350×700 mm の直方体に収める初期姿勢はここで指す。** 実際にどの
    /// 姿勢にするかは別途詰めるので、既定はモデルに入っている畳んだ姿勢
    /// `constrain` にしてある（脚を伸ばしたまま起動するより安全側）。
    #[serde(default = "default_start_pose")]
    pub start_pose: String,
    /// 電源投入後の初回起動で、**いまの姿勢をマルチターン原点に張り直す**か。
    ///
    /// # 何のためにあるのか
    ///
    /// 角度規約は `q_model = sign * q_motor + zero_pose_rad` で、**モータ角 0 が
    /// 伏せ姿勢**と決めてある。モータのマルチターンカウンタは電源投入で 0 に
    /// なるので、本来は「伏せ姿勢でモータ電源を入れる」だけで原点が揃う。
    ///
    /// ところが SBC はモータと同じ電源から作っているので、電源投入の瞬間は
    /// SBC のブート中で、操縦者がまだロボットを置いている最中かもしれない。
    /// 原点が決まる瞬間を人が狙えない。これを有効にすると、原点が決まる瞬間が
    /// **制御ループ開始の直前・脱力が確認できている時点**に移る。
    ///
    /// # 危険と、その封じ方
    ///
    /// このコマンドは**そのときの姿勢を無条件に原点にする**。立脚中に実行すれば
    /// 立脚姿勢が伏せ扱いになり、12 軸すべての `zero_pose_rad` が無効になる。
    ///
    /// したがって**電源投入後の初回起動でしか実行しない**。目印を
    /// `/dev/shm`（tmpfs。再起動で必ず消える）に置いて判定する。試合中に
    /// クラッシュしてサービスが再起動しても、そこでは張り直さない。
    /// SBC とモータが同じ電源なので「再起動で消える」＝「モータ電源が
    /// 入り直した」と一致する。**電源系統を分けたらこの前提は崩れる。**
    #[serde(default)]
    pub zero_multiturn_on_boot: bool,
    /// 脚の運動学を自動検出するときに使う「立った姿勢」の名前。
    ///
    /// この姿勢での順運動学から公称の脚の高さが決まるので、脚を伸ばし切った
    /// 姿勢を指すと歩容の立ち位置が高くなりすぎる。
    #[serde(default = "default_kinematics_pose")]
    pub kinematics_pose: String,
    /// S.BUS が途絶えたとみなすまでの時間 (ms)。
    #[serde(default = "default_teleop_timeout_ms")]
    pub teleop_timeout_ms: u64,
    /// 胴体がこれ以上傾いたら「転倒しかけている」と報告する [rad]。`0` で無効。
    ///
    /// **報告だけで、自動では脱力しない。** 荷重がかかった四足を勝手に
    /// 脱力させると崩れるので、止めるかどうかは operator が決める（受信断や
    /// 異常ビットと同じ方針）。**接地センサが無い機体では、転倒に近づいた
    /// ことを知る手がかりが IMU の姿勢角しかない。**
    ///
    /// 測るのは鉛直からの傾き（`cos θ = cos roll · cos pitch`）で、roll と
    /// pitch の和ではない。
    ///
    /// **省略時は `gait.body_attitude_max_rad + 0.3`（最低 0.5）** を使う
    /// （[`AppConfig::max_tilt_rad`]）。固定値にしないのは、**意図して傾ける
    /// 量が機体ごとに違う**から — namiashi は 0.6 rad まで傾けるので 0.5 の
    /// 固定値だと指令どおり傾けただけで転倒扱いになり、namiashi2 は 0.20 rad なので
    /// 0.9 では倒れてから気づくことになる。
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_tilt_rad: Option<f64>,
}

fn default_model_path() -> String {
    "models/namiashi/namiashi.misa".into()
}
fn default_rate_hz() -> f64 {
    200.0
}
fn default_transition_s() -> f64 {
    1.5
}
fn default_start_pose() -> String {
    // namiashi.misa に入っている畳んだ姿勢（thigh 1.0 / calf -2.0）。
    "constrain".into()
}
fn default_kinematics_pose() -> String {
    // namiashi.misa の軽く膝を曲げた姿勢（thigh 0.3 / calf -0.6）。
    "extend".into()
}
fn default_teleop_timeout_ms() -> u64 {
    100
}


impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            model: default_model_path(),
            rest_pose: None,
            rate_hz: default_rate_hz(),
            transition_s: default_transition_s(),
            start_pose: default_start_pose(),
            // 既定は false。**姿勢を無条件に原点にする**副作用があるので、
            // 明示的に書いた設定でだけ有効になるようにしてある。
            zero_multiturn_on_boot: false,
            kinematics_pose: default_kinematics_pose(),
            teleop_timeout_ms: default_teleop_timeout_ms(),
            // 省略 = 姿勢指令の上限から導く（`AppConfig::max_tilt_rad`）。
            max_tilt_rad: None,
        }
    }
}

/// 歩容のチューニング。プロポで選ぶ 3 種（Crawl / Walk / Trot）の共通部分と、
/// 種別ごとの上書きを分けてある。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GaitTuning {
    /// 膝の曲がる向き。**機体ごとに違う。**
    ///
    /// namiashi も namiashi2 も 4 脚とも後ろ向き（thigh + / calf − で畳む）。
    ///
    /// **モデルの可動域で決まる。憶測で選ばないこと。** namiashi2 の calf は
    /// `-2.705..-0.838` で常に負なので、後脚に正の膝角を要求する `<>` は
    /// 物理的に取れない。それでも `sim` は動いてしまい、**脚が可動域に
    /// 当たって長さが変わったぶん速く前へ進んで見える**。`dump` と `sim` の
    /// 可動域検査を必ず通すこと (2026-09-01)。
    #[serde(default)]
    pub knee_pattern: KneeShape,
    /// 立ち姿勢での胴体高さ (m)。
    #[serde(default = "default_stance_height")]
    pub stance_height_m: f64,
    /// 遊脚の持ち上げ高さ (m)。
    #[serde(default = "default_swing_height")]
    pub swing_height_m: f64,
    /// 前後速度の上限 (m/s)。スティック全開でこの値。
    #[serde(default = "default_max_vx")]
    pub max_vx_m_s: f64,
    /// 左右（真横）速度の上限 (m/s)。
    #[serde(default = "default_max_vy")]
    pub max_vy_m_s: f64,
    /// 旋回速度の上限 (rad/s)。
    #[serde(default = "default_max_wz")]
    pub max_wz_rad_s: f64,
    /// プロポで胴体高さを動かせる幅 (m)。`stance_height_m ± この値`。
    #[serde(default = "default_height_range")]
    pub height_range_m: f64,
    /// Crawl を `LinearCrawl`（胴体を +X 直線に載せる専用プランナ）で走らせる。
    ///
    /// **これを true にすると横移動と旋回の指令が効かなくなる**（LinearCrawl は
    /// 前進しか扱わない。実測で横・旋回とも 0）。
    ///
    /// **ただし CHAMP の crawl は静的歩容になっていない。** MuJoCo で
    /// 接地パターンを数えると（namiashi、前進 0.05、20 秒）:
    ///
    /// | | 全脚接地 | 1 本遊脚 | **2 本以上浮く** | roll 振幅 | 進む量 |
    /// |---|---|---|---|---|---|
    /// | CHAMP（既定） | 41% | 35% | **24%** | 9.9° | 64% |
    /// | LinearCrawl | 88% | 12% | **0%** | 2.4° | 101% |
    /// | 理論（接地比 0.85） | 40% | 60% | 0% | — | 100% |
    ///
    /// 全脚接地の割合は理論どおりなので**計画は正しい**。CHAMP の crawl は
    /// 重心を支持三角形へ寄せないので、1 本上げるたびに胴体が傾き、
    /// 浮くべきでない足まで離れて残りが滑る（追従誤差は 0.016 rad しかなく、
    /// **ゲインを上げると悪化する** — 24% → 68%）。
    ///
    /// つまり **crawl は「横・旋回が効くが揺れる」か「直進だけで正しい」の
    /// どちらかを選ぶことになる。** trot は roll 1.4° で問題ない。
    #[serde(default)]
    pub crawl_use_linear: bool,
    /// 歩容コントローラの種別。**既定の `auto` は `crawl_use_linear` の
    /// 従来どおりの解釈**なので、書かなければ挙動は変わらない。
    #[serde(default)]
    pub controller: GaitControllerKind,
    /// MPC の予測ホライズン（段数）。`horizon_steps * mpc_dt_per_step` が
    /// 予測窓で、Di Carlo 2018 は 300 ms 前後を使う。
    #[serde(default = "default_mpc_horizon_steps")]
    pub mpc_horizon_steps: usize,
    /// MPC の 1 段の時間 [s]。**制御周期ではない**（MPC は制御周期より
    /// 粗い刻みで先を見る）。
    #[serde(default = "default_mpc_dt_per_step")]
    pub mpc_dt_per_step: f64,
    /// MPC に**推定した胴体の高さと姿勢**を観測として渡すか。
    ///
    /// quadruped-gait の SRBD MPC は既定で「z = 公称立ち高さ、roll = pitch = 0」
    /// を現在状態に置き、参照もそこから作るので、高さと姿勢の誤差が**構造的に
    /// 見えない**（MuJoCo の trot でトルク出力が 0.06 m まで沈んでも MPC は
    /// 0.20 m に居るつもりだった）。legged_control は推定器の全状態を毎 tick
    /// MPC に渡す（`setCurrentObservation`）。これを true にすると脚
    /// オドメトリの高さと IMU の roll/pitch が MPC の現在状態に入り、参照は
    /// 公称高さ・水平のままなので誤差として効く。
    #[serde(default = "default_true")]
    pub mpc_observe_pose: bool,
    /// MPC の現在状態の**並進速度**に推定値を入れるか（既定 true）。
    ///
    /// false にすると、MPC は「胴体は指令どおりの速さで動いている」と
    /// 見なす（ランプ後の速度指令を世界向きに回して入れる）。速度の閉ループ
    /// を切る形。
    ///
    /// # なぜ切る選択肢が要るか
    ///
    /// 脚オドメトリは「接地足は世界に対して止まっている」と仮定するので、
    /// **足が進行方向へ流れていると胴体を遅く読む**。MuJoCo の trot 0.80 で
    /// 真値 0.91 m/s のところを 0.63 m/s と読んでいた（`sim` の「歩容中の
    /// 前後距離 真値 / 推定器の積分」）。MPC がその偽の不足ぶんを埋めようと
    /// 押すので、トルク出力は指令より 18 % 速く走る。位置出力では MPC の力は
    /// 位置サーボの上の前置でしかないので同じ偽の不足があっても速さは
    /// 運動学で決まる（追従率 1.02）。LKF（`estimator = "kalman"`）も足の
    /// 運動学を信じる重みなので同じ癖を持つ。
    #[serde(default = "default_true")]
    pub mpc_observe_velocity: bool,
    /// 推定器（脚オドメトリ / LKF）の立脚に**実測の接地**を使うか。
    ///
    /// 計画の立脚フラグは着地・離地の周期で実際とずれる（trot 0.80 で
    /// 各足 3〜10 % の周期）。浮いている足を「止まっている」と信じると、
    /// その足の速度がそのまま胴体速度の誤差になるので、接地を測れる Plant
    /// では実測を使う。測れない足（実機）は計画のまま。WBC の接地拘束には
    /// 影響しない（そちらは `[wbc] use_measured_contact`）。
    #[serde(default)]
    pub estimator_use_measured_contact: bool,
    /// 胴体の状態（高さ・速度）をどう推定するか。
    ///
    /// - `leg_odometry`（既定）: 接地足の運動学だけ。状態を持たず毎周期の
    ///   観測で決まる。IMU は姿勢と角速度しか使わない。
    /// - `kalman`: legged_control の 18 状態 LKF
    ///   （`legged_estimation::LinearKalmanEstimator`）。IMU の加速度で予測し、
    ///   足の運動学で補正する。接地していない足は共分散を 100 倍にして
    ///   ほとんど見ない。**加速度計が要る** — MuJoCo は胴体速度の差分で
    ///   作る。計画に対する位置誤差は変わらず脚オドメトリで出す。
    #[serde(default)]
    pub estimator: EstimatorKind,
    /// 接地点の捕捉点フィードバックのゲイン [s]。`0` で無効。
    ///
    /// **硬い PD（kp ≥ 100 / kv ≤ 1.2）では正帰還になることが
    /// `quadruped_gait` 側で報告されている**（追従の雑音を増幅する）。
    /// 既定は quadruped-gait の既定値と同じ 0.05。切り分けるときは 0 に。
    #[serde(default = "default_mpc_capture_point_gain_s")]
    pub mpc_capture_point_gain_s: f64,
    /// 速度指令を 0 から最大まで振り切るのにかける時間 [s]。0 でランプ無し。
    ///
    /// **歩容はスティックが動いた瞬間に出力を階段状に飛ばす。** 実測で
    /// 制御 1 周期あたり Crawl 31.5 / Walk 23.0 / Trot 9.9 rad/s
    /// （Crawl は 5 ms で 9.0°）。2 tick 目以降は滑らかなので跳ぶのは
    /// 切り替わりの 1 点だけで、スティック側を鈍らせれば消える
    /// （0.5 s のランプで Crawl 31.5 → 3.34 rad/s）。
    #[serde(default = "default_velocity_ramp_s")]
    pub velocity_ramp_s: f64,
    /// 速度指令を**落とす**ときのランプ時間 [s]。0 でランプ無し。
    ///
    /// **上げる側 (`velocity_ramp_s`) と別にしてある。止まるのは速い方が
    /// よい。** スティックを中立に戻してから実際に止まるまでが長いと、
    /// リング外へ出る。上げる側は滑らかさのために鈍らせてよいが、
    /// 下げる側を同じ時間にする理由は無い。
    #[serde(default = "default_velocity_ramp_stop_s")]
    pub velocity_ramp_stop_s: f64,
    /// 速度をちょうど 0 にする前に、全脚接地を待つ上限 [s]。0 で待たない。
    ///
    /// 歩容は `v = 0` で静止姿勢へ分岐し、遊脚を一気に接地させる
    /// （実測 35.4 rad/s = 制御 1 周期で 10.2°）。全脚が接地した瞬間に
    /// 0 へ落とせば跳ばない。
    ///
    /// **ただし trot では 4 脚が同時に接地しないことがあり、その場合は
    /// 毎回この時間だけ待ち切る。** 長くすると「スティックを戻しても
    /// 止まらない」になるので、**遊脚 1 回ぶんで足りる長さにする**。
    /// かつて 2.0 s 固定だったため、停止に最大 2.5 s かかっていた。
    #[serde(default = "default_stop_settle_s")]
    pub stop_settle_s: f64,
    /// 胴体姿勢をプロポで傾けられる上限 (rad)。**0 で機能ごと無効**。
    ///
    /// CH8 が ON のとき、CH1 がロール、CH3 がピッチになる（OFF のときは
    /// CH1 = 横移動、CH3 = 高さのまま）。歩容が出した足先位置を回してから
    /// IK を解き直すので、**足は接地したまま胴体だけ傾く**。
    ///
    /// **傾けると脚の可動域を食う。** `start` ポーズの calf は可動域まで
    /// 10°、`constrain_2` は 5.55° しかない。大きくすると IK が届かない脚が
    /// 出る（クランプされて姿勢が崩れる）。`dump` で範囲内に収まる角度を
    /// 確かめてから上げること。
    #[serde(default)]
    pub body_attitude_max_rad: f64,
    /// 胴体姿勢の追従時定数 (s)。CH8 を切り替えた瞬間に胴体が跳ねないよう
    /// 一次遅れを入れる。0 で素通し。
    #[serde(default = "default_body_attitude_tau_s")]
    pub body_attitude_tau_s: f64,
    /// 歩容種別ごとの周期 (s)。指定が無ければ `quadruped-gait` のプリセット値。
    #[serde(default)]
    pub crawl_cycle_s: Option<f64>,
    #[serde(default)]
    pub walk_cycle_s: Option<f64>,
    #[serde(default)]
    pub trot_cycle_s: Option<f64>,
    /// 1 歩の最大歩幅 [m]。指定が無ければ `quadruped-gait` のプリセット値
    /// （crawl 0.06 / walk 0.08 / trot 0.10）。
    ///
    /// # ここが速度の上限を決める
    ///
    /// 歩容が出せる速度は `歩幅 / (周期 × 接地比)` で頭打ちになる。
    /// プリセットのままだと、この機体（脚長 0.306 m）では
    ///
    /// | | 歩幅 | 周期 | 接地比 | 上限 |
    /// |---|---|---|---|---|
    /// | crawl | 0.06 | 1.667 | 0.85 | **0.042 m/s** |
    /// | walk | 0.08 | 0.600 | 0.75 | 0.178 m/s |
    /// | trot | 0.10 | 0.400 | 0.50 | 0.500 m/s |
    ///
    /// **crawl は 0.042 m/s しか出ない。** プロファイルの `max_vx_m_s`
    /// (0.15) を指令しても届かず、追従率だけが落ちる。それはコントローラの
    /// 失敗ではなく算術で、**歩幅を上げないと直らない**（articara が
    /// namiashi の WBC を詰めたとき、いちばん効いたのがこれ）。
    ///
    /// articara の詰めた値は 3 歩容とも **0.145 m**（脚長の 47 %）。
    /// **上げると遊脚の擦りが出る**ので `swing_height_m` も一緒に上げること
    /// （crawl は 0.005 → 0.040 で追従率 74 % → 104 %）。
    #[serde(default)]
    pub step_length_m: Option<f64>,
}

fn default_stance_height() -> f64 {
    // 脚は thigh 0.1528 + calf 0.1528 = 0.306 m。膝を曲げた常用姿勢としての初期値。
    0.20
}
fn default_swing_height() -> f64 {
    0.03
}
fn default_max_vx() -> f64 {
    0.15
}
fn default_max_vy() -> f64 {
    0.08
}
fn default_max_wz() -> f64 {
    0.6
}
/// quadruped-gait の `SrbdMpcConfig::default()` と同じ。
fn default_true() -> bool {
    true
}

fn default_mpc_horizon_steps() -> usize {
    10
}

fn default_mpc_dt_per_step() -> f64 {
    0.030
}

/// `quadruped_gait::mpc_controller::DEFAULT_CAPTURE_POINT_GAIN_S`。
fn default_mpc_capture_point_gain_s() -> f64 {
    0.05
}

fn default_velocity_ramp_s() -> f64 {
    0.5
}

fn default_velocity_ramp_stop_s() -> f64 {
    0.15
}

fn default_stop_settle_s() -> f64 {
    0.25
}

fn default_body_attitude_tau_s() -> f64 {
    0.15
}

fn default_height_range() -> f64 {
    0.04
}

impl Default for GaitTuning {
    fn default() -> Self {
        Self {
            knee_pattern: KneeShape::default(),
            stance_height_m: default_stance_height(),
            swing_height_m: default_swing_height(),
            max_vx_m_s: default_max_vx(),
            max_vy_m_s: default_max_vy(),
            max_wz_rad_s: default_max_wz(),
            height_range_m: default_height_range(),
            crawl_use_linear: false,
            controller: GaitControllerKind::default(),
            mpc_horizon_steps: default_mpc_horizon_steps(),
            mpc_dt_per_step: default_mpc_dt_per_step(),
            mpc_capture_point_gain_s: default_mpc_capture_point_gain_s(),
            mpc_observe_pose: true,
            mpc_observe_velocity: true,
            estimator_use_measured_contact: false,
            estimator: EstimatorKind::LegOdometry,
            velocity_ramp_s: default_velocity_ramp_s(),
            velocity_ramp_stop_s: default_velocity_ramp_stop_s(),
            stop_settle_s: default_stop_settle_s(),
            // **既定は無効。** 制御ループの出力に手が入る機能なので、
            // 設定で明示的に上げるまで従来と 1 ビットも変わらない出力を出す。
            body_attitude_max_rad: 0.0,
            body_attitude_tau_s: default_body_attitude_tau_s(),
            crawl_cycle_s: None,
            walk_cycle_s: None,
            trot_cycle_s: None,
            step_length_m: None,
        }
    }
}

/// ポーズ再生の設定。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PoseConfig {
    /// プロポのポーズ再生スイッチで走らせるもの。`.misa` の
    /// `[[sequence]]` 名、または `[[pose]]` 名。
    #[serde(default = "default_greeting")]
    pub greeting: String,
    /// **CH8 を押しながら CH7** で再生するもの。空なら [`Self::greeting`]。
    ///
    /// 振る足を現場で選べるようにするためのもの。空きチャンネルが無いので、
    /// 既にある CH8（姿勢モード）を修飾キーとして使う。
    #[serde(default = "default_greeting_alt")]
    pub greeting_alt: String,
    /// チキンヘッドの基準角 (rad)。胴体ピッチ 0 のときの腕角。
    #[serde(default)]
    pub chicken_head_base_rad: f64,
    /// チキンヘッドの補償ゲイン。1.0 で胴体ピッチを完全に打ち消す。
    #[serde(default = "default_chicken_gain")]
    pub chicken_head_gain: f64,
    /// チキンヘッドの一次遅れ時定数 (s)。IMU のノイズをサーボへ通さないため。
    #[serde(default = "default_chicken_tau")]
    pub chicken_head_tau_s: f64,
}

fn default_greeting() -> String {
    "greeting".into()
}

fn default_greeting_alt() -> String {
    String::new()
}
fn default_chicken_gain() -> f64 {
    1.0
}
fn default_chicken_tau() -> f64 {
    0.05
}

impl Default for PoseConfig {
    fn default() -> Self {
        Self {
            greeting: default_greeting(),
            greeting_alt: default_greeting_alt(),
            chicken_head_base_rad: 0.0,
            chicken_head_gain: default_chicken_gain(),
            chicken_head_tau_s: default_chicken_tau(),
        }
    }
}

#[cfg(test)]
mod tests {

    /// **モデルファイルを `--config` に渡せてしまってはいけない。**
    ///
    /// 全フィールドに serde の既定値があるので、放っておくと TOML なら
    /// 何でも通る。実際 `.misa` が「設定: OK」と出て、ゼロ点 0 / 符号 +1 の
    /// **未校正の既定値**で実機を動かせる状態だった (2026-08-22)。
    #[test]
    fn a_model_file_is_not_accepted_as_a_config() {
        let misa = r#"
[[pose]]
name = "start"

[pose.angles]
FL_hip_joint = 0.0
"#;
        let e = AppConfig::from_toml(misa).unwrap_err();
        assert!(e.contains("[hardware]"), "{e}");
        assert!(e.contains("モデルファイル"), "{e}");
    }

    #[test]
    fn an_empty_toml_is_not_accepted_as_a_config() {
        // ファイルを指定した以上、中身が設定であることを求める。
        // 既定値で走らせたいなら --config を付けない。
        assert!(AppConfig::from_toml("").is_err());
    }

    /// **名前は実行時のパスに入る。**
    ///
    /// 脚バスの flock (`/run/lock/misa-legs-<name>.lock`) と、マルチターン
    /// 原点の目印 (`/dev/shm/misa-multiturn-zeroed-<name>`) に埋め込まれる。
    /// `/` や `..` を通すと、プロファイル 1 枚で別のディレクトリのファイルを
    /// 掴みに行けてしまう。
    #[test]
    fn a_robot_name_that_would_escape_its_path_is_rejected() {
        for bad in ["../../etc/passwd", "a/b", "", "name with space", "名前"] {
            let mut cfg = AppConfig::default();
            cfg.name = bad.to_string();
            assert!(
                cfg.validate().is_err(),
                "name {bad:?} が通ってしまった"
            );
        }
    }

    #[test]
    fn the_shipped_profile_names_the_robot() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../robots/namiashi.toml");
        let text = std::fs::read_to_string(path).unwrap();
        let cfg = AppConfig::from_toml(&text).unwrap();
        assert_eq!(cfg.name, "namiashi");
    }

    /// **同梱プロファイルのモデルパスが、組み込みの既定と揃っていること。**
    ///
    /// submodule を models/namiashi/ へ移したとき、既定値だけ直して
    /// プロファイル本体を直し忘れ、`dump` が「読み込みに失敗」で落ちた。
    /// 2 か所にある以上、揃っていることを試験で押さえる。
    #[test]
    fn the_shipped_profile_points_at_the_same_model_as_the_default() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../robots/namiashi.toml");
        let text = std::fs::read_to_string(path).unwrap();
        let cfg = AppConfig::from_toml(&text).unwrap();
        assert_eq!(cfg.control.model, default_model_path());
    }

    /// **ブリッジ越しのプロファイルが読めること。**
    ///
    /// この形の機体は配線もモータ id も校正値も持たない。「[hardware] に
    /// モータが無い設定は設定として認めない」という検証が効いたままだと、
    /// ここで弾かれる。
    ///
    /// **実機のプロファイルは機体側のリポジトリにあるので、ここでは書式だけ
    /// を見る。** 見ているのは設定の読み書きであって、特定の機体の値ではない。
    #[test]
    fn a_profile_for_a_robot_behind_a_bridge_loads() {
        let text = r#"
name = "bridged"
[control]
model = "models/namiashi/namiashi.misa"
[hardware]
kind = "ros2"
namespace = "/bridged"
[[aux]]
joint = "FL_wheel_joint"
role = "wheel"
"#;
        let cfg = AppConfig::from_toml(text).expect("ブリッジ越しのプロファイルが読めない");
        assert_eq!(cfg.name, "bridged");
        assert_eq!(cfg.aux.len(), 1);
        assert!(
            cfg.hardware.serial().is_err(),
            "ブリッジ越しの構成が serial として読めてしまっている"
        );
        // 制御周期の上限はバスの周期ではなく向こうが決める。
        assert_eq!(cfg.hardware.max_control_rate_hz(), None);
        // MIT ゲインは既定が入る（0 だと脱力したまま崩れるので）。
        let g = cfg.hardware.mit_gains().expect("ブリッジ越しならゲインを持つ");
        assert!(g.validate().is_ok());
    }

    /// **傾きのしきい値は機体ごとに違う。** 固定値にすると、意図して
    /// 大きく傾ける機体（0.6 rad）で指令どおりの姿勢が転倒扱いになり、
    /// あまり傾けない機体（0.20 rad）では倒れてから気づく。
    /// 姿勢指令の上限（と、明示的な傾きのしきい値）だけを差し替えた設定。
    fn tilt_cfg(attitude_max_rad: f64, max_tilt_rad: Option<f64>) -> AppConfig {
        AppConfig {
            control: ControlConfig {
                max_tilt_rad,
                ..Default::default()
            },
            gait: GaitTuning {
                body_attitude_max_rad: attitude_max_rad,
                ..Default::default()
            },
            ..Default::default()
        }
    }

    #[test]
    fn the_tilt_limit_is_derived_from_how_far_the_body_is_tilted_on_purpose() {
        assert!((tilt_cfg(0.20, None).max_tilt_rad() - 0.50).abs() < 1e-12, "あまり傾けない機体");
        assert!((tilt_cfg(0.6, None).max_tilt_rad() - 0.9).abs() < 1e-12, "大きく傾ける機体");
        // 姿勢を振らない機体でも 0.5 rad を下回らない。
        assert!((tilt_cfg(0.0, None).max_tilt_rad() - 0.5).abs() < 1e-12);
        // 導出値は必ず姿勢指令の上限より外側。
        for max in [0.0, 0.1, 0.2, 0.6, 1.0] {
            assert!(tilt_cfg(max, None).max_tilt_rad() > max, "{max}");
        }
    }

    /// 明示的に書いた値が姿勢指令の上限以下なら、設定として認めない。
    #[test]
    fn a_tilt_limit_inside_the_attitude_command_is_rejected() {
        let e = tilt_cfg(0.6, Some(0.5)).validate().expect_err("弾かれていない");
        assert!(e.contains("max_tilt_rad"), "{e}");
        // 0 は「無効にする」なので通る。
        let off = tilt_cfg(0.6, Some(0.0));
        off.validate().expect("0 は無効化として認める");
        assert_eq!(off.max_tilt_rad(), 0.0);
    }

    /// **同梱のプロファイルはどれも、導出値が姿勢指令の外側にある。**
    ///
    /// 機体ごとのプロファイルは別リポジトリにもあるので、ここで見るのは
    /// `robots/` に入っているものだけ。あちらは あちらで同じ試験を持つ。
    #[test]
    fn every_shipped_profile_reports_a_tilt_beyond_its_attitude_command() {
        let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../../robots");
        let mut seen = 0;
        for entry in std::fs::read_dir(dir).expect("robots/ が読めません") {
            let path = entry.unwrap().path();
            if path.extension().and_then(|e| e.to_str()) != Some("toml") {
                continue;
            }
            seen += 1;
            let name = path.file_stem().unwrap().to_string_lossy().to_string();
            let text = std::fs::read_to_string(&path).expect("プロファイルが読めません");
            let cfg = AppConfig::from_toml(&text).expect("プロファイルが読めない");
            assert!(
                cfg.max_tilt_rad() > cfg.gait.body_attitude_max_rad,
                "{name}: 傾きの報告 {} が姿勢指令の上限 {} の内側",
                cfg.max_tilt_rad(),
                cfg.gait.body_attitude_max_rad
            );
        }
        assert!(seen > 0, "robots/ にプロファイルが 1 枚も無い");
    }

    #[test]
    fn the_shipped_config_still_loads() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../robots/namiashi.toml");
        let text = std::fs::read_to_string(path).expect("robots/namiashi.toml が読めません");
        AppConfig::from_toml(&text).expect("同梱の設定が読めなくなっている");
    }
    use super::*;

    #[test]
    fn default_config_is_valid() {
        AppConfig::default().validate().unwrap();
    }

    #[test]
    fn default_config_round_trips_through_toml() {
        let cfg = AppConfig::default();
        let back = AppConfig::from_toml(&cfg.to_toml().unwrap()).unwrap();
        assert_eq!(cfg, back);
    }

    /// 設定ファイルを書かずに起動できること。
    ///
    /// **`from_toml("")` ではない。** `--config` を付けない経路は
    /// `AppConfig::default()` を直接使う（`main::load_config`）ので、
    /// 空 TOML が通るかどうかとは無関係。かつてこのテストは
    /// `from_toml("")` を見ており、**「TOML なら何でも設定として通る」
    /// という穴の方を守っていた**。
    #[test]
    fn the_defaults_are_usable_without_a_config_file() {
        AppConfig::default().validate().unwrap();
    }

    #[test]
    fn control_rate_above_the_bus_rate_is_rejected() {
        let mut cfg = AppConfig::default();
        cfg.control.rate_hz = cfg.hardware.max_control_rate_hz().unwrap() * 2.0;
        assert!(cfg.validate().is_err());
    }
}
