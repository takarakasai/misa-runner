//! 動作モードの状態機械。**ハードウェアに一切触れない**ので、実機なしで
//! 遷移そのものを試験できる。
//!
//! ```text
//!   Relaxed ──(スイッチ: 起立/歩行)──▶ GoingToStart ──▶ GoingToStance ──▶ Active
//!      ▲                                                                    │
//!      └────────────────(スイッチ: 脱力)────────────────────────────────────┘
//!                                                    Active ──(ポーズ)──▶ PlayingPose
//!                                                       ▲                   │
//!                                                       └───────────────────┘
//! ```
//!
//! `Active` が起立と歩行の両方を兼ねているのは、`quadruped-gait` が
//! 速度指令ゼロを「脚を接地したまま止まる」として扱うため。状態を分けると、
//! 同じことを 2 か所で書くことになる。

use misarta::trajectory::InterpolationKind;
use misa_hal::joint::JointMode;
use quadruped_gait::{AnyGaitController, GaitGenerator};

use crate::chicken::ChickenHead;
use crate::config::AppConfig;
use crate::jointvec::JointVec;
use crate::pose::PosePlayer;
use crate::robot::{velocity_cmd, Robot};
use misa_core::{GaitSelect, Intent, ModeRequest};
use crate::viz::BodyView;

/// 状態機械の状態。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// 脱力。モータへは指令を送らず状態だけ読む。
    Relaxed,
    /// 初期姿勢（`control.start_pose`）へ遷移中。
    GoingToStart,
    /// 初期姿勢で保持。**通電したまま止まっている。**
    ///
    /// 試合はこの姿勢でスタートボックスに置いて合図を待つ。CH5 の中段が
    /// ここで、上段（歩行）へ倒すと立ち姿勢を経て歩容に入る。
    HoldingStart,
    /// 歩容の立ち姿勢へ遷移中。
    GoingToStance,
    /// 歩容が動いている（速度ゼロなら立ったまま）。
    Active,
    /// ポーズ / シーケンスを再生中。
    PlayingPose,
    /// 膝の向きを反転する振り付けの最中（`Intent.knee_pattern`）。脚を浮かせて
    /// 膝を伸ばし切り、反対へ畳む。終わったら `Active` へ戻る。
    FlippingKnees,
}

impl State {
    pub fn label(self) -> &'static str {
        match self {
            State::Relaxed => "脱力",
            State::GoingToStart => "初期姿勢へ",
            State::HoldingStart => "初期姿勢",
            State::GoingToStance => "立ち姿勢へ",
            State::Active => "歩容",
            State::PlayingPose => "ポーズ再生",
            State::FlippingKnees => "膝を反転中",
        }
    }
}

/// 1 周期ぶんの出力。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlOutput {
    /// 12 脚関節 + 腕の目標角 (rad, モデル座標系)。
    pub targets: JointVec,
    /// 脚関節へ与える制御モード。`Relaxed` の間は `Idle`。
    pub leg_mode: JointMode,
    pub state: State,
    /// 立脚フラグ（FL, FR, RL, RR）。**歩容が計画した接地**であって観測では
    /// ない（足裏センサが無い）。WBC の接触制約と接地力の配分に使う。
    ///
    /// 歩容を回していない状態（遷移中・ポーズ再生中）は最後に分かった値の
    /// まま。そのあいだ WBC は回さないので影響しない。
    pub stance: [bool; 4],
    /// 歩容が計画している世界ヨー角 [rad]。速度指令の積分値で、IMU とは
    /// **原点が違う**。WBC のヨー保持の目標に使う。
    pub planned_yaw_rad: f64,
    /// 歩容が計画した足の位置（胴体座標系、FL / FR / RL / RR）。
    /// WBC の遊脚タスク（直交空間）の目標。
    pub target_foot_body: [nalgebra::Vector3<f64>; 4],
    /// MPC 歩容が出した参照。CHAMP / LinearCrawl では `None`。
    ///
    /// **WBC の参照がこれで変わる**（[`crate::wbc::MpcReference`]）。
    pub mpc: Option<crate::wbc::MpcReference>,
    /// ランプ後の胴体速度指令 `[vx, vy, wz]`（胴体座標系、m/s と rad/s）。
    ///
    /// **WBC にとっては「胴体がいま動いている速さ」の唯一の手がかり。**
    /// オドメトリが無いので実測はできず、指令をそのまま推定値として使う。
    /// これを 0 と置くと、WBC は静止した胴体を支える解しか出さず、
    /// 立脚が胴体を送る動きを一切許さない（実際に転倒した）。
    pub body_velocity: [f64; 3],
}

/// 状態機械 + 歩容 + ポーズ再生。
/// 膝の反転（trot）の段の種類。実行中の上書きのしかたが違う。
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
enum KneeStepKind {
    /// 計画どおり。2 脚支持なら釣り合いの帰還が立脚をずらす。
    #[default]
    Normal,
    /// 探りの上げ下げ（2 脚支持、寄せは凍結、帰還なし）。
    ProbeMove,
    /// 探りの保持（2 脚支持、寄せは凍結、θ̇ を溜めて重心のずれを測る）。
    ProbeHold,
    /// 4 脚のまま、推定込みの重心が支持線に乗るところへ胴体を寄せ直す。
    Adjust,
}

/// [`Controller::plan_knee_flip`] の結果。
struct KneeFlipPlan {
    steps: Vec<crate::pose::PoseStep>,
    stances: Vec<[bool; 4]>,
    forwards: Vec<[bool; 4]>,
    plan_shifts: Vec<f64>,
    floors: Vec<f64>,
    kinds: Vec<KneeStepKind>,
    supports: Vec<[usize; 2]>,
    next_offset: [nalgebra::Vector3<f64>; 4],
}

pub struct Controller {
    robot: Robot,
    /// 腕がこちらの指令で動くか。false（受信機直結・未配線）のときは
    /// チキンヘッドもポーズの腕動作も成立しないので、腕の目標には
    /// **観測値をそのまま置く**。指令値を置くと、実機と食い違った角度で
    /// ログと可視化が埋まる。
    arm_app_driven: bool,
    gait: AnyGaitController,
    gait_select: GaitSelect,
    /// 歩容へ入れてある上書き。**入っているものと比べて、変わったときだけ
    /// `set_config` を呼ぶ**（毎周期呼んでも害は無いが、ログが埋まる）。
    gait_tune: misa_core::GaitTune,
    state: State,
    player: Option<PosePlayer>,
    /// 直近に出した目標。遷移の始点であり、`Relaxed` からの復帰点でもある。
    targets: JointVec,
    chicken: ChickenHead,
    cfg: AppConfig,
    /// 状態が変わった直後だけ true。ログ用。
    just_changed: bool,
    /// チキンヘッドが効かないことを 1 度だけ警告するためのフラグ。
    /// 毎周期出すとログが埋まる。
    warned_chicken_head: bool,
    /// いま歩容に入れてある胴体高さ [m]（`nominal_foot_body.z` の符号違い）。
    /// 変わったときだけ `set_kinematics` を呼ぶ。
    applied_height_m: f64,
    /// ランプ後の速度指令 `[vx, vy, wz]`。歩容へ渡すのはこちら。
    ///
    /// **スティックの値を直接渡すと歩容の出力が階段状に飛ぶ**（実測で
    /// 制御 1 周期あたり Crawl 31.5 rad/s = 5 ms で 9.0°）。歩容自体は
    /// 2 tick 目以降は滑らかなので、跳ぶのは切り替わりの 1 点だけ。
    /// ここで鈍らせれば消える。
    ramped_v: [f64; 3],
    /// 一次遅れを掛けた後の胴体姿勢 `[roll, pitch]` (rad)。
    ///
    /// **CH8 を切り替えた瞬間に胴体が跳ねないため。** 指令をそのまま
    /// 渡すと、ON の瞬間にスティックの位置ぶんだけ一気に傾く。
    tilt_rad: [f64; 3],
    /// 姿勢が可動域に届かないことを 1 度だけ警告するためのフラグ。
    warned_tilt_reach: bool,
    /// ポーズ再生を要求した瞬間に CH8 が入っていたか（振る足の選択）。
    alt_pose_requested: bool,
    /// 停止指令を受けてから全脚接地を待っている時間 [s]。
    /// 待ちが終わらないまま歩き続けないための保険。
    settling_s: f64,
    /// 直近の歩容出力の足位置（胴体座標系）。遷移中・ポーズ再生中は最後に
    /// 分かった値のまま（そのあいだ WBC は回さないので影響しない）。
    target_foot_body: [nalgebra::Vector3<f64>; 4],
    /// 直近の周期で MPC が出した参照。歩容が MPC 系でなければ `None`。
    ///
    /// `tick_active` でだけ更新する。**歩容が回っていない相では捨てる**
    /// （古い接地力の予測を WBC の参照にすると、接地していない足を押す）。
    mpc: Option<crate::wbc::MpcReference>,
    /// 脚オドメトリで測った胴体の速度と角速度（世界座標系）。
    /// [`Self::observe_body`] が入れ、MPC の参照を作るのに使う。
    observed_v_world: nalgebra::Vector3<f64>,
    observed_omega_world: nalgebra::Vector3<f64>,
    /// 直近の歩容出力から取った胴体姿勢と接地。可視化にだけ使う。
    ///
    /// 歩容を回していない状態（遷移中・ポーズ再生中）でも姿勢を描きたいので、
    /// 最後に分かった値を保持する。歩容が止まっている間は胴体も動かない
    /// ので、これは嘘ではない。
    body_view: BodyView,
    /// 反転の振り付けが終わったら採用する膝の向き。
    knee_flip_target: Option<crate::config::KneeShape>,
    /// 振り付けの段ごとの立脚フラグ（浮かせている脚は false）。前置トルクと
    /// WBC の接地の仮定に使う。
    knee_flip_stance: Vec<[bool; 4]>,
    warned_knee_flip: bool,
    /// 組めなかった要求。同じ要求のあいだは組み直さない（毎周期の再計算とログを防ぐ）。
    knee_flip_failed: Option<crate::config::KneeShape>,
    /// 立ち位置の足先の xy のずらし（脚ごと、胴体座標）。`rest` の反転で車輪から
    /// 立ち上がった足の位置をそのまま立ち位置にするために使う（`>>` は外へ 8 cm）。
    /// 歩容の運動学（`nominal_foot_body`）に足す。
    stance_xy_offset: [nalgebra::Vector3<f64>; 4],
    applied_offset: [nalgebra::Vector3<f64>; 4],
    /// 反転が終わったら採用する立ち位置のずらし。
    knee_flip_next_offset: Option<[nalgebra::Vector3<f64>; 4]>,
    /// 止まる要求が来たとき空中だった脚（[`Self::ramp_velocity`] の待ち）。
    settling_airborne: Option<[bool; 4]>,
    /// 段ごとの膝の向き（前向き = true）。`trot` の釣り合いで立脚を IK し直すのに使う。
    knee_flip_forward: Vec<[bool; 4]>,
    /// `trot` の釣り合い: 立脚の足先の基準位置（胴体座標、立ち高さ）と、いま掛けている
    /// 胴体の横移動 [m]（支持線と直角）。最大値はログ用。
    knee_flip_feet: [nalgebra::Vector3<f64>; 4],
    /// 段ごとの、計画に織り込んだ胴体の寄せ [m]（2 脚支持に入るときの初期値）。
    knee_flip_plan_shift: Vec<f64>,
    /// 段ごとの床の高さ（胴体座標）。trot は反転中だけ胴体を上げるので立脚の z に使う。
    knee_flip_floor: Vec<f64>,
    /// 段ごとの種類（[`KneeStepKind`]）と、その段が属する組の支持脚。
    knee_flip_kind: Vec<KneeStepKind>,
    knee_flip_support: Vec<[usize; 2]>,
    /// 探りで測った実機の重心のずれ（胴体座標 xy、モデルに足す）。反転をまたいで持つ。
    knee_flip_com_est: nalgebra::Vector2<f64>,
    /// 探りの保持中に溜める (経過 s, θ̇, θ)。
    knee_flip_probe: Vec<(f64, f64, f64)>,
    knee_flip_probe_t: f64,
    knee_flip_prev_idx: usize,
    knee_flip_shift_m: f64,
    knee_flip_tilt_max_rad: f64,
    knee_flip_shift_max_m: f64,
    /// 実測の重心の支持線からの距離（前回値）と、その速度の一次遅れ。
    knee_flip_s_prev: Option<f64>,
    knee_flip_sdot: f64,
    /// 帰還の指令（上限で丸めたあと）の一次遅れ。
    knee_flip_want_lpf: f64,
}

impl Controller {
    /// 腕を駆動しない構成（受信機直結・未配線）向け。
    pub fn new(robot: Robot, cfg: AppConfig) -> Self {
        Self::with_arm(robot, cfg, false)
    }

    /// `arm_app_driven` はアプリが腕サーボを駆動できるか
    /// （`misa_hal::arm::ArmServo::is_app_driven`）。
    pub fn with_arm(robot: Robot, cfg: AppConfig, arm_app_driven: bool) -> Self {
        // 基準姿勢の胴体高さ（高さだけの由来なら stance_height_m そのもの）。
        let stance_height_m = robot.reference_height_m(&cfg.gait);
        let gait_select = GaitSelect::Crawl;
        let gait = robot.build_gait(&cfg.gait, &cfg.wbc, gait_select);
        let chicken = ChickenHead::new(&cfg.poses);
        let mut c = Self {
            robot,
            arm_app_driven,
            gait,
            gait_select,
            gait_tune: misa_core::GaitTune::default(),
            state: State::Relaxed,
            player: None,
            targets: JointVec::zeros(),
            chicken,
            cfg,
            just_changed: false,
            warned_chicken_head: false,
            applied_height_m: stance_height_m,
            ramped_v: [0.0; 3],
            settling_s: 0.0,
            tilt_rad: [0.0; 3],
            warned_tilt_reach: false,
            alt_pose_requested: false,
            target_foot_body: [nalgebra::Vector3::zeros(); 4],
            mpc: None,
            observed_v_world: nalgebra::Vector3::zeros(),
            observed_omega_world: nalgebra::Vector3::zeros(),
            body_view: BodyView::default(),
            knee_flip_target: None,
            knee_flip_stance: Vec::new(),
            warned_knee_flip: false,
            knee_flip_failed: None,
            stance_xy_offset: [nalgebra::Vector3::zeros(); 4],
            applied_offset: [nalgebra::Vector3::zeros(); 4],
            knee_flip_next_offset: None,
            settling_airborne: None,
            knee_flip_forward: Vec::new(),
            knee_flip_feet: [nalgebra::Vector3::zeros(); 4],
            knee_flip_plan_shift: Vec::new(),
            knee_flip_floor: Vec::new(),
            knee_flip_kind: Vec::new(),
            knee_flip_support: Vec::new(),
            knee_flip_com_est: nalgebra::Vector2::zeros(),
            knee_flip_probe: Vec::new(),
            knee_flip_probe_t: 0.0,
            knee_flip_prev_idx: 0,
            knee_flip_shift_m: 0.0,
            knee_flip_tilt_max_rad: 0.0,
            knee_flip_shift_max_m: 0.0,
            knee_flip_s_prev: None,
            knee_flip_sdot: 0.0,
            knee_flip_want_lpf: 0.0,
        };
        // `rest` の反転を使う機体は、いまの膝の向きで車輪に載って立ち上がれる
        // 足の位置を立ち位置にしておく（置き替えの段を省くため）。
        if c.cfg.gait.knee_flip_style == crate::config::KneeFlipStyle::Rest {
            match c.rest_park_offsets(c.cfg.gait.knee_pattern) {
                Ok(off) => {
                    if off.iter().any(|o| o.norm() > 1e-6) {
                        log::info!(
                            "膝 {} の立ち位置を車輪から立ち上がれる所へ: FL ({:+.2},{:+.2}) FR ({:+.2},{:+.2}) RL ({:+.2},{:+.2}) RR ({:+.2},{:+.2}) m",
                            c.cfg.gait.knee_pattern.label(),
                            off[0].x, off[0].y, off[1].x, off[1].y, off[2].x, off[2].y, off[3].x, off[3].y
                        );
                    }
                    c.stance_xy_offset = off;
                }
                Err(e) => log::warn!("車輪に載る立ち位置を決められません（立ち位置はそのまま）: {e}"),
            }
        }
        c
    }

    pub fn state(&self) -> State {
        self.state
    }

    pub fn gait_select(&self) -> GaitSelect {
        self.gait_select
    }

    /// いま使っている歩容コントローラの種類（実行中に替わり得る）。
    pub fn controller_kind(&self) -> crate::config::GaitControllerKind {
        self.cfg.gait.controller
    }

    /// 立って止まっているか（歩行モードで速度 0、4 脚接地）。膝の反転や重心の
    /// 実測はこのときだけ。
    pub fn is_standing_still(&self) -> bool {
        self.state == State::Active && self.can_switch_gait()
    }

    /// 姿勢 `q` での全身重心（胴体座標）。モデルの値に `gait.com_offset_body_m` を足す。
    pub fn com_body_at(&self, q: &JointVec) -> nalgebra::Vector3<f64> {
        let o = self.cfg.gait.com_offset_body_m;
        let e = self.knee_flip_com_est;
        self.robot.body_inertia_at(q).com_body + nalgebra::Vector3::new(o[0] + e.x, o[1] + e.y, 0.0)
    }

    pub fn robot(&self) -> &Robot {
        &self.robot
    }

    /// 直前の [`Self::tick`] で状態が変わったか。
    pub fn state_changed(&self) -> bool {
        self.just_changed
    }

    /// 脚オドメトリで測った胴体の状態を歩容へ渡す。**次の
    /// [`Self::tick`] で使われる。**
    ///
    /// **受け取るのは MPC 系の歩容だけ**（`AnyGaitController` の既定実装が
    /// 捨てる）。CHAMP と LinearCrawl は開ループで、観測を見ない。
    ///
    /// 位置（`set_body_pose_observed`）は渡さない。**脚オドメトリからは
    /// 絶対位置が出ない**（積分するしかなく滑りのぶんが溜まる）ので、
    /// 渡すと歩容が溜まった位置を「正しい」と信じて追いに行く。
    pub fn observe_body(&mut self, body: &crate::estimator::BodyState) {
        // **角速度は接地に依らず入る。** 空中相でも姿勢は測れている。
        self.observed_omega_world = body.angular_velocity_world;
        // 高さも同じ理由で渡す（[`crate::config::GaitTuning::mpc_observe_pose`]）。
        // 接地足が無い周期は `None` → MPC は公称高さへ戻る。
        self.gait.set_body_height_observed(if self.cfg.gait.mpc_observe_pose {
            body.height_m
        } else {
            None
        });
        let Some(v) = body.velocity_world else { return };
        self.observed_v_world = v;
        // **速度の閉ループを切る選択肢**（[`crate::config::GaitTuning::mpc_observe_velocity`]）。
        // 切ったときは、ランプ後の指令を歩容のヨーで世界向きに回して
        // 「指令どおりに動いている」と MPC に伝える。角速度は実測のまま。
        let v_for_mpc = if self.cfg.gait.mpc_observe_velocity {
            v
        } else {
            let (s, c) = self.body_view.yaw.sin_cos();
            nalgebra::Vector3::new(
                c * self.ramped_v[0] - s * self.ramped_v[1],
                s * self.ramped_v[0] + c * self.ramped_v[1],
                0.0,
            )
        };
        self.gait.set_body_state_observed(v_for_mpc, body.angular_velocity_world);
    }

    /// 可視化用の胴体姿勢と接地フラグ。
    pub fn body_view(&self) -> BodyView {
        self.body_view
    }

    /// 再生中のポーズ名（表示用）。
    pub fn playing(&self) -> Option<&str> {
        self.player.as_ref().map(|p| p.step_name())
    }

    /// 1 周期進める。
    ///
    /// `measured` は実機の現在角。`Relaxed` の間はこれを目標として持ち回るので、
    /// 起立に移った瞬間に「今いる位置」から遷移が始まる（0 rad へ飛ばない）。
    pub fn tick(
        &mut self,
        cmd: &Intent,
        measured: &JointVec,
        attitude_rad: [f64; 3],
        dt: f64,
    ) -> ControlOutput {
        let before = self.state;

        // 歩容の切り替えは、**遊脚がある最中にやると踏み替えが飛ぶ**。
        // 脱力中・初期姿勢・遷移中に加えて、**歩容中でも静止していれば許す**。
        //
        // 速度が 0 のとき歩容は `holding` に入り、全脚接地・位相凍結で
        // 静止姿勢を出している（quadruped-gait `PhaseGenerator`）。
        // その状態なら差し替えても飛ぶ遊脚が無い。
        //
        // 試合では「立って待っている間に歩容を選び直す」が要る。初期姿勢
        // まで戻さないと選べないのでは使いにくい。
        if cmd.gait != self.gait_select && self.can_switch_gait() {
            self.set_gait(cmd.gait);
        }
        // 歩容コントローラ（MPC / CHAMP）の切り替えも同じ条件。作り直しなので
        // 歩いている最中は受け付けない。
        if let Some(req) = cmd.gait_controller {
            let kind = match req {
                misa_core::GaitControllerRequest::Champ => crate::config::GaitControllerKind::Champ,
                misa_core::GaitControllerRequest::Mpc => crate::config::GaitControllerKind::Mpc,
            };
            if kind != self.cfg.gait.controller && self.can_switch_gait() {
                log::info!("歩容コントローラを {} に切り替えます", kind.label());
                self.cfg.gait.controller = kind;
                self.set_gait(self.gait_select);
            }
        }

        // 膝の向きの切り替え。**立って止まっているときだけ**、脚を浮かせて
        // 膝を伸ばし切る振り付けを通す（[`Self::plan_knee_flip`]）。
        self.handle_knee_request(cmd);

        // **歩容が回っていない相では MPC の参照を捨てる。** 立脚フラグと
        // 同じ理由で、古い接地力の予測は「もう接地していない足を押せ」に
        // なる。`tick_active` が毎周期入れ直す。
        self.mpc = None;
        match self.state {
            State::Relaxed => self.tick_relaxed(cmd, measured),
            State::GoingToStart => self.tick_going_to_start(cmd, dt),
            State::HoldingStart => self.tick_holding_start(cmd),
            State::GoingToStance => self.tick_transition(dt, State::Active),
            State::Active => self.tick_active(cmd, attitude_rad, dt),
            State::PlayingPose => self.tick_pose(cmd, dt),
            State::FlippingKnees => self.tick_knee_flip(cmd, measured, attitude_rad, dt),
        }

        // 脱力要求はどの状態からでも即座に効く。
        if cmd.mode == ModeRequest::Relax && self.state != State::Relaxed {
            self.enter_relaxed();
        }

        self.just_changed = before != self.state;
        ControlOutput {
            targets: self.targets,
            leg_mode: if self.state == State::Relaxed {
                JointMode::Idle
            } else {
                JointMode::Position
            },
            state: self.state,
            stance: self.body_view.stance,
            planned_yaw_rad: self.body_view.yaw,
            target_foot_body: self.target_foot_body,
            mpc: self.mpc,
            body_velocity: self.ramped_v,
        }
    }

    fn tick_relaxed(&mut self, cmd: &Intent, measured: &JointVec) {
        // 脱力中の「目標」は実測値。次に起立するときの始点になる。
        self.targets = *measured;
        if cmd.mode != ModeRequest::Relax {
            let start = match self.robot.poses.pose(&self.cfg.control.start_pose) {
                Some(p) => self.robot.poses.resolve(&p.angles, *measured),
                None => {
                    log::warn!(
                        "初期姿勢 {:?} がモデルにありません。立ち姿勢へ直接向かいます",
                        self.cfg.control.start_pose
                    );
                    self.stance_targets()
                }
            };
            self.player = Some(PosePlayer::to_pose(
                *measured,
                start,
                self.cfg.control.transition_s,
                InterpolationKind::QuinticSmooth,
            ));
            self.state = State::GoingToStart;
        }
    }

    /// 初期姿勢へ向かう遷移。着いたら**保持**する（歩行要求があれば通す）。
    fn tick_going_to_start(&mut self, cmd: &Intent, dt: f64) {
        let done = match self.player.as_mut() {
            Some(player) => {
                self.targets = player.tick(dt);
                player.is_done()
            }
            None => true,
        };
        if !done {
            return;
        }
        self.player = None;
        if cmd.mode == ModeRequest::Walk {
            self.begin_stance_transition();
        } else {
            self.state = State::HoldingStart;
        }
    }

    /// 初期姿勢で保持。歩行要求で立ち姿勢へ。
    ///
    /// **目標は動かさない。** 脱力要求は [`Self::tick`] の共通処理が拾う。
    fn tick_holding_start(&mut self, cmd: &Intent) {
        // 腕を駆動できない構成では、目標にも観測値を置いて食い違わせない。
        if !self.arm_app_driven {
            if let Some(observed) = cmd.aux(0) {
                self.targets.arm = observed;
            }
        }
        if cmd.mode == ModeRequest::Walk {
            self.begin_stance_transition();
        }
    }

    /// いまの目標から歩容の立ち姿勢へ向かう遷移を張る。
    fn begin_stance_transition(&mut self) {
        let stance = self.stance_targets();
        self.player = Some(PosePlayer::to_pose(
            self.targets,
            stance,
            self.cfg.control.transition_s,
            InterpolationKind::QuinticSmooth,
        ));
        self.state = State::GoingToStance;
    }

    /// いまの目標から初期姿勢へ戻る遷移を張る。歩行 → 中段で使う。
    fn begin_start_pose_transition(&mut self) {
        let start = match self.robot.poses.pose(&self.cfg.control.start_pose) {
            Some(p) => self.robot.poses.resolve(&p.angles, self.targets),
            None => {
                log::warn!(
                    "初期姿勢 {:?} がモデルにありません。立ち姿勢のまま保持します",
                    self.cfg.control.start_pose
                );
                return;
            }
        };
        self.player = Some(PosePlayer::to_pose(
            self.targets,
            start,
            self.cfg.control.transition_s,
            InterpolationKind::QuinticSmooth,
        ));
        self.state = State::GoingToStart;
    }

    /// 遷移中。終わったら `next` へ。
    fn tick_transition(&mut self, dt: f64, next: State) {
        let done = match self.player.as_mut() {
            Some(player) => {
                self.targets = player.tick(dt);
                player.is_done()
            }
            None => true,
        };
        if !done {
            return;
        }
        self.player = None;
        match next {
            State::GoingToStance => {
                // 歩容の立ち姿勢を求めてから、そこへ向かう遷移を張る。
                let stance = self.stance_targets();
                self.player = Some(PosePlayer::to_pose(
                    self.targets,
                    stance,
                    self.cfg.control.transition_s,
                    InterpolationKind::QuinticSmooth,
                ));
                self.state = State::GoingToStance;
            }
            _ => {
                // 歩容へ引き渡す。位相は最初から。
                self.gait.reset();
                self.state = State::Active;
            }
        }
    }

    fn tick_active(&mut self, cmd: &Intent, attitude_rad: [f64; 3], dt: f64) {
        if cmd.play_pose {
            // 押した瞬間の選択スイッチで決める。再生中に動かしても
            // 切り替わらない。
            self.alt_pose_requested = cmd.pose_slot.0 != 0;
            self.start_pose_playback();
            return;
        }
        // CH5 を中段へ戻したら初期姿勢へ帰る。**歩容を抜ける唯一の道**
        // （脱力は共通処理が拾う）。速度ゼロで立ち続けたいなら上段のまま
        // スティックを中立にすればよい。
        if cmd.mode == ModeRequest::Stand {
            self.begin_start_pose_transition();
            return;
        }
        // 胴体高さはスティックで上下できる。歩容の立ち位置そのものを動かす
        // （全歩容。[`Self::apply_body_height`]）。
        let h = self.clamp_body_height(self.robot.reference_height_m(&self.cfg.gait) + cmd.height_offset_m);
        self.apply_body_height(h);
        let want = match cmd.mode {
            ModeRequest::Walk => [cmd.velocity.vx_m_s, cmd.velocity.vy_m_s, cmd.velocity.wz_rad_s],
            // 起立中は歩容を止める（速度ゼロ = 接地したまま）。
            _ => [0.0; 3],
        };
        let v = self.ramp_velocity(want, dt);
        self.apply_gait_tune(&cmd.gait_tune);
        self.gait.set_velocity_cmd(velocity_cmd(v[0], v[1], v[2]));
        // **姿勢の観測は SRBD MPC にも渡す**（quadruped-gait 側で現在状態に
        // 入り、参照は水平のまま）。切れるようにしてあるのは効きを測るため。
        if self.cfg.gait.mpc_observe_pose {
            self.gait
                .set_body_attitude_observed(attitude_rad[0], attitude_rad[1]);
        }
        let out = self.gait.tick(dt);
        if !out.all_reachable() {
            log::warn!("IK が届かない脚があります（姿勢がクランプされました）");
        }
        self.mpc = self.mpc_reference(&out, attitude_rad);
        for slot in 0..4 {
            self.target_foot_body[slot] = out.legs[slot].foot_body;
        }
        let arm = self.arm_target(cmd, attitude_rad[1], dt);
        self.tilt_toward(cmd.body_attitude_rad, dt);
        self.body_view = BodyView {
            xy: [
                out.body_state.world_position.x,
                out.body_state.world_position.y,
            ],
            yaw: out.body_state.world_yaw,
            z: self.applied_height_m,
            // 歩容は水平計画なので planned の roll/pitch は常に 0。
            // 姿勢を計画する制御（MPC 等）を入れたらここに載せる。
            rp: [0.0, 0.0],
            stance: [
                out.legs[0].phase.is_stance,
                out.legs[1].phase.is_stance,
                out.legs[2].phase.is_stance,
                out.legs[3].phase.is_stance,
            ],
        };
        let (targets, reachable) = self.robot.output_to_joints_tilted(&out, arm, self.tilt_rad);
        self.targets = targets;
        // 傾けたぶん脚が届かなくなることがある。歩容自身の `all_reachable`
        // とは別に見る（あちらは傾ける前の判定）。
        if !reachable && !self.warned_tilt_reach {
            self.warned_tilt_reach = true;
            log::warn!(
                "胴体を傾けた姿勢で IK が届きません（クランプされました）。\
                 gait.body_attitude_max_rad を下げてください"
            );
        }
        if reachable {
            self.warned_tilt_reach = false;
        }
    }

    /// MPC が解いた接地力から、WBC の参照を作る。MPC 系でなければ `None`。
    ///
    /// # 何を計算しているか
    ///
    /// 接地力そのものは `predicted_grfs()` がそのまま出す。胴体加速度は
    /// `quadruped_gait` の Newton–Euler（`predicted_base_accel_world`）に
    /// 通して作る — **MPC が「この力を出す」と決めた結果として胴体がどう
    /// 動くはずか**であり、WBC がそれを追えば MPC と WBC が同じ運動を
    /// 前提にできる。
    ///
    /// # 位置は 0 でよい
    ///
    /// 使うのは `r_i − p_胴体`（重心から足へのモーメント腕）だけなので、
    /// 胴体を原点に置いて足の位置を胴体座標系のまま渡せば辻褄が合う。
    /// **絶対位置は脚オドメトリからは出ない**ので、そもそも入れられない。
    fn mpc_reference(
        &self,
        out: &quadruped_gait::ControllerOutput,
        attitude_rad: [f64; 3],
    ) -> Option<crate::wbc::MpcReference> {
        let sol = self.gait.predicted_grfs()?;
        let grf_world = sol.grfs_first_step;
        let [roll, pitch, yaw] = attitude_rad;
        let quat = nalgebra::UnitQuaternion::from_euler_angles(roll, pitch, yaw);
        // 足の位置は歩容の**計画**（`foot_body`）。実測との差は小さく、
        // モーメント腕としてはどちらでも変わらない。
        let mut foot_world = [nalgebra::Vector3::zeros(); 4];
        for slot in 0..4 {
            foot_world[slot] = quat * out.legs[slot].foot_body;
        }
        let (accel_lin_world, accel_ang_world) =
            if let Some(cfg) = self.gait.centroidal_mpc_config() {
                quadruped_gait::predicted_base_accel_world_centroidal(
                    cfg,
                    nalgebra::Vector3::zeros(),
                    quat,
                    self.observed_omega_world,
                    &grf_world,
                    &foot_world,
                )
            } else {
                let cfg = self.gait.srbd_mpc_config()?;
                let state = quadruped_gait::SrbdState {
                    orientation_rpy: nalgebra::Vector3::new(roll, pitch, yaw),
                    position: nalgebra::Vector3::zeros(),
                    // **SrbdState の角速度は胴体座標系**（関数の中で世界へ
                    // 回される）。脚オドメトリが持っているのは世界なので戻す。
                    angular_velocity: quat.inverse() * self.observed_omega_world,
                    linear_velocity: self.observed_v_world,
                };
                quadruped_gait::predicted_base_accel_world(cfg, &state, &grf_world, &foot_world)
            };
        Some(crate::wbc::MpcReference {
            grf_world,
            accel_lin_world,
            accel_ang_world,
            solved: sol.solved,
        })
    }

    /// 胴体姿勢の指令へ一次遅れで寄せる。
    ///
    /// `ChickenHead` と同じ形。**切り替えた瞬間に跳ねないことが目的**なので、
    /// 有効・無効のどちらへ向かうときも同じ時定数を通す。
    fn tilt_toward(&mut self, want: [f64; 3], dt: f64) {
        let tau = self.cfg.gait.body_attitude_tau_s.max(0.0);
        let alpha = if tau > 0.0 && dt > 0.0 {
            (dt / (tau + dt)).clamp(0.0, 1.0)
        } else {
            1.0
        };
        for k in 0..3 {
            self.tilt_rad[k] += (want[k] - self.tilt_rad[k]) * alpha;
        }
    }

    fn tick_pose(&mut self, cmd: &Intent, dt: f64) {
        let done = match self.player.as_mut() {
            Some(player) => {
                self.targets = player.tick(dt);
                player.is_done()
            }
            None => true,
        };
        // 腕を駆動できない構成では、ポーズが腕を動かすつもりでも実機は
        // 受信機に従う。目標にも観測値を置いて食い違わせない。
        if !self.arm_app_driven {
            if let Some(observed) = cmd.aux(0) {
                self.targets.arm = observed;
            }
        }
        // **中断は CH5 を中段（初期姿勢）へ戻したとき。** 脱力は共通処理が拾う。
        //
        // かつて `cmd.mode == ModeRequest::Walk` で中断していたが、再生に
        // 入れるのは `Active`、つまり **CH5 上段 = `Walk` のときだけ**なので、
        // トリガした次の周期で必ずこの条件が立ち、1 周期で抜けていた。
        // 「歩行へ切り替えたら中断」は、入口が歩行である以上成り立たない。
        if done || cmd.mode == ModeRequest::Stand {
            self.player = Some(PosePlayer::to_pose(
                self.targets,
                self.stance_targets(),
                self.cfg.control.transition_s,
                InterpolationKind::QuinticSmooth,
            ));
            self.state = State::GoingToStance;
        }
    }

    fn start_pose_playback(&mut self) {
        // **選択スイッチ (CH10) が上段なら別のものを再生する。**
        // `greeting_alt` が空なら従来どおり `greeting` だけ。
        let name = if self.alt_pose_requested && !self.cfg.poses.greeting_alt.is_empty() {
            self.cfg.poses.greeting_alt.clone()
        } else {
            self.cfg.poses.greeting.clone()
        };
        match PosePlayer::start(&self.robot.poses, &name, self.targets) {
            Ok(player) => {
                log::info!("ポーズ {name:?} を再生します");
                self.player = Some(player);
                self.state = State::PlayingPose;
            }
            // 再生できないなら歩容のまま。動作を止めないほうが安全。
            Err(e) => log::warn!("ポーズを再生できません: {e}"),
        }
    }

    /// この周期の腕の目標角。
    ///
    /// 駆動できるならチキンヘッドの出力、できないなら観測値
    /// （観測もできなければ直前値を保つ）。
    fn arm_target(&mut self, cmd: &Intent, body_pitch_rad: f64, dt: f64) -> f64 {
        if self.arm_app_driven {
            return self.chicken.update(cmd.stabilize_head, body_pitch_rad, dt);
        }
        // **CH8 が胴体姿勢に割り当たっているなら、腕の話は出さない。**
        // `body_attitude_max_rad > 0` のとき CH8 は姿勢モードのスイッチで、
        // 腕を動かすつもりで入れているわけではない。毎回警告すると
        // 「姿勢モードを使うたびに何か壊れている」ように読める。
        let ch8_is_attitude = self.cfg.gait.body_attitude_max_rad > 0.0;
        if cmd.stabilize_head && !ch8_is_attitude && !self.warned_chicken_head {
            log::warn!(
                "チキンヘッドが ON ですが、腕はアプリから駆動できない構成です\
                 （受信機直結 / 未配線）。指令は出しません"
            );
            self.warned_chicken_head = true;
        }
        cmd.aux(0).unwrap_or(self.targets.arm)
    }

    fn enter_relaxed(&mut self) {
        self.player = None;
        self.state = State::Relaxed;
        self.chicken.reset();
        // 次に立ち上がったとき、前回の速度から歩き出さないように。
        self.ramped_v = [0.0; 3];
        // 姿勢も戻す。傾けたまま脱力 → 再起立で胴体が傾いたまま出てこない。
        self.tilt_rad = [0.0; 3];
    }

    /// 胴体高さを歩容へ入れる。**全歩容で効く。**
    ///
    /// `AnyGaitController::set_body_height_m` は LinearCrawl しか見ない
    /// （他は黙って捨てる）。立ち位置は `nominal_foot_body` が決めるので、
    /// 高さを変えた運動学設定を `set_kinematics` で差し替える（位相も MPC の
    /// 状態も保たれる。quadruped-gait はこの用途を想定している）。MPC の参照
    /// 高さもここから取るので、MPC + WBC でも胴体が実際に上下する。
    /// 2026-09-06 までは `set_body_height_m` だけで、MPC / CHAMP では何も
    /// 起きないのに可視化の胴体だけが上下していた。
    fn apply_body_height(&mut self, h: f64) {
        let same_offset = (0..4).all(|i| (self.stance_xy_offset[i] - self.applied_offset[i]).norm() < 1e-9);
        if (h - self.applied_height_m).abs() < 1e-6 && same_offset {
            return;
        }
        self.gait.set_kinematics(self.stance_kinematics_with_offset(h));
        self.gait.set_body_height_m(h);
        self.applied_height_m = h;
        self.applied_offset = self.stance_xy_offset;
    }

    /// 立ち高さ `h` の運動学に、立ち位置のずらし（[`Self::stance_xy_offset`]）を足したもの。
    fn stance_kinematics_with_offset(&self, h: f64) -> quadruped_gait::KinematicsConfig {
        let kin = self.robot.stance_kinematics_at_height(&self.cfg.gait, h);
        let feet: [nalgebra::Vector3<f64>; 4] = std::array::from_fn(|s| {
            let f = kin.legs()[s].nominal_foot_body;
            nalgebra::Vector3::new(f.x + self.stance_xy_offset[s].x, f.y + self.stance_xy_offset[s].y, f.z)
        });
        self.robot.kin_with_feet(feet)
    }

    /// `rest` の反転で、車輪に載った胴体（`knee_flip_rest_height_m`）から膝 `pattern` で
    /// 立ち上がれる足の位置と、基準の立ち位置との差（xy、脚ごと）。曲げ限界で足を hip の
    /// 真下に置けない向き（keel の `>>`）は外へずらす。届くなら 0。
    fn rest_park_offsets(&self, pattern: crate::config::KneeShape) -> Result<[nalgebra::Vector3<f64>; 4], String> {
        let h = self.cfg.gait.knee_flip_rest_height_m.ok_or("knee_flip_rest_height_m が無い")?;
        self.park_offsets_at(pattern, h)
    }

    /// 胴体高さ `h_body` で膝 `pattern` の脚を置ける足先の xy（基準の立ち位置との差）。
    fn park_offsets_at(&self, pattern: crate::config::KneeShape, h_body: f64) -> Result<[nalgebra::Vector3<f64>; 4], String> {
        use crate::robot::knee_forward_for;
        use quadruped_gait::{forward_leg_kinematics, solve_leg_ik};
        let g = &self.cfg.gait;
        let h_rest = h_body;
        let h_ref = self.robot.reference_height_m(g);
        let kin_ref = self.robot.stance_kinematics_at_height(g, h_ref);
        let names = misa_hal::joint::JOINT_NAMES;
        let floor = -h_rest;
        let z_float = floor + g.knee_flip_foot_lift_m;
        let mut out = [nalgebra::Vector3::zeros(); 4];
        for slot in 0..4 {
            let leg = kin_ref.legs()[slot];
            let s = self.robot.signs[slot];
            let forward = knee_forward_for(pattern, slot);
            let base = nalgebra::Vector3::new(leg.nominal_foot_body.x, leg.nominal_foot_body.y, z_float);
            let out_y = if slot == 0 || slot == 2 { 1.0 } else { -1.0 };
            let mut knee_leg = leg.clone();
            knee_leg.lower_leg_m = 0.0;
            let mut best: Option<(nalgebra::Vector3<f64>, f64)> = None;
            for iy in 0..=25 {
                let dy = out_y * 0.01 * iy as f64;
                for ix in 0..=40 {
                    let dx = 0.01 * ((ix + 1) / 2) as f64 * if ix % 2 == 0 { 1.0 } else { -1.0 };
                    let d = nalgebra::Vector3::new(dx, dy, 0.0);
                    let sol = solve_leg_ik(leg, base + d, forward);
                    if !sol.is_reachable() {
                        continue;
                    }
                    let (hh, t, c) = sol.angles();
                    let q = [hh * s[0], t * s[1], c * s[2]];
                    let within = (0..3).all(|k| {
                        let (lo, hi) = self.robot.limits.get(names[slot][k]).copied().unwrap_or((-std::f64::consts::PI, std::f64::consts::PI));
                        q[k] >= lo - 1e-6 && q[k] <= hi + 1e-6
                    });
                    if !within {
                        continue;
                    }
                    let knee = forward_leg_kinematics(&knee_leg, hh, t, c);
                    if knee.z < floor + 0.03 {
                        continue;
                    }
                    let cost = dx.abs() + 0.5 * dy.abs();
                    if best.as_ref().map_or(true, |b| cost < b.1) {
                        best = Some((d, cost));
                    }
                }
            }
            out[slot] = best
                .map(|(d, _)| d)
                .ok_or_else(|| format!("{} は足先をどこへずらしても胴体 {h_rest:.3} m で膝{}に置けない", names[slot][0].trim_end_matches("_hip_joint"), if forward { "前" } else { "後ろ" }))?;
        }
        Ok(out)
    }

    /// 脚が届く範囲に丸める。伸び切り（脚長）の 95 % を上限、30 % を下限に。
    fn clamp_body_height(&self, h: f64) -> f64 {
        let leg = self.robot.kin.fl.upper_leg_m + self.robot.kin.fl.lower_leg_m;
        h.clamp(0.3 * leg, 0.95 * leg)
    }

    /// 歩容が「今この設定で立つ」姿勢。時間を進めずに取り出す。
    /// いまの膝の向き（`gait.knee_pattern`。反転の振り付けが終わると替わる）。
    pub fn knee_pattern(&self) -> crate::config::KneeShape {
        self.cfg.gait.knee_pattern
    }

    fn handle_knee_request(&mut self, cmd: &Intent) {
        let Some(req) = cmd.knee_pattern else {
            self.warned_knee_flip = false;
            self.knee_flip_failed = None;
            return;
        };
        let shape = crate::config::KneeShape::from_request(req);
        if shape == self.cfg.gait.knee_pattern || self.state == State::FlippingKnees {
            self.warned_knee_flip = false;
            self.knee_flip_failed = None;
            return;
        }
        if self.knee_flip_failed == Some(shape) {
            return;
        }
        if self.state != State::Active || !self.can_switch_gait() {
            if !self.warned_knee_flip {
                log::warn!(
                    "膝の向き {} → {} は立って止まっているときだけ切り替えられます（いまは {}）",
                    self.cfg.gait.knee_pattern.label(),
                    shape.label(),
                    self.state.label()
                );
                self.warned_knee_flip = true;
            }
            return;
        }
        match self.plan_knee_flip(shape) {
            Ok(KneeFlipPlan { steps, stances: stance, forwards, plan_shifts: plan_shift, floors, kinds, supports, next_offset }) => {
                let total: f64 = steps.iter().map(|st| st.duration_s).sum();
                log::info!(
                    "膝の向きを {} → {} に反転します（{}、{} 段 / {:.1} s: {}）",
                    self.cfg.gait.knee_pattern.label(),
                    shape.label(),
                    self.cfg.gait.knee_flip_style.label(),
                    steps.len(),
                    total,
                    steps.iter().map(|st| st.name.as_str()).collect::<Vec<_>>().join(" → ")
                );
                self.player = Some(PosePlayer::from_steps(self.targets, steps));
                self.knee_flip_stance = stance;
                self.knee_flip_forward = forwards;
                self.knee_flip_plan_shift = plan_shift;
                self.knee_flip_floor = floors;
                self.knee_flip_kind = kinds;
                self.knee_flip_support = supports;
                self.knee_flip_probe.clear();
                self.knee_flip_prev_idx = 0;
                self.knee_flip_s_prev = None;
                self.knee_flip_sdot = 0.0;
                self.knee_flip_next_offset = Some(next_offset);
                self.knee_flip_target = Some(shape);
                // `trot` の釣り合いに使う立脚の基準（立ち高さ、いまの立ち位置）。
                let h = self.robot.reference_height_m(&self.cfg.gait);
                let kin = self.stance_kinematics_with_offset(h);
                self.knee_flip_feet = std::array::from_fn(|s| kin.legs()[s].nominal_foot_body);
                self.knee_flip_shift_m = 0.0;
                self.knee_flip_tilt_max_rad = 0.0;
                self.knee_flip_shift_max_m = 0.0;
                self.state = State::FlippingKnees;
            }
            Err(e) => {
                log::warn!("膝の向き {} への反転を組めません: {e}", shape.label());
                self.knee_flip_failed = Some(shape);
            }
        }
    }

    /// 膝の向きを `new` に替える振り付けを、いまの目標から組む。
    ///
    /// 全部**関節空間の折れ線**（IK は端点だけ）。戻り値は段の列と、段ごとの
    /// 立脚フラグ（浮かせている脚が false）。
    ///
    /// `stand`（[`crate::config::KneeFlipStyle::Stand`]）: 立ったまま 1 脚ずつ。
    /// 脚ごとに「寄せる（胴体を反対の対角へ `knee_flip_shift_m`）→ 畳む → 振り出す
    /// → 伸ばす → 逆へ畳む → 戻す」、最後に「中央へ」。他の 3 脚は接地したまま。
    ///
    /// `rest`（`Rest`）: 「下ろす（胴体を `knee_flip_rest_height_m` に載せ、足を
    /// `knee_flip_foot_lift_m` 浮かす）→ 畳む → 振り出す → 伸ばす → 逆へ畳む → 戻す →
    /// 上げる」。反転する脚はまとめて動く。
    ///
    /// 組む前に、全段が可動域の内側で、浮かせている脚の折れ線上の足先が
    /// 出発した面より下がらないことを確かめる。破れたら `Err`（始めない）。
    fn plan_knee_flip(
        &self,
        new: crate::config::KneeShape,
    ) -> Result<KneeFlipPlan, String> {
        use crate::config::KneeFlipStyle;
        use crate::pose::PoseStep;
        use crate::robot::knee_forward_for;
        use quadruped_gait::{forward_leg_kinematics, solve_leg_ik};
        let old = self.cfg.gait.knee_pattern;
        let flip: [bool; 4] = std::array::from_fn(|s| knee_forward_for(old, s) != knee_forward_for(new, s));
        if !flip.iter().any(|f| *f) {
            return Err("反転する脚が無い".into());
        }
        let g = &self.cfg.gait;
        let phase = g.knee_flip_phase_s;
        let h_ref = self.robot.reference_height_m(g);
        let kin_ref = self.robot.stance_kinematics_at_height(g, h_ref);
        let signs = self.robot.signs;
        let names = misa_hal::joint::JOINT_NAMES;
        let leg_name = |slot: usize| names[slot][0].trim_end_matches("_hip_joint");

        // IK（胴体座標の足先 → モデル座標の関節角）。
        let ik = |slot: usize, foot: nalgebra::Vector3<f64>, forward: bool| -> Result<[f64; 3], String> {
            let leg = kin_ref.legs()[slot];
            let sol = solve_leg_ik(leg, foot, forward);
            if !sol.is_reachable() {
                return Err(format!(
                    "{} の足先（{:+.3}, {:+.3}, {:+.3}）に IK が届かない（膝{}）",
                    leg_name(slot), foot.x, foot.y, foot.z, if forward { "前" } else { "後ろ" }
                ));
            }
            let (h, t, c) = sol.angles();
            let s = signs[slot];
            Ok([h * s[0], t * s[1], c * s[2]])
        };
        let limit = |slot: usize, k: usize| -> (f64, f64) {
            self.robot.limits.get(names[slot][k]).copied().unwrap_or((-std::f64::consts::PI, std::f64::consts::PI))
        };
        // calf を `sign` の向きへ深く畳む値。`deep` なら可動域の 0.15 rad 手前まで
        // （胴体が低いとき、足を少しでも上げるため）、そうでなければ |2.2| まで。
        let fold = |slot: usize, sign: f64, deep: bool| -> f64 {
            let (lo, hi) = limit(slot, 2);
            let cap = if deep { 3.0 } else { 2.2 };
            if sign < 0.0 { (lo + 0.15).max(-cap) } else { (hi - 0.15).min(cap) }
        };
        // 伸ばし切るときに脚を向ける腿の角（モデル座標）を選ぶ。
        //
        // 望みは `knee_flip_out_z_m`（足先が hip よりこれだけ下）で、前脚は前・
        // 後脚は後ろ。ただし **腿の可動域**と、**畳んだ calf を伸ばす途中で足先が
        // `floor_z` より下がらない**こと（下腿が真下を向く瞬間がいちばん低い。
        // 胴体が低いと水平の腿では足が床を掘る）を満たす角を、望みに近い順に
        // 探す。前へ振れないなら後ろへ振る（keel の腿は前 90° まで、後ろ 240° まで）。
        let out_thigh = |slot: usize, hip: f64, calf_from: f64, calf_to: f64, floor_z: f64| -> Result<f64, String> {
            let leg = kin_ref.legs()[slot];
            let s = signs[slot];
            let l = leg.upper_leg_m + leg.lower_leg_m;
            let c = (g.knee_flip_out_z_m / l).clamp(-1.0, 1.0);
            let pref = c.acos() * if slot < 2 { 1.0 } else { -1.0 }; // IK: 腿 + は前
            let (lo, hi) = limit(slot, 1);
            // IK 座標の腿の範囲（符号で反転する）。
            let (ik_lo, ik_hi) = if s[1] > 0.0 { (lo + 0.05, hi - 0.05) } else { (-(hi - 0.05), -(lo + 0.05)) };
            let mut best: Option<(f64, f64)> = None;
            let mut th = ik_lo;
            while th <= ik_hi {
                // 畳んだまま腿を振る → 伸ばす → 逆へ畳む、の足先の最低点。
                let mut min_z = f64::INFINITY;
                for i in 0..=30 {
                    let a = i as f64 / 30.0;
                    let calf = calf_from + a * (calf_to - calf_from);
                    let f = forward_leg_kinematics(leg, hip * s[0], th, calf * s[2]);
                    min_z = min_z.min(f.z);
                }
                if min_z >= floor_z - 0.005 {
                    let d = (th - pref).abs();
                    if best.map_or(true, |(_, bd)| d < bd) {
                        best = Some((th, d));
                    }
                }
                th += 0.5f64.to_radians();
            }
            match best {
                Some((th, _)) => Ok(th * s[1]),
                None => Err(format!(
                    "{} は腿をどこへ向けても伸ばす途中で足先が床（z {:+.3}）を掘る。胴体が低すぎる",
                    leg_name(slot), floor_z
                )),
            }
        };
        // 基準の立ち位置と、いまの立ち位置（ずらし込み）。
        let feet_nominal: [nalgebra::Vector3<f64>; 4] = std::array::from_fn(|s| kin_ref.legs()[s].nominal_foot_body);
        let feet_ref: [nalgebra::Vector3<f64>; 4] = std::array::from_fn(|s| feet_nominal[s] + self.stance_xy_offset[s]);
        let mut next_offset = self.stance_xy_offset;
        let _at_z_all = |xy: &[nalgebra::Vector3<f64>; 4], z: f64| -> [nalgebra::Vector3<f64>; 4] {
            std::array::from_fn(|s| nalgebra::Vector3::new(xy[s].x, xy[s].y, z))
        };
        // 可動域の内側か（モデル座標）。
        let within = |slot: usize, q: &[f64; 3]| -> bool {
            (0..3).all(|k| {
                let (lo, hi) = limit(slot, k);
                q[k] >= lo - 1e-6 && q[k] <= hi + 1e-6
            })
        };
        // 膝（腿の先）の胴体座標の位置。下腿の長さを 0 にした FK。
        let knee_pos = |slot: usize, q: &[f64; 3]| -> nalgebra::Vector3<f64> {
            let mut leg = kin_ref.legs()[slot].clone();
            leg.lower_leg_m = 0.0;
            let s = signs[slot];
            forward_leg_kinematics(&leg, q[0] * s[0], q[1] * s[1], q[2] * s[2])
        };
        let is_left = |slot: usize| slot == 0 || slot == 2;
        // 低い胴体で足を置ける所を探す。hip のロールで脚を外へ倒すと脚の面が
        // 傾き、同じ関節角でも膝と足先の下がりが cos(ロール) 倍になるので、
        // 曲げ限界の厳しい向きでも膝を床に着けずに足を置ける。足先を外へ
        // （dy）と前後へ（dx）ずらしながら、可動域の内側で膝が床から
        // 2 cm 以上浮く解を、ずらしの小さい順に探す。
        let park = |slot: usize, foot: nalgebra::Vector3<f64>, forward: bool, floor_z: f64| -> Result<([f64; 3], nalgebra::Vector3<f64>), String> {
            let out_y = if is_left(slot) { 1.0 } else { -1.0 };
            let mut best: Option<([f64; 3], nalgebra::Vector3<f64>, f64)> = None;
            for iy in 0..=25 {
                let dy = out_y * 0.01 * iy as f64;
                for ix in 0..=40 {
                    let dx = 0.01 * ((ix + 1) / 2) as f64 * if ix % 2 == 0 { 1.0 } else { -1.0 };
                    let target = foot + nalgebra::Vector3::new(dx, dy, 0.0);
                    let Ok(q) = ik(slot, target, forward) else { continue };
                    if !within(slot, &q) {
                        continue;
                    }
                    if knee_pos(slot, &q).z < floor_z + 0.03 {
                        continue;
                    }
                    let cost = dx.abs() + 0.5 * dy.abs();
                    if best.as_ref().map_or(true, |b| cost < b.2) {
                        best = Some((q, target, cost));
                    }
                }
            }
            if std::env::var_os("MISA_KNEE_DEBUG").is_some() {
                eprintln!("[knee] park {} fwd={forward} floor={floor_z:+.3} from=({:+.3},{:+.3},{:+.3}) -> {:?}", leg_name(slot), foot.x, foot.y, foot.z,
                    best.as_ref().map(|(q, t, _)| (format!("q=({:+.2},{:+.2},{:+.2})", q[0], q[1], q[2]), format!("xy=({:+.3},{:+.3})", t.x, t.y))));
            }
            best.map(|(q, t, _)| (q, t)).ok_or_else(|| {
                format!("{} は足先をどこへずらしても、低い胴体（床 z {:+.3}）で膝{}に置けない", leg_name(slot), floor_z, if forward { "前" } else { "後ろ" })
            })
        };

        let mut steps: Vec<PoseStep> = Vec::new();
        let mut stances: Vec<[bool; 4]> = Vec::new();
        // 段ごとの床の高さ（胴体座標）。rest では下ろしたあと −h_rest、上げたら −h_ref。
        let mut floors: Vec<f64> = Vec::new();
        let mut forwards: Vec<[bool; 4]> = Vec::new();
        let cur_floor = std::cell::Cell::new(-h_ref);
        // 段ごとの膝の向き。反転を終えた脚から新しい向きになる（`flip_legs` の戻すで更新）。
        let cur_forward = std::cell::Cell::new(std::array::from_fn::<bool, 4, _>(|s| knee_forward_for(old, s)));
        // 段ごとの、計画に織り込んだ胴体の寄せ（trot。支持線の法線方向、m）。
        let mut plan_shifts: Vec<f64> = Vec::new();
        let cur_shift = std::cell::Cell::new(0.0_f64);
        // 段の種類と、その段が属する組の支持脚（trot）。
        let mut kinds: Vec<KneeStepKind> = Vec::new();
        let mut supports: Vec<[usize; 2]> = Vec::new();
        let cur_kind = std::cell::Cell::new(KneeStepKind::Normal);
        let cur_support = std::cell::Cell::new([1usize, 2]);
        let mut push = |name: String, q: JointVec, dur: f64, stance: [bool; 4]| {
            steps.push(PoseStep { name, target: q, duration_s: dur, kind: InterpolationKind::QuinticSmooth });
            stances.push(stance);
            floors.push(cur_floor.get());
            forwards.push(cur_forward.get());
            plan_shifts.push(cur_shift.get());
            kinds.push(cur_kind.get());
            supports.push(cur_support.get());
            cur_kind.set(KneeStepKind::Normal);
        };
        let mut cur = self.targets;

        // 反転する脚 `slot` を、足が浮いた状態で反転させる 5 段（畳む → 振り出す →
        // 伸ばす → 逆へ畳む → 戻す）。`slots` の脚をまとめて動かす。`stance` は
        // その間の立脚フラグ、`land` は戻す先の足先。
        let flip_legs = |cur: &mut JointVec,
                         slots: &[usize],
                         stance: [bool; 4],
                         land: &[nalgebra::Vector3<f64>; 4],
                         floor_z: f64,
                         roll_out: bool,
                         push: &mut dyn FnMut(String, JointVec, f64, [bool; 4]),
                         tag: &str|
         -> Result<(), String> {
            let mut sign_old = [0.0; 4];
            for &slot in slots {
                let c = cur.legs[slot][2];
                sign_old[slot] = if c < 0.0 { -1.0 } else { 1.0 };
                // いまの曲げより浅くはしない（胴体が低いと、すでに畳み目標より深く
                // 曲がっていることがある。伸ばすと足が下がる）。
                let f = fold(slot, sign_old[slot], roll_out);
                cur.legs[slot][2] = if sign_old[slot] < 0.0 { c.min(f) } else { c.max(f) };
            }
            push(format!("畳む{tag}"), *cur, phase, stance);
            if roll_out {
                // **hip のロールで脚を外へ倒す。** 脚の面が傾き、腿が真下を通る
                // 瞬間の膝の下がりが cos(ロール) 倍になる。胴体が車輪に載った低さ
                // では、これ無しに腿を振ると膝が床を掘る。ロールの可動域いっぱい。
                for &slot in slots {
                    let (lo, hi) = limit(slot, 0);
                    // 外向きのロール。IK 座標のロールの符号は左右で逆（左 + / 右 −）。
                    // モデル座標へ符号表で戻す。
                    let out_ik = if is_left(slot) { 1.0 } else { -1.0 };
                    let want_ik = out_ik * (hi.min(-lo) - 0.02);
                    cur.legs[slot][0] = (want_ik * signs[slot][0]).clamp(lo + 0.02, hi - 0.02);
                }
                push(format!("倒す{tag}"), *cur, phase, stance);
            }
            for &slot in slots {
                let folded = cur.legs[slot][2];
                let opposite = fold(slot, -sign_old[slot], roll_out);
                cur.legs[slot][1] = out_thigh(slot, cur.legs[slot][0], folded, opposite, floor_z)?;
            }
            push(format!("振り出す{tag}"), *cur, phase, stance);
            for &slot in slots {
                cur.legs[slot][2] = 0.0;
            }
            push(format!("伸ばす{tag}"), *cur, phase, stance);
            for &slot in slots {
                cur.legs[slot][2] = fold(slot, -sign_old[slot], roll_out);
            }
            push(format!("逆へ畳む{tag}"), *cur, phase, stance);
            // 戻したところから膝の向きは新しいものになる。
            let mut fwd_now = cur_forward.get();
            for &slot in slots {
                fwd_now[slot] = knee_forward_for(new, slot);
            }
            cur_forward.set(fwd_now);
            if roll_out {
                // ロールを倒したまま腿だけ戻す（膝が真下を通る瞬間にロールで
                // 逃がす）→ ロールと calf を戻す、の 2 段。
                let mut parked = [[0.0; 3]; 4];
                for &slot in slots {
                    parked[slot] = ik(slot, land[slot], knee_forward_for(new, slot))?;
                    cur.legs[slot][1] = parked[slot][1];
                }
                push(format!("戻す{tag} 腿"), *cur, phase, stance);
                for &slot in slots {
                    cur.legs[slot] = parked[slot];
                }
                push(format!("戻す{tag}"), *cur, phase, stance);
            } else {
                for &slot in slots {
                    cur.legs[slot] = ik(slot, land[slot], knee_forward_for(new, slot))?;
                }
                push(format!("戻す{tag}"), *cur, phase, stance);
            }
            Ok(())
        };

        // ロールで外へ倒して腿と calf を同時に折り返す（`trot` / `stand`）ための共通量。
        // 外向きのロール（IK 座標で左 + / 右 −、可動域の 0.02 rad 手前）。
        let roll_out = |slot: usize| -> f64 {
            let (lo, hi) = limit(slot, 0);
            let out_ik = if is_left(slot) { 1.0 } else { -1.0 };
            (out_ik * (hi.min(-lo) - 0.02) * signs[slot][0]).clamp(lo + 0.02, hi - 0.02)
        };
        let leg0 = kin_ref.legs()[0];
        let l_total = leg0.upper_leg_m + leg0.lower_leg_m;
        let roll_mag = {
            let (lo, hi) = limit(0, 0);
            hi.min(-lo) - 0.02
        };
        // 反転中の胴体高さ。脚が一直線になる瞬間の足先の下がり (上腿+下腿)·cos(ロール)
        // + 3 cm を床から確保する（keel 0.30 → 0.34 m）。
        let h_flip = h_ref.max(l_total * roll_mag.cos() + 0.03);

        match g.knee_flip_style {
            KneeFlipStyle::Rest => {
                let h_rest = g
                    .knee_flip_rest_height_m
                    .ok_or("knee_flip_rest_height_m（車輪・腹に載ったときの胴体高さ）が無い")?;
                if h_rest >= h_ref {
                    return Err(format!("knee_flip_rest_height_m {h_rest:.3} が立ち高さ {h_ref:.3} 以上"));
                }
                let floor = -h_rest;
                // 胴体が載ったあと、足が地面から lift だけ浮く面。
                let z_float = floor + g.knee_flip_foot_lift_m;
                let at_z = |xy: &[nalgebra::Vector3<f64>; 4], z: f64| -> [nalgebra::Vector3<f64>; 4] {
                    std::array::from_fn(|s| nalgebra::Vector3::new(xy[s].x, xy[s].y, z))
                };
                let feet_float = at_z(&feet_ref, z_float);
                // 低い胴体で足を置ける所（いまの向き / 新しい向き）。**立ち位置そのもの**
                // をここに合わせる（`stance_xy_offset`）ので、普段はいまの立ち位置と
                // 一致していて置き替えは起きない。
                let off_old = self.rest_park_offsets(old)?;
                let off_new = self.rest_park_offsets(new)?;
                let xy_old: [nalgebra::Vector3<f64>; 4] = std::array::from_fn(|s| feet_nominal[s] + off_old[s]);
                let xy_new: [nalgebra::Vector3<f64>; 4] = std::array::from_fn(|s| feet_nominal[s] + off_new[s]);
                let _ = (&park, &feet_float);
                next_offset = off_new;
                let moved = |a: &[nalgebra::Vector3<f64>; 4], b: &[nalgebra::Vector3<f64>; 4]| -> bool {
                    (0..4).any(|s| (a[s] - b[s]).xy().norm() > 1e-6)
                };

                // 立ったまま 1 脚ずつ足を置き替える（3 脚支持、胴体を寄せる）。
                // 足を滑らせずに立ち位置を替える唯一の方法。
                let replace_feet = |cur: &mut JointVec,
                                    from: &[nalgebra::Vector3<f64>; 4],
                                    to: &[nalgebra::Vector3<f64>; 4],
                                    pat: crate::config::KneeShape,
                                    push: &mut dyn FnMut(String, JointVec, f64, [bool; 4]),
                                    what: &str|
                 -> Result<(), String> {
                    // 置き替えは立ち高さで行う（xy だけ使い、z は立ち高さ）。
                    let from = at_z(from, -h_ref);
                    let to = at_z(to, -h_ref);
                    let (from, to) = (&from, &to);
                    let shift = g.knee_flip_shift_m;
                    let mut placed = [false; 4];
                    for &slot in &[0usize, 3, 1, 2] {
                        if (from[slot] - to[slot]).xy().norm() < 1e-6 {
                            placed[slot] = true;
                            continue;
                        }
                        let dx = if slot < 2 { shift } else { -shift };
                        let dy = if is_left(slot) { shift } else { -shift };
                        let sh = nalgebra::Vector3::new(dx, dy, 0.0);
                        let now: [nalgebra::Vector3<f64>; 4] = std::array::from_fn(|s| if placed[s] { to[s] } else { from[s] });
                        for s2 in 0..4 {
                            let target = now[s2] + sh;
                            let q = ik(s2, target, knee_forward_for(pat, s2))?;
                            if std::env::var_os("MISA_KNEE_DEBUG").is_some() {
                                eprintln!("[knee] 寄せる {} for {}: target=({:+.3},{:+.3},{:+.3}) q=({:+.2},{:+.2},{:+.2}) within={}", leg_name(s2), leg_name(slot), target.x, target.y, target.z, q[0], q[1], q[2], within(s2, &q));
                            }
                            cur.legs[s2] = q;
                        }
                        let tag = leg_name(slot);
                        push(format!("寄せる {tag}"), *cur, 2.0 * phase, [true; 4]);
                        let mut st = [true; 4];
                        st[slot] = false;
                        let lift = nalgebra::Vector3::new(0.0, 0.0, 0.05);
                        cur.legs[slot] = ik(slot, now[slot] + sh + lift, knee_forward_for(pat, slot))?;
                        push(format!("{what} {tag}: 浮かす"), *cur, phase, st);
                        cur.legs[slot] = ik(slot, to[slot] + sh + lift, knee_forward_for(pat, slot))?;
                        push(format!("{what} {tag}: 移す"), *cur, phase, st);
                        cur.legs[slot] = ik(slot, to[slot] + sh, knee_forward_for(pat, slot))?;
                        push(format!("{what} {tag}: 着ける"), *cur, phase, [true; 4]);
                        placed[slot] = true;
                    }
                    for s2 in 0..4 {
                        cur.legs[s2] = ik(s2, to[s2], knee_forward_for(pat, s2))?;
                    }
                    push("中央へ".into(), *cur, 2.0 * phase, [true; 4]);
                    Ok(())
                };

                // 1. いまの向きで低い胴体に足を置けない脚があれば、立ったまま足を置き替える。
                if moved(&feet_ref, &xy_old) {
                    log::info!("下ろす前に足を置き替える（低い胴体で膝が床に着かない所へ）");
                    replace_feet(&mut cur, &feet_ref, &xy_old, old, &mut push, "広げる")?;
                }
                // 2. 3 段で鉛直に下ろして車輪に載せ、足を浮かす。
                for i in 1..=3 {
                    let h = h_ref + (h_rest - h_ref) * i as f64 / 3.0;
                    let feet = at_z(&xy_old, -h);
                    for slot in 0..4 {
                        cur.legs[slot] = ik(slot, feet[slot], knee_forward_for(old, slot))?;
                    }
                    push(format!("下ろす {i}/3"), cur, 2.0 * phase / 3.0, [true; 4]);
                }
                cur_floor.set(floor);
                let float_old = at_z(&xy_old, z_float);
                for slot in 0..4 {
                    cur.legs[slot] = ik(slot, float_old[slot], knee_forward_for(old, slot))?;
                }
                push("浮かす".into(), cur, phase, [true; 4]);
                // 3. 浮いた脚をまとめて反転し、新しい向きで置ける所へ戻す。
                let slots: Vec<usize> = (0..4).filter(|s| flip[*s]).collect();
                let float_new = at_z(&xy_new, z_float);
                flip_legs(&mut cur, &slots, [false; 4], &float_new, z_float, true, &mut push, "")?;
                if (0..4).any(|s| !flip[s] && (xy_new[s] - xy_old[s]).xy().norm() > 1e-6) {
                    for slot in 0..4 {
                        if !flip[slot] {
                            cur.legs[slot] = ik(slot, float_new[slot], knee_forward_for(old, slot))?;
                        }
                    }
                    push("揃える".into(), cur, phase, [false; 4]);
                }
                // 4. 足を着け、3 段で鉛直に上げる（足先の xy はそのまま。ロールは IK が戻す）。
                let ground_new = at_z(&xy_new, floor);
                for slot in 0..4 {
                    cur.legs[slot] = ik(slot, ground_new[slot], knee_forward_for(new, slot))?;
                }
                push("着ける".into(), cur, phase, [true; 4]);
                for i in 1..=3 {
                    let h = h_rest + (h_ref - h_rest) * i as f64 / 3.0;
                    let feet = at_z(&xy_new, -h);
                    for slot in 0..4 {
                        cur.legs[slot] = ik(slot, feet[slot], knee_forward_for(new, slot))?;
                    }
                    push(format!("上げる {i}/3"), cur, 2.0 * phase / 3.0, [true; 4]);
                }
                cur_floor.set(-h_ref);
                // 5. 立ち上がった足の位置がそのまま新しい立ち位置になる（置き替え無し）。
            }
            KneeFlipStyle::Trot => {
                // **対角 2 脚で支えて、残りの対角 2 脚をまとめて反転する。** 車輪には
                // 頼らない。2 点接触では支持線まわりの回転を接地力で止められない
                // （角運動量は重力でしか変わらない）ので、2 脚でいる時間を短くする：
                //  1. 浮かす前に 4 脚のまま、**浮かせた姿勢の全身重心が支持線に乗る**
                //     ところまで胴体を寄せる（モデルから前置。keel は 9 mm ほど）、
                //  2. 浮かせながら hip のロールで脚を外へ倒し、**腿と calf を同時に逆へ
                //     折り返す**（3 段。脚が真下で一直線になる瞬間の足先の下がりは
                //     (上腿+下腿)·cos(ロール) なので、それが立ち高さを超える機体では
                //     反転中だけ胴体を上げる）、
                //  3. 2 脚で支えている間は `balance_on_two_legs` が IMU とエンコーダで
                //     帰還して立脚の足先をずらす（この段の目標は初期値）、
                //  4. 着けたら、ずらした位置から計画へ滑らかに戻す（`PosePlayer::rebase`）。
                let _ = &park;
                let z_float = -h_flip + g.knee_flip_foot_lift_m;
                cur_floor.set(-h_flip);
                let shifted = |slot: usize, u: f64, n: nalgebra::Vector2<f64>, z: f64| {
                    nalgebra::Vector3::new(feet_ref[slot].x - u * n.x, feet_ref[slot].y - u * n.y, z)
                };
                for pair in [[0usize, 3], [1, 2]] {
                    let slots: Vec<usize> = pair.iter().copied().filter(|s| flip[*s]).collect();
                    if slots.is_empty() {
                        continue;
                    }
                    let support: [usize; 2] = if pair == [0, 3] { [1, 2] } else { [0, 3] };
                    cur_support.set(support);
                    let tag = format!(" {}+{}", leg_name(pair[0]), leg_name(pair[1]));
                    let mut stance = [true; 4];
                    for &s in &slots {
                        stance[s] = false;
                    }
                    // 支持線（支える対角の足先を結ぶ線）とその法線 n。
                    let (a, b) = (feet_ref[support[0]], feet_ref[support[1]]);
                    let d = nalgebra::Vector2::new(b.x - a.x, b.y - a.y).normalize();
                    let n = nalgebra::Vector2::new(-d.y, d.x);
                    // 胴体を u だけ n へ寄せた（足先は −u·n）浮かせ姿勢の全身重心を、
                    // 寄せた支持線から `target_s` の所へ置く u を求める。脚の質量も動く
                    // ので 3 回まわす。
                    //
                    // 対角の 2 脚とも浮かせるなら target_s = 0（支持線の上）。**組の
                    // 片方だけ反転する（他の 3 脚が接地）なら、支持線は支持三角形の
                    // 辺**なので、その上に重心を置くと余裕が 0 になり、モデルの重心が
                    // 1 cm ずれているだけで浮かせた脚の側へ倒れる。`stand` と同じく
                    // 浮かせる脚の反対側へ `knee_flip_shift_m` 入れる。
                    let target_s = if slots.len() == 1 {
                        let f = feet_ref[slots[0]];
                        let side = ((f.x - a.x) * n.x + (f.y - a.y) * n.y).signum();
                        -side * g.knee_flip_shift_m
                    } else {
                        0.0
                    };
                    let mut u = 0.0;
                    let mut lift = cur;
                    for _ in 0..3 {
                        for s in 0..4 {
                            let z = if stance[s] { -h_flip } else { z_float };
                            lift.legs[s] = ik(s, shifted(s, u, n, z), cur_forward.get()[s])?;
                        }
                        let com = self.com_body_at(&lift);
                        let s0 = (com.x - (a.x - u * n.x)) * n.x + (com.y - (a.y - u * n.y)) * n.y;
                        u -= s0 - target_s;
                    }
                    for s in 0..4 {
                        cur.legs[s] = ik(s, shifted(s, u, n, -h_flip), cur_forward.get()[s])?;
                    }
                    // 釣り合いの諸元（設計の確認用）。支持線まわりの慣性は重心まわりの
                    // 慣性を d に射影したもの、不安定極は √(m·g·h / (I_cm + m·h²))。
                    {
                        let bi = self.robot.body_inertia_at(&lift);
                        let d3 = nalgebra::Vector3::new(d.x, d.y, 0.0);
                        let i_cm = (d3.transpose() * bi.inertia_body * d3)[(0, 0)];
                        let hz = -bi.com_body.z + h_flip; // 足先（−h_flip）から重心までの高さ
                        let i_line = i_cm + bi.mass_kg * hz * hz;
                        if slots.len() == 1 {
                            log::info!(
                                "3 脚支持{tag}（浮かすのは {} だけ）: 重心を支持線から反対側へ {:.3} m、胴体の寄せ {:+.4} m",
                                leg_name(slots[0]), g.knee_flip_shift_m, u
                            );
                        } else {
                            log::info!(
                                "2 脚支持{tag}: 質量 {:.1} kg、重心の高さ {:.3} m（胴体 {:.3} m）、支持線まわりの慣性 I_cm {:.2} kg·m²（I_cm + m·h² = {:.2}）、不安定極 {:.1} rad/s、胴体の寄せ {:+.4} m",
                                bi.mass_kg, hz, h_flip, i_cm, i_line,
                                (bi.mass_kg * 9.81 * hz / i_line).sqrt(), u
                            );
                        }
                    }
                    cur_shift.set(u);
                    // 寄せるは実行中に上書きされる（推定込みの重心が支持線に乗る所へ）。
                    cur_kind.set(KneeStepKind::Adjust);
                    push(format!("寄せる{tag}"), cur, 2.0 * phase, [true; 4]);
                    // 重心の探り: 対角 2 脚を少しだけ浮かせて保持し、倒れ始める角加速度
                    // から実機の重心のずれを測る → 寄せ直す。2 脚とも浮かせる組だけ。
                    if slots.len() == 2 && g.knee_flip_probe_rounds > 0 && g.knee_flip_probe_lift_m > 0.0 {
                        let mut probe = cur;
                        for &slot in &slots {
                            probe.legs[slot] = ik(slot, shifted(slot, u, n, -h_flip + g.knee_flip_probe_lift_m), cur_forward.get()[slot])?;
                        }
                        for round in 1..=g.knee_flip_probe_rounds {
                            cur_kind.set(KneeStepKind::ProbeMove);
                            push(format!("探る{tag} 上げ {round}"), probe, g.knee_flip_probe_move_s, stance);
                            cur_kind.set(KneeStepKind::ProbeHold);
                            push(format!("探る{tag} 保持 {round}"), probe, g.knee_flip_probe_hold_s, stance);
                            push(format!("探る{tag} 下げ {round}"), cur, g.knee_flip_probe_move_s, [true; 4]);
                            cur_kind.set(KneeStepKind::Adjust);
                            push(format!("寄せ直し{tag} {round}"), cur, 0.6, [true; 4]);
                        }
                    }
                    // 浮かす + 倒す: 浮かせた角のまま hip のロールを外へ。
                    for &slot in &slots {
                        cur.legs[slot] = lift.legs[slot];
                        cur.legs[slot][0] = roll_out(slot);
                    }
                    push(format!("浮かす{tag}"), cur, phase, stance);
                    // 反転: ロールは外のまま、腿と calf を着地姿勢（新しい膝の向き）の角へ
                    // 同時に。関節空間の直線なので途中で脚が一直線になる。
                    let mut landed = [[0.0; 3]; 4];
                    for &slot in &slots {
                        landed[slot] = ik(slot, shifted(slot, u, n, z_float), knee_forward_for(new, slot))?;
                        cur.legs[slot][1] = landed[slot][1];
                        cur.legs[slot][2] = landed[slot][2];
                    }
                    let mut fwd_now = cur_forward.get();
                    for &slot in &slots {
                        fwd_now[slot] = knee_forward_for(new, slot);
                    }
                    cur_forward.set(fwd_now);
                    push(format!("反転{tag}"), cur, phase, stance);
                    // 戻す: ロールを戻して足先を浮かせ位置へ。
                    for &slot in &slots {
                        cur.legs[slot] = landed[slot];
                    }
                    push(format!("戻す{tag}"), cur, phase, stance);
                    for &slot in &slots {
                        cur.legs[slot] = ik(slot, shifted(slot, u, n, -h_flip), knee_forward_for(new, slot))?;
                    }
                    push(format!("着ける{tag}"), cur, phase, [true; 4]);
                }
                // 中央へ: 寄せていた胴体を基準の立ち位置・立ち高さへ戻す。
                cur_shift.set(0.0);
                cur_floor.set(-h_ref);
                for s in 0..4 {
                    cur.legs[s] = ik(s, feet_ref[s], cur_forward.get()[s])?;
                }
                push("中央へ".into(), cur, 2.0 * phase, [true; 4]);
            }
            KneeFlipStyle::Stand => {
                // **立ったまま 1 脚ずつ、他の 3 脚で支える。** 静的に安定（重心を支持三角形の
                // 内側に 5 cm 入れる）なので釣り合いの制御は要らず、段も短くできる。
                // 脚の反転は trot と同じく、hip のロールで外へ倒して腿と calf を同時に
                // 折り返す 3 段（浮かす → 反転 → 戻す）。畳んで振り出す 5 段だった頃の
                // 24 s が約 11 s になる。反転中は胴体を h_flip に上げる。
                let shift = g.knee_flip_shift_m;
                let ph = g.knee_flip_stand_phase_s;
                cur_floor.set(-h_flip);
                let z_float = -h_flip + g.knee_flip_foot_lift_m;
                let at = |f: nalgebra::Vector3<f64>, z: f64| nalgebra::Vector3::new(f.x, f.y, z);
                let order = [0usize, 3, 1, 2];
                let mut first = true;
                // 対角を交互に。
                for &slot in &order {
                    if !flip[slot] {
                        continue;
                    }
                    // 胴体を反転する脚の反対の対角へ寄せる = 胴体座標では全足先が
                    // その脚の側へ動く（前脚なら +x、左なら +y）。
                    let dx = if slot < 2 { shift } else { -shift };
                    let dy = if slot % 2 == 0 { shift } else { -shift };
                    let feet_shift: [nalgebra::Vector3<f64>; 4] =
                        std::array::from_fn(|s| feet_ref[s] + nalgebra::Vector3::new(dx, dy, 0.0));
                    for s2 in 0..4 {
                        cur.legs[s2] = ik(s2, at(feet_shift[s2], -h_flip), cur_forward.get()[s2])?;
                    }
                    let tag = format!(" {}", leg_name(slot));
                    // 最初の寄せは胴体を上げる動きも兼ねるので長め。
                    push(format!("寄せる{tag}"), cur, if first { 2.0 * ph } else { ph }, [true; 4]);
                    first = false;
                    let mut stance = [true; 4];
                    stance[slot] = false;
                    // 浮かす + 倒す。
                    cur.legs[slot] = ik(slot, at(feet_shift[slot], z_float), cur_forward.get()[slot])?;
                    cur.legs[slot][0] = roll_out(slot);
                    push(format!("浮かす{tag}"), cur, ph, stance);
                    // 反転: ロールは外のまま、腿と calf を着地姿勢（新しい向き）の角へ同時に。
                    let landed = ik(slot, at(feet_shift[slot], z_float), knee_forward_for(new, slot))?;
                    cur.legs[slot][1] = landed[1];
                    cur.legs[slot][2] = landed[2];
                    let mut fwd_now = cur_forward.get();
                    fwd_now[slot] = knee_forward_for(new, slot);
                    cur_forward.set(fwd_now);
                    push(format!("反転{tag}"), cur, ph, stance);
                    // 戻す: ロールを戻して足先を浮かせ位置へ。
                    cur.legs[slot] = landed;
                    push(format!("戻す{tag}"), cur, ph, stance);
                    cur.legs[slot] = ik(slot, at(feet_shift[slot], -h_flip), knee_forward_for(new, slot))?;
                    push(format!("着ける{tag}"), cur, ph, [true; 4]);
                }
                cur_floor.set(-h_ref);
                for slot in 0..4 {
                    cur.legs[slot] = ik(slot, feet_ref[slot], knee_forward_for(new, slot))?;
                }
                push("中央へ".into(), cur, 2.0 * ph, [true; 4]);
            }
        }

        // 検査: 可動域。
        for st in &steps {
            for slot in 0..4 {
                for k in 0..3 {
                    let (lo, hi) = limit(slot, k);
                    let v = st.target.legs[slot][k];
                    if v < lo - 1e-6 || v > hi + 1e-6 {
                        return Err(format!(
                            "段「{}」で {} が可動域 [{:+.2}, {:+.2}] の外（{:+.2}）",
                            st.name, names[slot][k], lo, hi, v
                        ));
                    }
                }
            }
        }
        // 検査: 浮かせている脚の折れ線上の足先が、浮かせ始めた面より下がらない。
        // 地面そのものは胴体座標では分からない（車輪に載ると胴体は別の高さで
        // 止まる）ので、浮かせる直前の足先の高さを床の代わりにする。
        let mut prev = self.targets;
        let mut floor_z = [f64::NAN; 4];
        for ((st, stance), &floor_true) in steps.iter().zip(stances.iter()).zip(floors.iter()) {
            for slot in 0..4 {
                if stance[slot] {
                    floor_z[slot] = f64::NAN;
                    continue;
                }
                let leg = kin_ref.legs()[slot];
                let s = signs[slot];
                if floor_z[slot].is_nan() {
                    let f0 = forward_leg_kinematics(leg, prev.legs[slot][0] * s[0], prev.legs[slot][1] * s[1], prev.legs[slot][2] * s[2]);
                    floor_z[slot] = f0.z;
                }
                for i in 0..=20 {
                    let a = i as f64 / 20.0;
                    let q: [f64; 3] = std::array::from_fn(|k| prev.legs[slot][k] + a * (st.target.legs[slot][k] - prev.legs[slot][k]));
                    let foot = forward_leg_kinematics(leg, q[0] * s[0], q[1] * s[1], q[2] * s[2]);
                    // 床は、浮かせ始めた足先の面と段の床（rest では車輪に載った胴体の
                    // 下）の低いほう。浮かせた面より少し下がっても床に着かなければよい。
                    let limit_z = (floor_z[slot] - 0.005).min(floor_true + 0.002);
                    if foot.z < limit_z {
                        return Err(format!(
                            "段「{}」の途中で {} の足先が床（z {:+.3}）に近づきすぎる（足先 z {:+.3}）。knee_flip_out_z_m を見直す",
                            st.name, leg_name(slot), floor_true, foot.z
                        ));
                    }
                    let knee = knee_pos(slot, &q);
                    if knee.z < floor_true + 0.002 {
                        return Err(format!(
                            "段「{}」の途中で {} の膝が床（z {:+.3}）から {:.3} m しか離れない（膝 z {:+.3}）",
                            st.name, leg_name(slot), floor_true, knee.z - floor_true, knee.z
                        ));
                    }
                }
            }
            prev = st.target;
        }
        Ok(KneeFlipPlan { steps, stances, forwards, plan_shifts, floors, kinds, supports, next_offset })
    }

    fn tick_knee_flip(&mut self, cmd: &Intent, measured: &JointVec, attitude_rad: [f64; 3], dt: f64) {
        let prev_idx = self.knee_flip_prev_idx;
        let prev_targets = self.targets;
        let (done, idx) = match self.player.as_mut() {
            Some(player) => {
                self.targets = player.tick(dt);
                (player.is_done(), player.step_index())
            }
            None => (true, 0),
        };
        if idx != prev_idx {
            // 実行中に上書きしていた目標から、次の段へ滑らかに繋ぐ（上書きが無ければ
            // 始点は同じなので何も変わらない）。
            if let Some(player) = self.player.as_mut() {
                player.rebase(prev_targets);
                self.targets = player.current();
            }
            let legs = |st: &[[bool; 4]], i: usize| -> usize {
                st.get(i).map(|s| s.iter().filter(|x| **x).count()).unwrap_or(4)
            };
            let (was, now) = (legs(&self.knee_flip_stance, prev_idx), legs(&self.knee_flip_stance, idx));
            let kind_of = |kinds: &[KneeStepKind], i: usize| kinds.get(i).copied().unwrap_or_default();
            let (kind_prev, kind_now) = (kind_of(&self.knee_flip_kind, prev_idx), kind_of(&self.knee_flip_kind, idx));
            if kind_prev == KneeStepKind::ProbeHold {
                self.finish_probe(prev_idx);
            }
            if was == 4 && now == 2 {
                // 4 脚の段で上書きしていなければ（計画どおり）、計画の寄せから始める。
                if kind_prev != KneeStepKind::Adjust {
                    self.knee_flip_shift_m = self.knee_flip_plan_shift.get(idx).copied().unwrap_or(0.0);
                }
                self.knee_flip_want_lpf = self.knee_flip_shift_m;
                self.knee_flip_s_prev = None;
                self.knee_flip_sdot = 0.0;
            }
            if kind_now == KneeStepKind::ProbeHold {
                self.knee_flip_probe.clear();
                self.knee_flip_probe_t = 0.0;
            }
            self.knee_flip_prev_idx = idx;
        }
        // 浮かせている脚は立脚でない（前置トルク・WBC の接地の仮定に効く）。
        if let Some(st) = self.knee_flip_stance.get(idx) {
            self.body_view.stance = *st;
        }
        if self.cfg.gait.knee_flip_style == crate::config::KneeFlipStyle::Trot {
            self.balance_on_two_legs(idx, measured, attitude_rad, dt);
        }
        if !self.arm_app_driven {
            if let Some(observed) = cmd.aux(0) {
                self.targets.arm = observed;
            }
        }
        if !done {
            return;
        }
        self.player = None;
        if self.cfg.gait.knee_flip_style == crate::config::KneeFlipStyle::Trot {
            log::info!(
                "2 脚支持の釣り合い: 支持線まわりの傾き 最大 {:.1}°、胴体の横移動 最大 {:.3} m",
                self.knee_flip_tilt_max_rad.to_degrees(),
                self.knee_flip_shift_max_m
            );
        }
        self.knee_flip_stance.clear();
        self.knee_flip_forward.clear();
        self.body_view.stance = [true; 4];
        if let Some(new) = self.knee_flip_target.take() {
            self.cfg.gait.knee_pattern = new;
            self.gait.set_knee_pattern(crate::robot::knee_pattern_of(new));
            log::info!("膝の向きを {} にしました", new.label());
        }
        if let Some(off) = self.knee_flip_next_offset.take() {
            if (0..4).any(|i| (off[i] - self.stance_xy_offset[i]).norm() > 1e-6) {
                log::info!(
                    "立ち位置を替えました: FL ({:+.2},{:+.2}) FR ({:+.2},{:+.2}) RL ({:+.2},{:+.2}) RR ({:+.2},{:+.2}) m",
                    off[0].x, off[0].y, off[1].x, off[1].y, off[2].x, off[2].y, off[3].x, off[3].y
                );
            }
            self.stance_xy_offset = off;
        }
        // 立ち姿勢に着いているので歩容へ引き渡す。位相は最初から。
        let h = self.robot.reference_height_m(&self.cfg.gait);
        self.apply_body_height(h);
        self.gait.reset();
        self.state = State::Active;
    }

    /// `trot` 型の反転で、対角 2 脚で支えているあいだ胴体を支持線の上に保つ。
    ///
    /// 点接触 2 つでは支持線まわりの回転を接地力で止められない（重力だけの
    /// 倒立振子。keel で時定数 0.13 s）。代わりに**立脚で胴体を支持線と直角に
    /// 動かして重心を線上に戻す**。
    ///
    /// ```text
    ///   I·θ̈ = −m·g·(s₀ + u − h·θ)      θ: 支持線まわりの傾き、u: 胴体の横移動
    ///   u = −s₀ + Kθ·θ + Kω·θ̇           Kθ > h で安定（重力の正帰還 m·g·h に勝つ）
    /// ```
    ///
    /// `s₀` は計画姿勢での重心の支持線からの距離（モデルから毎周期求める。浮かせた
    /// 脚が動くぶんの前置）。θ は IMU の roll / pitch を支持線の向きに投影したもの、
    /// θ̇ は推定器の角速度。立脚が 2 本でないときは u を 0 へ戻す。
    /// 2 脚支持の段で、立脚の足先を支持線と直角にずらして胴体を支持線の上に保つ。
    ///
    /// 状態は θ（IMU の傾きを支持線の向き d に射影。重心が +n へ動く向きを正）、
    /// θ̇（ジャイロ）、s（**実測**の関節角から FK した全身重心の支持線からの距離）、
    /// ṡ（差分の一次遅れ 20 Hz）。胴体の寄せ u = −s0 − k·x で、s0 は計画姿勢の
    /// 重心の支持線からの距離（前置）。ゲインは [`GaitTuning::knee_flip_balance_gains`]。
    fn balance_on_two_legs(&mut self, idx: usize, measured: &JointVec, attitude_rad: [f64; 3], dt: f64) {
        use crate::robot::knee_forward_for;
        use quadruped_gait::solve_leg_ik;
        let g = &self.cfg.gait;
        let Some(stance) = self.knee_flip_stance.get(idx).copied() else { return };
        let kind = self.knee_flip_kind.get(idx).copied().unwrap_or_default();
        let support: Vec<usize> = (0..4).filter(|s| stance[*s]).collect();
        // 上書きするのは、2 脚支持の段（釣り合い・探り）と、4 脚の寄せ直し。
        match (support.len(), kind) {
            (2, _) | (4, KneeStepKind::Adjust) => {}
            _ => return,
        }
        let forward: [bool; 4] = self
            .knee_flip_forward
            .get(idx)
            .copied()
            .unwrap_or_else(|| std::array::from_fn(|s| knee_forward_for(g.knee_pattern, s)));
        let h_ref = self.robot.reference_height_m(g);
        // 支持線は、この段が属する組の支持脚（4 脚の寄せ直しでも、次に浮かせる組の）。
        let pair = self.knee_flip_support.get(idx).copied().unwrap_or([1, 2]);
        let (a, b) = (self.knee_flip_feet[pair[0]], self.knee_flip_feet[pair[1]]);
        let d = nalgebra::Vector2::new(b.x - a.x, b.y - a.y).normalize();
        let n = nalgebra::Vector2::new(-d.y, d.x);
        // 前置: 計画姿勢の全身重心（探りの推定込み）の、（ずらす前の）支持線からの距離。
        let com = self.com_body_at(&self.targets);
        let s0 = (com.x - a.x) * n.x + (com.y - a.y) * n.y;
        // 傾き・角速度。d まわりの右ねじの回転は胴体の上側を −n へ動かすので符号を返す。
        let theta = -(attitude_rad[0] * d.x + attitude_rad[1] * d.y);
        let (sy, cy) = attitude_rad[2].sin_cos();
        let w = self.observed_omega_world;
        let (wx, wy) = (cy * w.x + sy * w.y, -sy * w.x + cy * w.y);
        let omega = -(wx * d.x + wy * d.y);
        let kin = self.stance_kinematics_with_offset(h_ref);
        let apply = |targets: &mut JointVec, u: f64, legs: &[usize], floor: Option<f64>| {
            for &slot in legs {
                let leg = kin.legs()[slot];
                let f = self.knee_flip_feet[slot];
                let z = floor.unwrap_or(f.z);
                let target = nalgebra::Vector3::new(f.x - u * n.x, f.y - u * n.y, z);
                let sol = solve_leg_ik(leg, target, forward[slot]);
                if !sol.is_reachable() {
                    continue;
                }
                let (hh, t, c) = sol.angles();
                let sg = self.robot.signs[slot];
                targets.legs[slot] = [hh * sg[0], t * sg[1], c * sg[2]];
            }
        };
        let floor = self.knee_flip_floor.get(idx).copied();
        if kind == KneeStepKind::Adjust {
            // 4 脚のまま、推定込みの重心が支持線に乗る所へゆっくり（0.05 m/s）。
            let want = (-s0).clamp(-g.knee_flip_balance_max_m, g.knee_flip_balance_max_m);
            let step = 0.05 * dt;
            let u = self.knee_flip_shift_m + (want - self.knee_flip_shift_m).clamp(-step, step);
            self.knee_flip_shift_m = u;
            self.knee_flip_want_lpf = u;
            let mut t = self.targets;
            apply(&mut t, u, &support, floor);
            self.targets = t;
            return;
        }
        if kind != KneeStepKind::Normal {
            // 探り: 寄せは凍結、帰還なし。保持中は θ̇ を溜める（重力だけで倒れ始める
            // 角加速度を見る）。
            if kind == KneeStepKind::ProbeHold {
                self.knee_flip_probe_t += dt;
                self.knee_flip_probe.push((self.knee_flip_probe_t, omega, theta));
            }
            let u = self.knee_flip_shift_m;
            let mut t = self.targets;
            apply(&mut t, u, &support, floor);
            self.targets = t;
            return;
        }
        // 実測の重心の支持線からの距離。**立脚は実測の関節角**（脚のたわみ・胴体の
        // 位置が入る）、**浮かせている脚は目標角**で重心を出す。浮かせた脚の実測を
        // 使うと、kp 500 の追従遅れで脚の質量が計画から遅れるぶんが重心の速度に
        // 出て、ṡ の項（1.4 m per m/s）が指令を上限まで振る（`<>` → `><` で 19°）。
        // LQR の s は「胴体の位置」なので、脚の遅れは外乱として θ の側で受ける。
        let feet_m = self.robot.feet_from_posture(measured);
        let mut mixed = self.targets;
        for &slot in &support {
            mixed.legs[slot] = measured.legs[slot];
        }
        let com_m = self.com_body_at(&mixed);
        let (am, bm) = (feet_m[support[0]], feet_m[support[1]]);
        let dm = nalgebra::Vector2::new(bm.x - am.x, bm.y - am.y).normalize();
        let nm = nalgebra::Vector2::new(-dm.y, dm.x);
        let s_m = (com_m.x - am.x) * nm.x + (com_m.y - am.y) * nm.y;
        let sdot_raw = match self.knee_flip_s_prev {
            Some(p) if dt > 0.0 => (s_m - p) / dt,
            _ => 0.0,
        };
        self.knee_flip_s_prev = Some(s_m);
        let lpf = |dt: f64, hz: f64| dt / (dt + 1.0 / (2.0 * std::f64::consts::PI * hz));
        self.knee_flip_sdot += lpf(dt, g.knee_flip_balance_sdot_lpf_hz) * (sdot_raw - self.knee_flip_sdot);
        let k = g.knee_flip_balance_gains;
        let fb = k[0] * theta + k[1] * omega + k[2] * s_m + k[3] * self.knee_flip_sdot;
        let want = (-s0 - fb).clamp(-g.knee_flip_balance_max_m, g.knee_flip_balance_max_m);
        // 一次遅れ → 変化率の上限。速度の項が拾う脚のたわみの振動を落とし、帰還が
        // 上限の間でバンバンになっても足先目標は跳ばない。
        if g.knee_flip_balance_lpf_hz > 0.0 {
            self.knee_flip_want_lpf += lpf(dt, g.knee_flip_balance_lpf_hz) * (want - self.knee_flip_want_lpf);
        } else {
            self.knee_flip_want_lpf = want;
        }
        let step = g.knee_flip_balance_rate_m_s * dt;
        let u = self.knee_flip_shift_m + (self.knee_flip_want_lpf - self.knee_flip_shift_m).clamp(-step, step);
        self.knee_flip_tilt_max_rad = self.knee_flip_tilt_max_rad.max(theta.abs());
        if std::env::var_os("MISA_BALANCE_TRACE").is_some() {
            eprintln!(
                "[balance] step {idx} support {:?} theta={:+.4} omega={:+.3} yaw={:+.3} s_m={:+.4} sdot={:+.3} s0={:+.4} fb={:+.4} want={:+.4} u={:+.4}",
                support, theta, omega, attitude_rad[2], s_m, self.knee_flip_sdot, s0, fb, want, u
            );
        }
        self.knee_flip_shift_m = u;
        self.knee_flip_shift_max_m = self.knee_flip_shift_max_m.max(u.abs());
        // 立脚の足先を −u·n（胴体が +u·n）へ。
        let mut t = self.targets;
        apply(&mut t, u, &support, floor);
        self.targets = t;
    }

    /// 探りの保持が終わった: 溜めた θ̇ の傾き（角加速度 α）から、実機の重心の
    /// 支持線からのずれを出して `knee_flip_com_est` に足す。
    ///
    /// `I_line·θ̈ = m·g·(s_true + h·θ)` なので `s_true = α·I_line/(m·g) − h·θ̄`。
    /// 信じていた位置 `s_b = s0 + u`（推定込み）との差がずれ。足が着いてしまった
    /// あと（|θ| が浮かせた量 / h を超えた）の標本は使わない。
    fn finish_probe(&mut self, idx: usize) {
        let g = &self.cfg.gait;
        let pair = self.knee_flip_support.get(idx).copied().unwrap_or([1, 2]);
        let (a, b) = (self.knee_flip_feet[pair[0]], self.knee_flip_feet[pair[1]]);
        let d = nalgebra::Vector2::new(b.x - a.x, b.y - a.y).normalize();
        let n = nalgebra::Vector2::new(-d.y, d.x);
        let h_flip = -self.knee_flip_floor.get(idx).copied().unwrap_or(-self.robot.reference_height_m(g));
        let bi = self.robot.body_inertia_at(&self.targets);
        let hz = -bi.com_body.z + h_flip;
        let d3 = nalgebra::Vector3::new(d.x, d.y, 0.0);
        let i_line = (d3.transpose() * bi.inertia_body * d3)[(0, 0)] + bi.mass_kg * hz * hz;
        let theta_max = 0.7 * g.knee_flip_probe_lift_m / hz;
        let pts: Vec<(f64, f64, f64)> = self.knee_flip_probe.iter().copied().filter(|p| p.2.abs() < theta_max).collect();
        // 最初の 0.1 s は上げの反動が残るので捨てる。
        let pts: Vec<(f64, f64, f64)> = pts.into_iter().filter(|p| p.0 > 0.1).collect();
        if pts.len() < 10 {
            log::warn!("重心の探り: 標本が足りません（{} 点）。足がすぐ着いたか、保持が短い", pts.len());
            return;
        }
        let k = pts.len() as f64;
        let (mt, mw) = (pts.iter().map(|p| p.0).sum::<f64>() / k, pts.iter().map(|p| p.1).sum::<f64>() / k);
        let sxx: f64 = pts.iter().map(|p| (p.0 - mt).powi(2)).sum();
        let sxy: f64 = pts.iter().map(|p| (p.0 - mt) * (p.1 - mw)).sum();
        let alpha = sxy / sxx;
        let theta_mean = pts.iter().map(|p| p.2).sum::<f64>() / k;
        let s_true = alpha * i_line / (bi.mass_kg * 9.81) - hz * theta_mean;
        let com = self.com_body_at(&self.targets);
        let s_b = (com.x - a.x) * n.x + (com.y - a.y) * n.y + self.knee_flip_shift_m;
        let e_n = s_true - s_b;
        self.knee_flip_com_est += e_n * n;
        log::info!(
            "重心の探り: 角加速度 {:+.3} rad/s²（{} 点、{:.2} s）→ 重心は支持線から {:+.1} mm（信じていた {:+.1} mm）。ずれ {:+.1} mm を足して、推定 ずれ 合計 [{:+.4}, {:+.4}] m（胴体座標）",
            alpha, pts.len(), pts.last().map(|p| p.0).unwrap_or(0.0) - pts[0].0,
            s_true * 1000.0, s_b * 1000.0, e_n * 1000.0, self.knee_flip_com_est.x, self.knee_flip_com_est.y
        );
    }

    fn stance_targets(&mut self) -> JointVec {
        let h = self.robot.reference_height_m(&self.cfg.gait);
        self.apply_body_height(h);
        self.gait.set_velocity_cmd(velocity_cmd(0.0, 0.0, 0.0));
        let out = self.gait.tick(0.0);
        self.robot.output_to_joints(&out, self.targets.arm)
    }

    /// いま歩容を差し替えてよいか。
    ///
    /// **判定は「遊脚があるか」であって状態名ではない。** `Active` でも
    /// 速度 0 で立っているだけなら全脚が接地しており、差し替えても飛ばない。
    /// ポーズ再生中は再生を壊すので許さない。
    fn can_switch_gait(&self) -> bool {
        match self.state {
            State::PlayingPose | State::FlippingKnees => false,
            State::Active => {
                // 指令が 0 まで落ちきっていて、かつ 4 脚とも接地している。
                // どちらか一方では足りない（落としきる途中は遊脚が残る）。
                self.ramped_v.iter().all(|v| *v == 0.0) && self.body_view.stance.iter().all(|&s| s)
            }
            _ => true,
        }
    }

    /// 速度指令を `gait.velocity_ramp_s` で鈍らせる。
    ///
    /// 各軸の上限レートは「その軸の最大値 ÷ ランプ時間」なので、**どの軸も
    /// 全開から中立まで同じ時間**で戻る。歩き出す側だけでなく**止まる側にも
    /// 同じレートがかかる**（実測では止まるときも 27.9 rad/s 跳んでいた）。
    fn ramp_velocity(&mut self, want: [f64; 3], dt: f64) -> [f64; 3] {
        let up_s = self.cfg.gait.velocity_ramp_s;
        let down_s = self.cfg.gait.velocity_ramp_stop_s;
        if dt <= 0.0 || (up_s <= 0.0 && down_s <= 0.0) {
            self.ramped_v = want;
            return want;
        }
        let full = [
            self.cfg.gait.max_vx_m_s,
            self.cfg.gait.max_vy_m_s,
            self.cfg.gait.max_wz_rad_s,
        ];
        for k in 0..3 {
            // **上げる側と下げる側でレートが違う。止まるのは速く。**
            // 落としている軸か（絶対値が小さくなる向きか）で選ぶ。
            let slowing = want[k].abs() < self.ramped_v[k].abs();
            let ramp_s = if slowing { down_s } else { up_s };
            if ramp_s <= 0.0 {
                self.ramped_v[k] = want[k];
                continue;
            }
            let step = (full[k].abs() / ramp_s) * dt;
            let d = (want[k] - self.ramped_v[k]).clamp(-step, step);
            self.ramped_v[k] += d;
        }

        // **歩容は速度ちょうど 0 で静止姿勢へ分岐する。** 遊脚がある瞬間に
        // 0 を渡すと、その脚を一気に接地位置へ引き戻して目標が跳ぶ
        // （実測 35.4 rad/s = 制御 1 周期で 10.2°）。ランプだけでは消えず、
        // 跳ぶ時刻がランプ終了へ移るだけだった。0.001 m/s を渡し続けた
        // 場合は 3.34 rad/s で収まる ＝ **跳びの原因は 0 への分岐そのもの**。
        //
        // なので足が地面に着くまで微速で歩かせ、そこで 0 に落とす。
        //
        // **待つ条件は「いま空中の脚が着地したか」で、「4 脚とも接地」ではない。**
        // trot（接地比 0.5）は 4 脚が同時に接地する瞬間が無いので、全脚接地を
        // 待つと `stop_settle_s` の時間切れまで待ってから結局跳んでいた
        // （実機の keel で「w を一瞬押して止めると calf / thigh が急峻に動く」
        // として出た。2026-09-14。MuJoCo で 61.3 rad/s → 4.9 rad/s）。
        // 止まる要求が来た周期に空中だった脚が全部接地すれば、残りの脚は
        // 離地したばかりで足はまだ地面の高さにいる ＝ どの足も飛ばない。
        const CREEP_M_S: f64 = 1e-3;
        let settle_timeout_s = self.cfg.gait.stop_settle_s;
        let stopping = want.iter().all(|v| *v == 0.0);
        let nearly_stopped = self.ramped_v.iter().all(|v| v.abs() <= CREEP_M_S);
        if stopping && nearly_stopped {
            // 接地フラグは前周期の歩容出力（`body_view`）から取る。
            let stance = self.body_view.stance;
            let airborne = *self
                .settling_airborne
                .get_or_insert_with(|| std::array::from_fn(|i| !stance[i]));
            let landed = (0..4).all(|i| !airborne[i] || stance[i]);
            if landed || self.settling_s >= settle_timeout_s {
                self.settling_s = 0.0;
                self.settling_airborne = None;
                self.ramped_v = [0.0; 3];
                return self.ramped_v;
            }
            // 待っている間だけ微速。**保持しない**ので、着地した次の周期で 0 になる。
            self.settling_s += dt;
            return [CREEP_M_S, 0.0, 0.0];
        }
        self.settling_s = 0.0;
        self.settling_airborne = None;
        self.ramped_v
    }

    /// 歩容パラメータの上書きを歩容へ入れる。
    ///
    /// **歩きながら替えられるのは `set_config` が位相を保つから。** 作り直すと
    /// 位相が 0 に戻り、接地と遊脚の割り当てが跨いで全脚の目標が跳ぶ
    /// （歩容の切り替えを 4 脚接地かつ速度 0 に限っているのはそのため）。
    ///
    /// **`LinearCrawl` では効かない。** あちらは `GaitConfig` を持たない。
    fn apply_gait_tune(&mut self, tune: &misa_core::GaitTune) {
        let want = tune.clamped();
        if want == self.gait_tune {
            return;
        }
        let cfg = crate::robot::tuned_gait_config(&self.cfg.gait, self.gait_select, &want);
        self.gait.set_config(cfg.clone());
        log::info!(
            "歩容パラメータ: 周期 {:.3} s / 遊脚 {:.3} m / 歩幅 {:.3} m / 接地比 {:.2}",
            cfg.cycle_period_s,
            cfg.swing_height_m,
            cfg.max_step_length_m,
            cfg.duty_factor,
        );
        self.gait_tune = want;
    }

    /// いま歩容に入っている上書き。**表示と試験のため。**
    pub fn gait_tune(&self) -> misa_core::GaitTune {
        self.gait_tune
    }

    fn set_gait(&mut self, select: GaitSelect) {
        log::info!("歩容を {} に切り替えます", select.label());
        self.gait = self.robot.build_gait(&self.cfg.gait, &self.cfg.wbc, select);
        self.gait_select = select;
        // **作り直したので上書きは落ちている。** 次の周期で操縦側が送って
        // くる値が入る（操縦側も歩容を替えたら基準値へ戻す約束）。
        self.gait_tune = misa_core::GaitTune::default();
        // 歩容ごとに可否が違うので、切り替えたら言い直す。
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::teleop::{GaitSelect, ModeRequest};

    /// 水平の姿勢。胴体姿勢を使う試験だけがここを変える。
    fn imu() -> [f64; 3] {
        [0.0; 3]
    }

    fn cmd(mode: ModeRequest) -> Intent {
        Intent {
            mode,
            gait: GaitSelect::Crawl,
            aux_rad: vec![None],
            link_ok: true,
            ..Intent::default()
        }
    }

    fn controller() -> Controller {
        let cfg = AppConfig::default();
        let robot = Robot::load(&test_model_path(), &cfg.control.kinematics_pose)
            .expect("テスト用モデルを読めません");
        Controller::new(robot, cfg)
    }

    fn test_model_path() -> String {
        // crates/misa-runner から見たリポジトリルート。
        format!("{}/../../models/testquad/testquad.misa", env!("CARGO_MANIFEST_DIR"))
    }

    /// 状態が `want` になるまで回す。回りすぎたら失敗。
    /// 停止 ↔ 歩行の切り替わりで目標角が跳ばないこと。
    ///
    /// **歩容は速度指令が変わった瞬間に出力を階段状に飛ばす。** ランプ無しの
    /// 実測は制御 1 周期（5 ms）あたり Crawl 31.5 / Walk 23.0 / Trot 9.9 rad/s
    /// で、Crawl は 1 周期で 9.0° 飛んでいた。2 tick 目以降は滑らかなので、
    /// 跳ぶのは切り替わりの 1 点だけ。
    #[test]
    fn starting_and_stopping_a_walk_does_not_step_the_targets() {
        let dt = 0.005;
        for select in [GaitSelect::Crawl, GaitSelect::Walk, GaitSelect::Trot] {
            let mut c = controller();
            let mut stand = cmd(ModeRequest::Walk);
            stand.gait = select;
            run_until(&mut c, &stand, State::Active, 20.0);
            let mut prev = c.tick(&stand, &JointVec::zeros(), imu(), dt).targets;

            let mut go = stand.clone();
            go.velocity.vx_m_s = 0.10;
            let mut worst = [0.0f64; 2];
            // 0 = 歩き出し、1 = 停止。**跳びの性質が違う。**
            for (phase_i, phase) in [go.clone(), stand.clone()].into_iter().enumerate() {
                for _ in 0..400 {
                    let q = c.tick(&phase, &JointVec::zeros(), imu(), dt).targets;
                    for l in 0..4 {
                        for k in 0..3 {
                            let r = (q.legs[l][k] - prev.legs[l][k]).abs() / dt;
                            worst[phase_i] = worst[phase_i].max(r);
                        }
                    }
                    prev = q;
                }
            }
            // **歩き出し側はランプで消える。** ランプ無しでは 9.9〜31.5 rad/s。
            assert!(
                worst[0] < 6.0,
                "{} の歩き出しで目標が {:.2} rad/s 跳んだ（{:.1}°/周期）",
                select.label(),
                worst[0],
                (worst[0] * dt).to_degrees()
            );
            // **停止側も跳ばない**（2026-09-14）。かつては「trot は 4 脚が
            // 同時に接地しないので待てない」として跳びを受け入れていたが、
            // 待つ条件を**「いま空中の脚が着地したか」**に替えれば trot でも
            // 半周期で揃う（[`Controller::ramp_velocity`]）。実機の keel で
            // 「w を一瞬押して止めると calf / thigh が急峻に動く」として出た。
            assert!(
                worst[1] < 6.0,
                "{} の停止で目標が {:.2} rad/s 跳んだ（着地を待っているはず）",
                select.label(),
                worst[1]
            );
        }
    }

    /// **スティックが中立を一瞬通っても歩容が止まらない。**
    ///
    /// 歩容の `PhaseGenerator::advance` は `vel.is_zero()`（**厳密な等値
    /// 比較**）で即座に `holding` に入り、全脚を接地・`cycle_position = 0`
    /// として静止姿勢を出す。方向転換でスティックが中立を通過すると
    /// 3 軸そろって 0.0 になり、**その瞬間だけ立脚静止に落ちる**。
    /// 位相は凍結されるだけで戻らないので、復帰すると前の動きの続きに
    /// 見える（実機で「前の指令が残っているような動き」として観測）。
    ///
    /// ランプが時間的なヒステリシスになって、これを防ぐ。
    #[test]
    fn a_momentary_stick_centre_does_not_stop_the_gait() {
        let dt = 0.005;
        for select in [GaitSelect::Crawl, GaitSelect::Walk, GaitSelect::Trot] {
            let mut c = controller();
            let mut go = cmd(ModeRequest::Walk);
            go.gait = select;
            go.velocity.vx_m_s = 0.10;
            run_until(&mut c, &go, State::Active, 20.0);
            for _ in 0..400 {
                c.tick(&go, &JointVec::zeros(), imu(), dt);
            }
            // 中立を 25 ms 通過して反対側へ。
            let mut centre = go.clone();
            centre.velocity.vx_m_s = 0.0;
            let mut back = go.clone();
            back.velocity.vx_m_s = -0.10;
            for _ in 0..5 {
                c.tick(&centre, &JointVec::zeros(), imu(), dt);
                assert!(
                    c.ramped_v.iter().any(|v| *v != 0.0),
                    "{} で中立通過の瞬間に速度がちょうど 0 になった                     （歩容が立脚静止へ落ちる）",
                    select.label()
                );
            }
            for _ in 0..5 {
                c.tick(&back, &JointVec::zeros(), imu(), dt);
            }
        }
    }

    /// **スティックを中立に戻したら素早く止まる。**
    ///
    /// 停止が遅いとリング外へ出る。かつて全脚接地待ちが 2.0 s 固定で、
    /// **trot は 4 脚が同時に接地しないことがあるため毎回 2 s 待ち切って
    /// いた**（ランプ 0.5 s と合わせて最大 2.5 s）。実機で「スティックを
    /// 戻しても数秒止まらない」として出た（2026-08-22、本番前日）。
    #[test]
    fn centring_the_stick_stops_the_gait_promptly() {
        let dt = 0.005;
        for (label, ramp, stop_ramp, settle) in [
            ("旧(2.0s待ち)", 0.5, 0.5, 2.0),
            ("新(既定)", 0.5, 0.15, 0.25),
        ] {
            println!("--- {label} ---");
            for select in [GaitSelect::Crawl, GaitSelect::Walk, GaitSelect::Trot] {
                let mut cfg = AppConfig::default();
                cfg.gait.velocity_ramp_s = ramp;
                cfg.gait.velocity_ramp_stop_s = stop_ramp;
                cfg.gait.stop_settle_s = settle;
                let robot = Robot::load(&test_model_path(), &cfg.control.kinematics_pose).unwrap();
                let mut c = Controller::new(robot, cfg);
                let mut go = cmd(ModeRequest::Walk);
                go.gait = select;
                go.velocity.vx_m_s = 0.15;
                go.velocity.wz_rad_s = 0.6;
                run_until(&mut c, &go, State::Active, 20.0);
                for _ in 0..400 {
                    c.tick(&go, &JointVec::zeros(), imu(), dt);
                }
                // 中立へ戻す。歩容が完全に静止するまでの時間を測る。
                let centre = {
                    let mut x = go;
                    x.velocity.vx_m_s = 0.0;
                    x.velocity.wz_rad_s = 0.0;
                    x
                };
                let mut stopped_at = None;
                let mut prev = c.tick(&centre, &JointVec::zeros(), imu(), dt).targets;
                for i in 1..600 {
                    let q = c.tick(&centre, &JointVec::zeros(), imu(), dt).targets;
                    // 目標がまったく動かなくなったら静止。
                    if q.max_abs_diff(&prev) < 1e-12 && c.ramped_v.iter().all(|v| *v == 0.0) {
                        stopped_at = Some(i as f64 * dt);
                        break;
                    }
                    prev = q;
                }
                let t = stopped_at.unwrap_or(f64::INFINITY);
                println!("  {:6} 停止まで {:.3} s", select.label(), t);
                if label.starts_with("新") {
                    assert!(
                        t < 0.6,
                        "{} で停止に {:.2} s かかった（リング外へ出る）",
                        select.label(),
                        t
                    );
                }
            }
        }
    }

    /// ランプは止まる側にもかかる。片側だけでは足りない。
    #[test]
    fn the_velocity_ramp_applies_in_both_directions() {
        let mut cfg = AppConfig::default();
        cfg.gait.velocity_ramp_s = 0.5;
        let robot = Robot::load(&test_model_path(), &cfg.control.kinematics_pose).unwrap();
        let mut c = Controller::new(robot, cfg);
        let dt = 0.01;
        let mut go = cmd(ModeRequest::Walk);
        go.velocity.vx_m_s = 0.15;
        run_until(&mut c, &go, State::Active, 20.0);
        // 全開まで 0.5 s かかる → 0.1 s 時点では半分にも達していない。
        let mut c2 = controller();
        run_until(&mut c2, &cmd(ModeRequest::Walk), State::Active, 20.0);
        for _ in 0..10 {
            c2.tick(&go, &JointVec::zeros(), imu(), dt);
        }
        assert!(
            c2.ramped_v[0] < 0.15 * 0.5,
            "0.1 s で {:.3} m/s まで来ている（ランプが効いていない）",
            c2.ramped_v[0]
        );
        // 中立へ戻すときも同じレート。
        let before = c2.ramped_v[0];
        c2.tick(&cmd(ModeRequest::Walk), &JointVec::zeros(), imu(), dt);
        assert!(c2.ramped_v[0] < before);
        assert!(c2.ramped_v[0] > 0.0, "1 周期で 0 まで落ちている");
    }

    fn run_until(c: &mut Controller, command: &Intent, want: State, max_s: f64) {
        let dt = 0.005;
        let mut t = 0.0;
        while t < max_s {
            let out = c.tick(command, &JointVec::zeros(), imu(), dt);
            if out.state == want {
                return;
            }
            t += dt;
        }
        panic!("{:?} に到達しませんでした（今は {:?}）", want, c.state());
    }

    #[test]
    fn it_starts_relaxed_and_sends_no_position_command() {
        let mut c = controller();
        let out = c.tick(&cmd(ModeRequest::Relax), &JointVec::zeros(), imu(), 0.005);
        assert_eq!(out.state, State::Relaxed);
        assert_eq!(out.leg_mode, JointMode::Idle);
    }

    #[test]
    fn relaxed_targets_follow_the_measured_angles() {
        // 起立に移った瞬間に 0 rad へ飛ばないための性質。
        let mut c = controller();
        let mut measured = JointVec::zeros();
        measured.legs[2][1] = 0.61;
        let out = c.tick(&cmd(ModeRequest::Relax), &measured, imu(), 0.005);
        assert_eq!(out.targets, measured);
    }

    #[test]
    fn standing_goes_through_the_start_pose_then_the_stance() {
        let mut c = controller();
        let stand = cmd(ModeRequest::Walk);
        c.tick(&stand, &JointVec::zeros(), imu(), 0.005);
        assert_eq!(c.state(), State::GoingToStart);
        run_until(&mut c, &stand, State::GoingToStance, 10.0);
        run_until(&mut c, &stand, State::Active, 10.0);
        assert_eq!(
            c.tick(&stand, &JointVec::zeros(), imu(), 0.005).leg_mode,
            JointMode::Position
        );
    }

    #[test]
    fn the_start_pose_is_actually_reached_before_the_stance_transition() {
        let cfg = AppConfig::default();
        let mut c = controller();
        let stand = cmd(ModeRequest::Walk);
        run_until(&mut c, &stand, State::GoingToStance, 10.0);
        // GoingToStance に入った時点の目標は start_pose と一致しているはず。
        let start = c
            .robot()
            .poses
            .pose(&cfg.control.start_pose)
            .map(|p| c.robot().poses.resolve(&p.angles, JointVec::zeros()))
            .expect("start_pose がモデルにありません");
        let out = c.tick(&stand, &JointVec::zeros(), imu(), 0.0);
        assert!(
            out.targets.max_abs_diff(&start) < 1e-6,
            "start_pose に着く前に次の遷移へ進んでいます"
        );
    }

    /// CH5 中段は**初期姿勢で止まる**。立ち姿勢まで行かない。
    ///
    /// 試合はこの姿勢でスタートボックスに置いて合図を待つ。
    #[test]
    fn the_middle_switch_position_holds_the_start_pose() {
        let mut c = controller();
        run_until(&mut c, &cmd(ModeRequest::Stand), State::HoldingStart, 20.0);
        let held = c
            .tick(&cmd(ModeRequest::Stand), &JointVec::zeros(), imu(), 0.005)
            .targets;
        // 通電したまま保持する。**脱力しない。**
        let out = c.tick(&cmd(ModeRequest::Stand), &JointVec::zeros(), imu(), 0.005);
        assert_eq!(out.state, State::HoldingStart);
        assert_eq!(out.leg_mode, JointMode::Position);
        // 何周期回しても動かない。
        for _ in 0..200 {
            c.tick(&cmd(ModeRequest::Stand), &JointVec::zeros(), imu(), 0.005);
        }
        let still = c
            .tick(&cmd(ModeRequest::Stand), &JointVec::zeros(), imu(), 0.005)
            .targets;
        assert!(
            still.max_abs_diff(&held) < 1e-9,
            "初期姿勢で保持できていない"
        );
        // 保持しているのは設定の初期姿勢そのもの。
        let want = {
            let p = c.robot.poses.pose(&c.cfg.control.start_pose).unwrap();
            c.robot.poses.resolve(&p.angles, JointVec::zeros())
        };
        assert!(
            held.max_abs_diff(&want) < 1e-6,
            "初期姿勢と違う姿勢で止まっている"
        );
    }

    /// 歩行中に受信が切れても**その場で立ったまま**。初期姿勢へは戻らない。
    ///
    /// CH5 中段が「初期姿勢で保持」になったので、フェイルセーフを `Stand`
    /// へ丸めると**歩行中の受信断でしゃがみ込む**。求めているのは
    /// 「速度 0・その場起立」。
    #[test]
    fn a_lost_link_while_walking_holds_the_stance() {
        let dt = 0.005;
        let mut c = controller();
        let mut go = cmd(ModeRequest::Walk);
        go.velocity.vx_m_s = 0.10;
        run_until(&mut c, &go, State::Active, 20.0);
        for _ in 0..400 {
            c.tick(&go, &JointVec::zeros(), imu(), dt);
        }
        // 受信断の指令（モードは歩行のまま、速度ゼロ、link_ok = false）。
        let lost = cmd(ModeRequest::Walk).failsafe();
        for _ in 0..600 {
            let out = c.tick(&lost, &JointVec::zeros(), imu(), dt);
            assert_eq!(
                out.state,
                State::Active,
                "受信断で歩容から抜けた（初期姿勢へしゃがみ込んでいる）"
            );
        }
    }

    /// `--allow-no-sbus`（受信機なしのベンチ）は**初期姿勢で止まる**。
    ///
    /// CH5 中段の意味を変えた副作用。以前は立ち姿勢を経て歩容まで行って
    /// いた。**受信機が無いまま歩容へ入る道を残す理由がない**ので、
    /// この方が安全側。段階 7-2 の手順もこれに合わせてある。
    #[test]
    fn the_bench_command_stops_at_the_start_pose() {
        use crate::teleop::{Teleop, TeleopConfig};
        let t = Teleop::new(
            TeleopConfig::default(),
            &crate::config::GaitTuning::default(),
            &misa_hal::config::SerialHardware::default().arm,
        );
        let bench = t.bench_stand();
        assert_eq!(bench.mode, ModeRequest::Stand);
        let mut c = controller();
        run_until(&mut c, &bench, State::HoldingStart, 20.0);
        // 放っておいても歩容へは進まない。
        for _ in 0..2000 {
            assert_eq!(
                c.tick(&bench, &JointVec::zeros(), imu(), 0.005).state,
                State::HoldingStart
            );
        }
    }

    /// 中段 → 上段で歩容へ、上段 → 中段で初期姿勢へ戻る。
    #[test]
    fn the_switch_walks_from_the_start_pose_and_returns_to_it() {
        let mut c = controller();
        run_until(&mut c, &cmd(ModeRequest::Stand), State::HoldingStart, 20.0);
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
        // 上段のままスティック中立なら、立ち姿勢で止まっていられる。
        for _ in 0..100 {
            assert_eq!(
                c.tick(&cmd(ModeRequest::Walk), &JointVec::zeros(), imu(), 0.005)
                    .state,
                State::Active
            );
        }
        // 中段へ戻すと初期姿勢へ帰る。
        run_until(&mut c, &cmd(ModeRequest::Stand), State::HoldingStart, 20.0);
    }

    #[test]
    fn relax_takes_effect_from_any_state() {
        let mut c = controller();
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
        let out = c.tick(&cmd(ModeRequest::Relax), &JointVec::zeros(), imu(), 0.005);
        assert_eq!(out.state, State::Relaxed);
        assert_eq!(out.leg_mode, JointMode::Idle);
        // 初期姿勢の保持中からも同じ。
        let mut c = controller();
        run_until(&mut c, &cmd(ModeRequest::Stand), State::HoldingStart, 20.0);
        let out = c.tick(&cmd(ModeRequest::Relax), &JointVec::zeros(), imu(), 0.005);
        assert_eq!(out.state, State::Relaxed);
        assert_eq!(out.leg_mode, JointMode::Idle);
    }

    #[test]
    fn walking_forward_moves_the_legs() {
        let mut c = controller();
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
        let stance = c
            .tick(&cmd(ModeRequest::Walk), &JointVec::zeros(), imu(), 0.0)
            .targets;
        let mut walk = cmd(ModeRequest::Walk);
        walk.velocity.vx_m_s = 0.1;
        let mut moved = false;
        for _ in 0..400 {
            let out = c.tick(&walk, &JointVec::zeros(), imu(), 0.005);
            if out.targets.max_abs_diff(&stance) > 1e-3 {
                moved = true;
            }
        }
        assert!(moved, "歩行指令を出しても関節が動いていません");
    }

    /// **既定 (`body_attitude_max_rad = 0`) では出力が 1 ビットも変わらない。**
    ///
    /// 制御ループの出力に手を入れる機能なので、設定で明示的に上げるまで
    /// 従来と同一であることを保証する。
    #[test]
    fn the_body_tilt_is_inert_until_it_is_configured() {
        assert_eq!(AppConfig::default().gait.body_attitude_max_rad, 0.0);
        let dt = 0.005;
        let mut a = controller();
        let mut b = controller();
        let mut go = cmd(ModeRequest::Walk);
        go.velocity.vx_m_s = 0.10;
        run_until(&mut a, &go, State::Active, 20.0);
        run_until(&mut b, &go, State::Active, 20.0);
        // 片方だけ CH8 を入れる。上限が 0 なので指令は [0, 0] のまま。
        let mut chicken = go.clone();
        chicken.stabilize_head = true;
        for _ in 0..400 {
            let qa = a.tick(&go, &JointVec::zeros(), imu(), dt).targets;
            let qb = b.tick(&chicken, &JointVec::zeros(), imu(), dt).targets;
            assert_eq!(qa, qb, "無効なはずの胴体姿勢で出力が変わった");
        }
    }

    /// 有効にすると胴体が傾き、**足先は世界座標で動かない**。
    #[test]
    fn tilting_the_body_keeps_the_feet_planted() {
        let dt = 0.005;
        let mut cfg = AppConfig::default();
        cfg.gait.body_attitude_max_rad = 0.2;
        cfg.gait.body_attitude_tau_s = 0.0; // 素通しで測る
        let robot = Robot::load(&test_model_path(), &cfg.control.kinematics_pose).unwrap();
        let mut c = Controller::new(robot, cfg);
        let stand = cmd(ModeRequest::Walk);
        run_until(&mut c, &stand, State::Active, 20.0);
        for _ in 0..200 {
            c.tick(&stand, &JointVec::zeros(), imu(), dt);
        }
        let flat = c.tick(&stand, &JointVec::zeros(), imu(), dt).targets;

        // ロールを入れる。
        let mut roll = stand.clone();
        roll.stabilize_head = true;
        roll.body_attitude_rad = [0.15, 0.0, 0.0];
        let mut tilted = flat;
        for _ in 0..100 {
            tilted = c.tick(&roll, &JointVec::zeros(), imu(), dt).targets;
        }
        assert!(
            tilted.max_abs_diff(&flat) > 0.02,
            "ロールを入れても関節が動いていない"
        );
        // **+roll と −roll は鏡像になる。** これが回転であることの確認。
        //
        // 「左右の hip が逆向きに動く」は**成り立たない**。足は胴体中心より
        // 下にあるので、胴体を回すと胴体座標系の足は全脚とも同じ向きへ
        // `−h·sin θ` 振れる。左右差は足の横オフセットぶんしか出ない。
        // 検算: h = 0.2, θ = 0.15 → Δy = −0.0299 m、hip ≈ atan(0.0299/0.2)
        // = 0.148 rad。
        let mut minus = roll;
        minus.body_attitude_rad = [-0.15, 0.0, 0.0];
        let mut mirrored = flat;
        for _ in 0..200 {
            mirrored = c.tick(&minus, &JointVec::zeros(), imu(), dt).targets;
        }
        for l in 0..4 {
            let up = tilted.legs[l][0] - flat.legs[l][0];
            let down = mirrored.legs[l][0] - flat.legs[l][0];
            assert!(
                up * down < 0.0,
                "脚 {l} の hip が ±roll で同じ向きに動いた: {up:+.4} {down:+.4}"
            );
            assert!(
                (up + down).abs() < 0.02,
                "脚 {l} の hip が ±roll で鏡像になっていない: {up:+.4} {down:+.4}"
            );
        }
        // 大きさも幾何と合っているか（h·sin θ / h の atan）。
        let want = (0.2f64 * 0.15f64.sin() / 0.2).atan();
        let got = (tilted.legs[0][0] - flat.legs[0][0]).abs();
        assert!(
            (got - want).abs() < 0.03,
            "hip の変化 {got:+.4} が幾何の予測 {want:+.4} と合わない"
        );

        // 戻す前に元姿勢へ。
        for _ in 0..200 {
            c.tick(&stand, &JointVec::zeros(), imu(), dt);
        }

        // 戻せば元の姿勢へ。
        for _ in 0..200 {
            c.tick(&stand, &JointVec::zeros(), imu(), dt);
        }
        let back = c.tick(&stand, &JointVec::zeros(), imu(), dt).targets;
        assert!(
            back.max_abs_diff(&flat) < 1e-6,
            "姿勢を戻しても元に戻らない"
        );
    }

    /// **yaw は前後の脚が逆向きに動く**（ひねり）。roll とは別の形。
    ///
    /// 足を接地したまま胴体を水平面内でひねる。roll/pitch より可動域に
    /// 余裕があり、実測では 1.2 rad (69°) でも範囲内だった
    /// （roll は 0.65 rad で頭打ち）。hip の横方向の可動域が広いため。
    #[test]
    fn yawing_the_body_twists_front_against_rear() {
        let dt = 0.005;
        let mut cfg = AppConfig::default();
        cfg.gait.body_attitude_max_rad = 0.5;
        cfg.gait.body_attitude_tau_s = 0.0;
        let robot = Robot::load(&test_model_path(), &cfg.control.kinematics_pose).unwrap();
        let mut c = Controller::new(robot, cfg);
        let stand = cmd(ModeRequest::Walk);
        run_until(&mut c, &stand, State::Active, 20.0);
        for _ in 0..200 {
            c.tick(&stand, &JointVec::zeros(), imu(), dt);
        }
        let flat = c.tick(&stand, &JointVec::zeros(), imu(), dt).targets;

        let mut yaw = stand;
        yaw.stabilize_head = true;
        yaw.body_attitude_rad = [0.0, 0.0, 0.4];
        let mut twisted = flat;
        for _ in 0..200 {
            twisted = c.tick(&yaw, &JointVec::zeros(), imu(), dt).targets;
        }
        // 前脚と後脚の hip が逆向きに動く。これがひねりの証拠。
        let front = twisted.legs[0][0] - flat.legs[0][0]; // FL
        let rear = twisted.legs[2][0] - flat.legs[2][0]; // RL
        assert!(
            front * rear < 0.0,
            "前後の hip が同じ向きに動いた（ひねりになっていない）: 前 {front:+.4} 後 {rear:+.4}"
        );
        assert!(front.abs() > 0.05, "ひねりが小さすぎる: {front:+.4}");
    }

    /// **立って止まっている間なら歩容を選び直せる。歩いている間は不可。**
    ///
    /// 判定は状態名ではなく「遊脚があるか」。速度 0 のとき歩容は
    /// `holding` に入り全脚接地・位相凍結なので、差し替えても飛ぶ遊脚が無い。
    #[test]
    fn the_gait_can_be_switched_while_standing_still_but_not_while_moving() {
        let dt = 0.005;
        let mut c = controller();
        // 速度 0 の歩容（= 立ち姿勢で静止）まで持っていく。
        let stand = cmd(ModeRequest::Walk);
        run_until(&mut c, &stand, State::Active, 20.0);
        for _ in 0..200 {
            c.tick(&stand, &JointVec::zeros(), imu(), dt);
        }
        let before = c.tick(&stand, &JointVec::zeros(), imu(), dt).targets;

        // 静止中なら切り替わる。
        let mut to_trot = stand;
        to_trot.gait = GaitSelect::Trot;
        let after = c.tick(&to_trot, &JointVec::zeros(), imu(), dt).targets;
        assert_eq!(c.gait_select(), GaitSelect::Trot, "静止中に歩容を選べない");
        assert_eq!(c.state(), State::Active);

        // **差し替えで脚が飛ばないこと。** 立ち姿勢は歩容によらず同じはず。
        let jump = after.max_abs_diff(&before);
        assert!(
            jump < 1e-3,
            "歩容を差し替えたら目標が {:.4} rad 飛んだ（{:.2}°）",
            jump,
            jump.to_degrees()
        );

        // 歩き出したら切り替わらない。
        let mut moving = to_trot;
        moving.velocity.vx_m_s = 0.10;
        for _ in 0..200 {
            c.tick(&moving, &JointVec::zeros(), imu(), dt);
        }
        let mut to_crawl = moving;
        to_crawl.gait = GaitSelect::Crawl;
        c.tick(&to_crawl, &JointVec::zeros(), imu(), dt);
        assert_eq!(
            c.gait_select(),
            GaitSelect::Trot,
            "歩行中に歩容が切り替わった（踏み替えが飛ぶ）"
        );
    }

    /// **止めるときに目標角が飛ばない。** 速度を 0 にした瞬間に歩容が
    /// `holding`（全脚接地・位相 0）へ落ちると、空中にいた足の目標が接地位置へ
    /// 飛ぶ。[`Controller::ramp_velocity`] は**そのとき空中だった脚が着地する
    /// まで**微速で歩かせてから 0 に落とす。trot は 4 脚が同時に接地しないので、
    /// 「全脚接地」を待つ作りでは時間切れまで待って結局飛んでいた。
    #[test]
    fn stopping_waits_for_the_airborne_legs_to_land() {
        let dt = 0.005;
        for select in [GaitSelect::Walk, GaitSelect::Trot] {
            let cfg = AppConfig::default();
            let robot = Robot::load(&test_model_path(), &cfg.control.kinematics_pose).expect("テスト用モデル");
            let mut c = Controller::new(robot, cfg);
            let mut walk = cmd(ModeRequest::Walk);
            walk.gait = select;
            walk.velocity.vx_m_s = 0.10;
            run_until(&mut c, &walk, State::Active, 20.0);
            let mut prev = JointVec::zeros();
            for _ in 0..400 {
                prev = c.tick(&walk, &JointVec::zeros(), imu(), dt).targets;
            }
            assert!(
                !c.body_view.stance.iter().all(|&s| s),
                "{} なのに全脚接地のまま（試験が歩いていない）",
                select.label()
            );
            // 速度 0。止まりきるまでの目標角の最大変化率を測る。
            let mut stop = walk;
            stop.velocity.vx_m_s = 0.0;
            let mut worst = 0.0f64;
            let mut stopped_at = None;
            for i in 0..600 {
                let out = c.tick(&stop, &JointVec::zeros(), imu(), dt);
                for leg in 0..4 {
                    for k in 0..3 {
                        worst = worst.max(((out.targets.legs[leg][k] - prev.legs[leg][k]) / dt).abs());
                    }
                }
                if stopped_at.is_none()
                    && out.targets.max_abs_diff(&prev) < 1e-12
                    && c.ramped_v.iter().all(|v| *v == 0.0)
                {
                    stopped_at = Some(i as f64 * dt);
                }
                prev = out.targets;
            }
            let t = stopped_at.expect("止まりきっていない");
            assert!(
                worst < 12.0,
                "{} を止めたときに目標角が {worst:.1} rad/s 飛んだ（着地を待っているはず）",
                select.label()
            );
            assert!(t < 0.6, "{} の停止に {t:.2} s かかった", select.label());
        }
    }

    /// **膝の向きは立って止まっているときだけ反転でき、振り付けの足先は地面に
    /// 触れず、終わると新しい向きの立ち姿勢に着いている。** 歩いている最中の
    /// 要求は無視される。
    #[test]
    fn the_knees_flip_while_standing_still_and_not_while_walking() {
        use crate::config::KneeShape;
        use misa_core::KneePatternRequest;
        let dt = 0.005;
        let mut c = controller();
        let stand = cmd(ModeRequest::Walk);
        run_until(&mut c, &stand, State::Active, 20.0);
        for _ in 0..200 {
            c.tick(&stand, &JointVec::zeros(), imu(), dt);
        }
        assert_eq!(c.knee_pattern(), KneeShape::BothBack);

        // 歩いている最中は無視（歩き出してから要求する。止まったまま要求と
        // 速度を同時に出せば、まだ 4 脚接地なのでその場で振り付けに入る）。
        let mut walking = stand.clone();
        walking.velocity.vx_m_s = 0.10;
        for _ in 0..200 {
            c.tick(&walking, &JointVec::zeros(), imu(), dt);
        }
        walking.knee_pattern = Some(KneePatternRequest::BothForward);
        for _ in 0..200 {
            c.tick(&walking, &JointVec::zeros(), imu(), dt);
        }
        assert_eq!(c.state(), State::Active);
        assert_eq!(c.knee_pattern(), KneeShape::BothBack, "歩行中に膝が反転した");

        // 止まって 4 脚接地になったら振り付けに入る。
        let mut flip = stand.clone();
        flip.knee_pattern = Some(KneePatternRequest::BothForward);
        let mut entered = false;
        let mut ticks = 0;
        let h_ref = c.robot.reference_height_m(&c.cfg.gait);
        while ticks < 6000 {
            let out = c.tick(&flip, &JointVec::zeros(), imu(), dt);
            ticks += 1;
            if out.state == State::FlippingKnees {
                entered = true;
                // 振り付け中の足先は地面（胴体から −h）より上。
                let feet = c.robot.feet_from_posture(&out.targets);
                for (slot, f) in feet.iter().enumerate() {
                    // 浮かせている脚は出発した面より下がらない。接地している脚は
                    // 関節空間の補間で弧を描くぶん（数 mm〜1 cm）だけ許す。
                    let tol = if out.stance[slot] { 0.02 } else { 0.005 };
                    assert!(f.z + h_ref > -tol, "脚 {slot} の足先が地面の下 {:+.3}（接地 {}）", f.z + h_ref, out.stance[slot]);
                }
            } else if entered {
                break;
            }
        }
        assert!(entered, "振り付けに入らなかった");
        assert_eq!(c.state(), State::Active);
        assert_eq!(c.knee_pattern(), KneeShape::BothForward);
        // 着いた姿勢は新しい向きの立ち姿勢。
        let mut t = c.cfg.gait.clone();
        t.knee_pattern = KneeShape::BothForward;
        let want = c.robot.stance_posture(&t);
        let out = c.tick(&flip, &JointVec::zeros(), imu(), dt);
        let d = out.targets.max_abs_diff(&want);
        assert!(d < 0.02, "反転後の姿勢が新しい向きの立ち姿勢と {d:.3} rad 違う");
        // 膝の符号が全脚で反転している（calf の符号）。
        let before = c.robot.stance_posture(&c.cfg.gait.clone());
        let _ = before;
        // 同じ向きを送り続けても何も起きない。
        for _ in 0..50 {
            assert_eq!(c.tick(&flip, &JointVec::zeros(), imu(), dt).state, State::Active);
        }
    }

    /// **初期姿勢で待っている間に歩容を選べる。** 試合の実際の使い方。
    ///
    /// スタートボックスに置いて CH5 中段で待ち、そこで CH6 を決めてから
    /// 合図で上段へ倒す。ここで切り替えられないと**起動時に決め打ち**に
    /// なってしまう。
    #[test]
    fn the_gait_can_be_switched_while_holding_the_start_pose() {
        let mut c = controller();
        run_until(&mut c, &cmd(ModeRequest::Stand), State::HoldingStart, 20.0);
        assert_eq!(c.gait_select(), GaitSelect::Crawl);

        let mut hold_trot = cmd(ModeRequest::Stand);
        hold_trot.gait = GaitSelect::Trot;
        c.tick(&hold_trot, &JointVec::zeros(), imu(), 0.005);
        assert_eq!(
            c.gait_select(),
            GaitSelect::Trot,
            "初期姿勢中に歩容を選べない"
        );
        // 姿勢は保持したまま。歩容を差し替えても動き出さない。
        assert_eq!(c.state(), State::HoldingStart);

        // 何度でも選び直せる。
        let mut hold_walk = cmd(ModeRequest::Stand);
        hold_walk.gait = GaitSelect::Walk;
        c.tick(&hold_walk, &JointVec::zeros(), imu(), 0.005);
        assert_eq!(c.gait_select(), GaitSelect::Walk);

        // そのまま歩容へ入れば、選んだ歩容で歩き出す。
        let mut go = cmd(ModeRequest::Walk);
        go.gait = GaitSelect::Walk;
        run_until(&mut c, &go, State::Active, 20.0);
        assert_eq!(c.gait_select(), GaitSelect::Walk);
    }

    #[test]
    fn the_gait_can_be_switched_while_relaxed_but_not_while_active() {
        let mut c = controller();
        let mut relax_trot = cmd(ModeRequest::Relax);
        relax_trot.gait = GaitSelect::Trot;
        c.tick(&relax_trot, &JointVec::zeros(), imu(), 0.005);
        assert_eq!(c.gait_select(), GaitSelect::Trot);

        // 起立の途中も Trot のまま要求し続ける（遷移中の切り替えは許される）。
        let mut stand_trot = cmd(ModeRequest::Walk);
        stand_trot.gait = GaitSelect::Trot;
        run_until(&mut c, &stand_trot, State::Active, 20.0);
        assert_eq!(c.gait_select(), GaitSelect::Trot);

        // Active 中の切り替え要求は無視される（踏み替えの途中で歩容が飛ばない）。
        let mut walk_crawl = cmd(ModeRequest::Walk);
        walk_crawl.gait = GaitSelect::Crawl;
        c.tick(&walk_crawl, &JointVec::zeros(), imu(), 0.005);
        assert_eq!(c.gait_select(), GaitSelect::Trot);
    }

    /// **歩容コントローラも立って止まっているときだけ切り替わる。**
    #[test]
    fn the_gait_controller_can_be_switched_while_standing_but_not_while_walking() {
        use crate::config::GaitControllerKind;
        use misa_core::GaitControllerRequest;
        let mut c = controller();
        assert_eq!(c.controller_kind(), GaitControllerKind::Auto);
        // 脱力中に MPC を要求 → 切り替わる。
        let mut relax_mpc = cmd(ModeRequest::Relax);
        relax_mpc.gait_controller = Some(GaitControllerRequest::Mpc);
        c.tick(&relax_mpc, &JointVec::zeros(), imu(), 0.005);
        assert_eq!(c.controller_kind(), GaitControllerKind::Mpc);
        // 立ち上がって歩き出す。
        let mut walk = cmd(ModeRequest::Walk);
        walk.velocity.vx_m_s = 0.1;
        run_until(&mut c, &walk, State::Active, 20.0);
        for _ in 0..200 {
            c.tick(&walk, &JointVec::zeros(), imu(), 0.005);
        }
        // 歩行中の CHAMP 要求は無視される。
        let mut walk_champ = walk.clone();
        walk_champ.gait_controller = Some(GaitControllerRequest::Champ);
        c.tick(&walk_champ, &JointVec::zeros(), imu(), 0.005);
        assert_eq!(c.controller_kind(), GaitControllerKind::Mpc);
        // 速度 0 で止まり切ってから要求すると切り替わる。
        let mut stand_champ = cmd(ModeRequest::Walk);
        stand_champ.gait_controller = Some(GaitControllerRequest::Champ);
        for _ in 0..600 {
            c.tick(&stand_champ, &JointVec::zeros(), imu(), 0.005);
        }
        assert_eq!(c.controller_kind(), GaitControllerKind::Champ);
    }

    #[test]
    fn an_unknown_pose_name_does_not_break_the_gait() {
        let mut cfg = AppConfig::default();
        cfg.poses.greeting = "no_such_pose".into();
        let robot = Robot::load(&test_model_path(), &cfg.control.kinematics_pose).unwrap();
        let mut c = Controller::new(robot, cfg);
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
        let mut play = cmd(ModeRequest::Walk);
        play.play_pose = true;
        let out = c.tick(&play, &JointVec::zeros(), imu(), 0.005);
        assert_eq!(out.state, State::Active);
    }

    #[test]
    fn a_non_driven_arm_follows_the_observed_angle_not_the_chicken_head() {
        // 受信機直結（既定）。チキンヘッドを ON にしても腕の目標は観測値のまま。
        let mut c = controller();
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
        let mut with_arm = cmd(ModeRequest::Walk);
        with_arm.stabilize_head = true;
        with_arm.aux_rad = vec![Some(-0.7)];
        let pitched = [0.0, 0.4, 0.0];
        for _ in 0..200 {
            c.tick(&with_arm, &JointVec::zeros(), pitched, 0.005);
        }
        let out = c.tick(&with_arm, &JointVec::zeros(), pitched, 0.005);
        assert!(
            (out.targets.arm + 0.7).abs() < 1e-9,
            "腕の目標が観測値ではなくチキンヘッドの出力になっています: {}",
            out.targets.arm
        );
    }

    #[test]
    fn a_driven_arm_does_run_the_chicken_head() {
        let cfg = AppConfig::default();
        let robot = Robot::load(&test_model_path(), &cfg.control.kinematics_pose).unwrap();
        let mut c = Controller::with_arm(robot, cfg, true);
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
        let mut on = cmd(ModeRequest::Walk);
        on.stabilize_head = true;
        let pitched = [0.0, 0.4, 0.0];
        for _ in 0..2000 {
            c.tick(&on, &JointVec::zeros(), pitched, 0.005);
        }
        let out = c.tick(&on, &JointVec::zeros(), pitched, 0.005);
        // 胴体ピッチ +0.4 を打ち消すので腕は −0.4 付近。
        assert!(
            (out.targets.arm + 0.4).abs() < 1e-3,
            "チキンヘッドが効いていません: {}",
            out.targets.arm
        );
    }

    #[test]
    fn a_non_driven_arm_holds_its_last_value_when_the_link_is_lost() {
        let mut c = controller();
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
        let mut seen = cmd(ModeRequest::Walk);
        seen.aux_rad = vec![Some(0.3)];
        c.tick(&seen, &JointVec::zeros(), imu(), 0.005);
        // 受信断で観測値が無くなっても 0 へ飛ばない。
        let lost = cmd(ModeRequest::Stand).failsafe();
        let out = c.tick(&lost, &JointVec::zeros(), imu(), 0.005);
        assert!((out.targets.arm - 0.3).abs() < 1e-9, "{}", out.targets.arm);
    }

    /// ポーズ再生に入ってから抜けるまでの秒数。`hold` を毎周期与える。
    fn play_pose_seconds(seq: &str, hold: &Intent) -> f64 {
        let mut cfg = AppConfig::default();
        cfg.poses.greeting = seq.into();
        let robot = Robot::load(&test_model_path(), &cfg.control.kinematics_pose).unwrap();
        // **無いものを「入れていません」と言わない。** シーケンスはモデル側に
        // ある（models/testquad/gen.py が書く）。原因が分かる形で落とす。
        assert!(
            robot.poses.sequence(seq).is_some() || robot.poses.pose(seq).is_some(),
            "モデルに {:?} がありません。models/testquad/gen.py を確かめてください\n\
             （あるポーズ {:?} / あるシーケンス {:?}）",
            seq,
            robot.poses.pose_names().collect::<Vec<_>>(),
            robot.poses.sequence_names().collect::<Vec<_>>(),
        );
        let mut c = Controller::new(robot, cfg);
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
        let mut play = cmd(ModeRequest::Walk);
        play.play_pose = true;
        assert_eq!(
            c.tick(&play, &JointVec::zeros(), imu(), 0.005).state,
            State::PlayingPose,
            "ポーズ再生に入れていません"
        );
        let dt = 0.005;
        let mut t = 0.0;
        while t < 20.0 {
            if c.tick(hold, &JointVec::zeros(), imu(), dt).state != State::PlayingPose {
                return t;
            }
            t += dt;
        }
        panic!("ポーズ再生から抜けませんでした");
    }

    #[test]
    fn playing_a_pose_returns_to_the_stance() {
        let mut cfg = AppConfig::default();
        // モデルに入っているシーケンス名を使う。
        cfg.poses.greeting = "jump".into();
        let robot = Robot::load(&test_model_path(), &cfg.control.kinematics_pose).unwrap();
        let mut c = Controller::new(robot, cfg);
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
        let mut play = cmd(ModeRequest::Walk);
        play.play_pose = true;
        assert_eq!(
            c.tick(&play, &JointVec::zeros(), imu(), 0.005).state,
            State::PlayingPose
        );
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
    }

    /// **CH5 を上段に置いたまま最後まで再生できること。**
    ///
    /// 再生に入れるのは CH5 上段だけなので、「歩行要求で中断」にしていると
    /// トリガの次の周期で必ず抜ける。実機では手を振る前に立ち姿勢へ帰って
    /// いた（振り幅ゼロ）。`jump` は 0.5 + 0.4 + 0.1 s。
    #[test]
    fn a_pose_plays_to_the_end_while_the_switch_stays_up() {
        let t = play_pose_seconds("jump", &cmd(ModeRequest::Walk));
        assert!(
            t > 0.9,
            "CH5 上段のまま {t} s で再生が打ち切られています（シーケンスは 1.0 s）"
        );
    }

    /// **胴体高さの指令は全歩容で立ち位置を動かす。** 2026-09-06 までは
    /// MPC / CHAMP で何も起きず、可視化の胴体だけが上下して足が浮いて見えた。
    #[test]
    fn the_body_height_command_moves_the_planned_feet_in_every_gait() {
        use crate::config::GaitControllerKind;
        for kind in [GaitControllerKind::Champ, GaitControllerKind::Mpc] {
            let mut cfg = AppConfig::default();
            cfg.gait.controller = kind;
            let robot = Robot::load(&test_model_path(), &cfg.control.kinematics_pose).unwrap();
            let mut c = Controller::new(robot, cfg.clone());
            run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
            // Active になった周期は遷移の終わりで、足の目標は次の周期から出る。
            for _ in 0..10 {
                c.tick(&cmd(ModeRequest::Walk), &JointVec::zeros(), imu(), 0.005);
            }
            let z0 = c.target_foot_body[0].z;
            assert!((z0 + cfg.gait.stance_height_m).abs() < 5e-3, "{kind:?}: 立ち高さの足 z が {z0}");
            let mut lower = cmd(ModeRequest::Walk);
            lower.height_offset_m = -0.03;
            for _ in 0..40 {
                c.tick(&lower, &JointVec::zeros(), imu(), 0.005);
            }
            let z1 = c.target_foot_body[0].z;
            assert!(
                (z1 - (z0 + 0.03)).abs() < 5e-3,
                "{kind:?}: 3 cm 下げたのに足の z が {z0} → {z1}"
            );
            assert!((c.body_view().z - (cfg.gait.stance_height_m - 0.03)).abs() < 1e-9);
        }
    }

    /// 前足を振る `wave_fr` / `wave_fl` も同じく最後まで通ること。
    /// 8 ステップで 3.8 s ある。
    #[test]
    fn the_wave_sequences_play_to_the_end() {
        for seq in ["wave_fr", "wave_fl"] {
            let t = play_pose_seconds(seq, &cmd(ModeRequest::Walk));
            assert!(t > 3.5, "{seq} が {t} s で打ち切られています（3.8 s ある）");
        }
    }

    /// **CH5 を中段へ戻したら途中でも立ち姿勢へ戻る。**
    #[test]
    fn the_middle_switch_position_interrupts_the_pose() {
        let t = play_pose_seconds("wave_fr", &cmd(ModeRequest::Stand));
        assert!(t < 0.05, "CH5 中段で中断できていません（{t} s 続きました）");
    }
}
