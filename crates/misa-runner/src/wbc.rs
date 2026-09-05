//! 全身制御（WBC）— 階層 QP の解を実機の指令へ落とす層。
//!
//! 毎周期 `quadruped_gait::wbc` の 3 優先度 HoQP に
//! `x = [q̈ | f_GRF | τ]` を解かせ、その解を**トルク・速度・位置のどれかの
//! 指令**にして返す（[`crate::config::WbcOutput`]）。解くのは 1 回で、
//! 出し方だけが変わる。
//!
//! ```text
//!   優先度 0（硬い）  浮遊ベースの運動方程式 + 摩擦錐 + トルク上限
//!                     + 立脚足が滑らないこと
//!   優先度 1（柔らか）胴体加速度の追従 + 関節空間の追従（遊脚と補助軸）
//!   優先度 2（柔らか）接地力の配分 + 重力補償トルクへの寄せ
//! ```
//!
//! # 参照は 2 通りある
//!
//! ```text
//!   歩容が MPC 系      f_grf_des  = MPC が解いた接地力（鈍らせて使う）
//!   （mpc / centroidal）a_base_des = その接地力から Newton–Euler で作った
//!                                   胴体加速度 + 姿勢の PD
//!
//!   それ以外           f_grf_des  = 体重を立脚数で割った鉛直力（静的配分）
//!   （champ / linear）  a_base_des = 姿勢の PD と、脚オドメトリで測った
//!                                   胴体の位置・速度への PD
//! ```
//!
//! **後者は準静的**で、加速に伴う荷重移動も支持多角形の乗り換えも入って
//! いない。前者は MPC が予測窓（既定 10 段 × 30 ms = 300 ms）の中で解いた
//! 結果なので、そこが入る。どちらを使ったかは
//! [`WbcStatus::mpc_driven`] に出る。
//!
//! # 脚オドメトリ
//!
//! 胴体の位置も速度も直接は測れないが、**接地足に対する相対量なら測れる**
//! （[`crate::estimator`]）。「接地していると歩容が言っている足は世界に対して
//! 止まっている」という仮定だけを使う:
//!
//! - 速度 — `J_lin·v = 0` を並進 3 成分について最小二乗で解く
//! - 位置 — 歩容が計画した足位置と実測の足位置の差
//!
//! 転ぶかどうかに効くのは支持足に対する相対位置だけなので、世界座標の
//! 絶対位置が出せなくても足りる。
//!
//! # 3 つの出力の性格が違うこと
//!
//! 同じ解を出すが、**実機に効く経路は同じではない**。
//!
//! - **位置**（既定・推奨）— `q* = q_計画 + ½·q̈·dt²` と `torque_nm`。
//!   関節の位置 PD が歩容の目標を追い、WBC の τ はその上に**前置として
//!   足される**。これは legged_control の hybrid joint と同じ形で、
//!   **articara が namiashi の WBC を検証したときの構成でもある**
//!   （`articara/tests/wbc_walk.rs`）。
//! - **速度** — `q̇* = q̇_計画 + q̈·dt + kp·(q_計画 − q_実測)` と `torque_nm`。
//! - **トルク** — τ だけ。位置のループを完全に外す。
//!
//! # なぜ位置が推奨なのか（トルクではなく）
//!
//! **WBC が出すのは加速度であって位置ではなく、位置の誤差を戻す積分器が
//! どこにも無い。** τ だけで駆動すると、モデル誤差のぶん関節がゆっくり
//! ずれ続ける — 静止立位で高さが 0.6 mm/s 沈むのがそれで、歩けば脚が
//! 畳まれていく。位置 PD を残せばそれが積分器の役をする。
//!
//! articara も同じ結論に至っていて、当初の `set_wbc_torques`（PD を完全に
//! 外す経路）から hybrid へ移している。**この crate の `torque` 出力は
//! その外した側**で、残してあるのは τ を直接出せる機体で比べられるように
//! するため。
//!
//! 参照を自由に積分して位置・速度の参照を作る形も試したが、遊脚 PD の
//! 加速度を二重積分すると参照が実測から離れ、離れたぶんだけ加速度が増える
//! 正の帰還になって発散した（MuJoCo の crawl で追従誤差 7.5 rad、
//! t=5.7 s で転倒）。**積むのは 1 周期ぶんだけ**で、錨は毎周期歩容の
//! 目標へ戻す。
//!
//! # どこまで動くか（2026-09-06、MuJoCo・同梱の namiashi・16 s）
//!
//! **歩容の詰め方でまるで変わる。** 進む量だけを見て WBC の良し悪しを
//! 判断しないこと（[`crate::config::GaitTuning::step_length_m`]）。
//!
//! articara が詰めた歩容の値（`step_length_m = 0.145`、周期 trot 0.320 /
//! walk 0.500 / crawl 0.800、`swing_height_m = 0.04`、
//! `mpc_capture_point_gain_s = 0`）で:
//!
//! | 歩容 | 指令 | WBC 無効 | 位置 | 位置 + MPC | トルク |
//! |---|---|---|---|---|---|
//! | trot | 0.80 m/s | +9.302 m | +9.469 m | **+9.498 m** (95 %) | +0.284 m |
//! | walk | 0.33 m/s | +3.830 m | +3.882 m | **+3.890 m** (94 %) | +0.290 m |
//! | crawl | 0.17 m/s | +2.017 m | +2.204 m | **+2.209 m** (104 %) | +0.230 m |
//!
//! （括弧は指令に対する追従率。転倒はどれも無し。）
//!
//! 同じ機体を**歩容の既定値のまま**（歩幅 0.06/0.08/0.10 m）0.05 m/s で
//! 走らせると:
//!
//! | 歩容 | WBC 無効 | 位置 | 位置 + MPC | トルク |
//! |---|---|---|---|---|
//! | crawl | +0.388 m | +0.436 m | +0.476 m | +0.221 m |
//! | trot | +0.496 m | +0.590 m | +0.659 m | 4.5 s で転倒 |
//!
//! crawl の既定は `歩幅 / (周期 × 接地比)` = 0.042 m/s しか出せないので、
//! **0.05 m/s の指令はそもそも届かない**。
//!
//! - **位置出力は素の歩容より良い。** 詰めた設定でも +0.2〜9 %、既定でも
//!   +12〜19 %。
//! - **MPC は位置出力をさらに少し良くする。** 詰めた設定では差が小さいが、
//!   既定の crawl ではヨーのずれが 19.4° → 8.9° と目に見えて減る。
//! - **トルク出力は歩けない。** 詰めた設定でも 2〜13 % しか進まない
//!   （転びはしない）。理由は上の「なぜ位置が推奨なのか」。
//! - **速度出力は詰め切れていない。** シムのアクチュエータのゲイン
//!   （`--kv-velocity`）に強く依る。
//!
//! # 調べ方
//!
//! `MISA_WBC_DEBUG=1` を付けると毎周期 1 行出る（参照の出どころ・接地・
//! 脚オドメトリの高さと位置誤差・姿勢・角速度・`a_base_des`・解の胴体
//! 加速度・接地力の合計、それとトルクが定格の 9 割を超えた軸）。
//! **要求した胴体加速度と解の胴体加速度が一致していれば QP は仕事をして
//! いる**ので、そこが合っていて実機が付いてこないならモデルか飽和を疑う。

use std::collections::BTreeMap;

use nalgebra as na;

use misarta::joint::JointType;
use misarta::model::{Model, ModelBuilder};
use quadruped_gait::wbc::{self, WbcDims, WbcInputs, WbcWarmStart, WbcWeights};

use crate::config::{WbcConfig, WbcOutput};
use crate::jointvec::JointVec;
use crate::robot::Robot;

/// 1 軸ぶんの出力。**3 つとも埋める。**
///
/// 使われるのはモードが選んだ 1 つだけだが、残りも埋めておくのは
/// [`misa_core::AxisCommand`] と同じ理由 — モードを跨いだ切り替えで
/// 前回値が消えると、切り替えた瞬間に指令が飛ぶ。記録と可視化も
/// 「トルク制御中の関節がどこを狙っていたか」を読めたほうがよい。
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AxisPlan {
    pub position_rad: f64,
    pub velocity_rad_s: f64,
    pub torque_nm: f64,
}

/// 1 周期ぶんの WBC 出力。**脚 12 軸だけ**。
///
/// 補助軸（チキンヘッドの腕）を含めないのは、あちらが胴体の傾きを打ち消す
/// という別の目的で動いているため。WBC のモデルには入っていて重力補償
/// トルクは出るが、指令は従来どおりチキンヘッドが出す。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WbcPlan {
    /// 脚 12 軸へ与える制御モード。
    pub mode: misa_core::ControlMode,
    /// `[leg][hip, thigh, calf]`。
    pub legs: [[AxisPlan; 3]; 4],
    pub status: WbcStatus,
}

/// 解の様子。**表示と記録のためだけ**で、制御には使わない。
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WbcStatus {
    /// 立脚と見なした脚の数。
    pub stance_count: usize,
    /// τ の絶対値の最大 [N·m]。上限に張り付いていたら、モデルか歩容が
    /// 機体に合っていない。
    pub tau_max_nm: f64,
    /// 解が出した接地力の鉛直成分の合計 [N]。体重（`m·g`）と大きく違えば
    /// 参照か接地フラグが疑わしい。
    pub f_z_total_n: f64,
    /// 参照が MPC 由来か。**`false` なら準静的な自前の参照**で、MPC 歩容を
    /// 選んだつもりで CHAMP のままだった、というのがここで分かる。
    pub mpc_driven: bool,
}

/// MPC 歩容が出した参照。**あれば準静的な自前の参照より優先する。**
///
/// これが [`crate::wbc`] の module doc で「MPC を入れるならここを差し替える」
/// と言っていた差し替え先そのもの。中身は
/// `quadruped_gait::srbd_mpc::predicted_base_accel_world`（と重心版）が
/// MPC の解いた接地力から Newton–Euler で作る:
///
/// ```text
///   p̈ = (Σf)/m + g
///   α = I⁻¹·(Σ(r_i − p_胴体) × f_i − ω × I·ω)
/// ```
///
/// **自前の参照との違いは「先を見ているか」。** こちらは MPC が
/// 予測窓（既定 10 段 × 30 ms = 300 ms）の中で解いた接地力で、加速に伴う
/// 荷重移動も支持多角形の乗り換えも入っている。自前のほうは静的配分と
/// 姿勢 PD しか無い。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MpcReference {
    /// 接地力の予測 [N]（世界座標系、FL / FR / RL / RR）。
    pub grf_world: [na::Vector3<f64>; 4],
    /// 胴体の並進加速度の参照 [m/s²]（世界座標系）。
    pub accel_lin_world: na::Vector3<f64>,
    /// 胴体の角加速度の参照 [rad/s²]（世界座標系）。
    pub accel_ang_world: na::Vector3<f64>,
    /// MPC の QP が収束したか。**収束していない解は参照にしない。**
    pub solved: bool,
}

/// 1 周期ぶんの観測。**WBC が要るものを全部ここに集める。**
pub struct WbcObservation<'a> {
    /// 実測角（モデル座標系）。
    pub measured_q: &'a JointVec,
    /// 実測角速度（モデル座標系）。読めない軸は 0。
    pub measured_qd: &'a JointVec,
    /// 歩容の IK 出力（モデル座標系）。遊脚 PD の目標。
    pub target_q: &'a JointVec,
    /// IMU の姿勢 `[roll, pitch, yaw]` [rad]。
    pub attitude_rad: [f64; 3],
    /// IMU の角速度 [rad/s]。**胴体座標系**。
    pub gyro_rad_s: [f64; 3],
    /// 歩容が計画しているヨー角 [rad]（`BodyState::world_yaw`）。
    pub planned_yaw_rad: f64,
    /// 胴体速度の指令 `[vx, vy, wz]`（胴体座標系）。**水平の目標速度。**
    pub body_velocity: [f64; 3],
    /// 脚オドメトリで測った胴体の状態（[`crate::estimator`]）。
    ///
    /// **WBC は推定を自分で持たない。** 同じ推定を MPC 歩容も使うので、
    /// 出どころを 1 か所にしてある。
    pub body: crate::estimator::BodyState,
    /// MPC 歩容の参照。CHAMP / LinearCrawl では `None`。
    pub mpc: Option<MpcReference>,
    /// 立脚フラグ（FL, FR, RL, RR）。
    pub stance: [bool; 4],
    /// 制御周期 [s]。**実測を渡すこと**（積分の刻みになる）。
    pub dt: f64,
}

/// 浮遊ベースのモデルと、その上で回す QP。
pub struct WbcLayer {
    cfg: WbcConfig,
    /// 浮遊ベースのモデル。`joints[1]` が FreeFlyer。
    model: Model<f64>,
    nv: usize,
    /// アクチュエータ数（`nv - 6`）。
    na: usize,
    /// 脚 12 軸 → 新モデルの v index。並びは [`JointVec`] と同じ。
    leg_v_idx: [[usize; 3]; 4],
    /// 同じく q index。**v から +1 で引かない。** FreeFlyer は nq 7 / nv 6 と
    /// 幅が違うので、たまたま揃っているだけの関係に依るとモデルの形が
    /// 変わった瞬間に黙って壊れる。
    leg_q_idx: [[usize; 3]; 4],
    /// 足リンクに対応する新モデルの joint index（FL, FR, RL, RR）。
    foot_joint: [usize; 4],
    /// 補助軸（腕）の v / q index。モデルに無ければ `None`。
    aux_v_idx: Option<usize>,
    aux_q_idx: Option<usize>,
    /// アクチュエータごとのトルク上限 [N·m]。
    torque_max: na::DVector<f64>,
    weights: WbcWeights,
    /// 前周期の解。QP の warm start に使う。
    x_prev: Option<na::DVector<f64>>,
    /// 鈍らせた接地力の参照（MPC 出力のとき）。
    smoothed_grf: [na::Vector3<f64>; 4],
    grf_seeded: bool,
    /// 機体質量 [kg]。接地力の静的配分に使う。
    mass_kg: f64,
    /// 前周期の歩容目標。`q̇_計画` の差分に使う。**全軸・毎周期更新。**
    /// 遊脚タスク用の [`Self::last_swing_target`] とは更新の仕方が違う。
    last_plan: [[f64; 3]; 4],
    /// 参照とヨー基準を張り直す必要があるか。
    seeded: bool,
    /// IMU のヨーと歩容のヨーの差。**原点が違う**ので、噛み合わせるのに要る。
    yaw_offset: f64,
    /// 前周期の遊脚目標。遊脚タスクの `q̈*` に入る `q̇*` の差分に使う。
    /// **立脚中は更新しない**（更新すると、次に遊脚へ入った瞬間に
    /// 「立脚保持 → 遊脚開始」の跳びがそのまま `q̇*` になる）。
    last_swing_target: [[f64; 3]; 4],
    /// 一度も解けていないことを 1 度だけ言うためのフラグ。
    warned_infeasible: bool,
}

impl WbcLayer {
    /// モデルから組み立てる。**実機に触れない**ので `check` でも作れる。
    pub fn new(robot: &Robot, cfg: &WbcConfig) -> Result<Self, String> {
        let model = build_floating_base_model(&robot.model);
        let nv = model.nv;
        if nv < 6 {
            return Err("浮遊ベースのモデルに 6 自由度の胴体がありません".into());
        }
        let na = nv - 6;

        // 関節名 → 新モデルの joint index。
        let by_name: BTreeMap<&str, usize> = model
            .joints
            .iter()
            .enumerate()
            .skip(1)
            .map(|(i, j)| (j.name.as_str(), i))
            .collect();

        let mut leg_v_idx = [[0usize; 3]; 4];
        let mut leg_q_idx = [[0usize; 3]; 4];
        for (leg, names) in misa_hal::joint::JOINT_NAMES.iter().enumerate() {
            for (k, name) in names.iter().enumerate() {
                let mi = *by_name
                    .get(name)
                    .ok_or_else(|| format!("関節 {name} がモデルにありません"))?;
                if model.joints[mi].joint_type.nv() != 1 {
                    return Err(format!("関節 {name} が 1 自由度ではありません"));
                }
                let vi = model.v_idx[mi];
                if vi < 6 {
                    return Err(format!("関節 {name} が胴体の 6 自由度と重なっています"));
                }
                leg_v_idx[leg][k] = vi;
                leg_q_idx[leg][k] = model.q_idx[mi];
            }
        }

        // 足リンク。**リンク名で引く**（`link_names[i]` は joints[i] の子リンク）。
        let mut foot_joint = [0usize; 4];
        for (slot, (_, link)) in quadruped_gait::DEFAULT_FOOT_LINKS.iter().enumerate() {
            let idx = model
                .link_names
                .iter()
                .position(|n| n == link)
                .ok_or_else(|| format!("足リンク {link} がモデルにありません"))?;
            if idx == 0 {
                return Err(format!("足リンク {link} が根リンクになっています"));
            }
            foot_joint[slot] = idx;
        }

        // トルク上限。宣言が無ければ 0 = 制限なしになってしまうので、
        // **WBC を回す以上は必ず何か入れる**（無ければ設定の値を要求する）。
        let mut torque_max = na::DVector::from_element(na, 0.0);
        for (i, j) in model.joints.iter().enumerate().skip(1) {
            if j.joint_type.nv() != 1 {
                continue;
            }
            let vi = model.v_idx[i];
            if vi < 6 {
                continue;
            }
            let declared = robot.effort_limits.get(&j.name).copied().unwrap_or(0.0);
            let limit = cfg.torque_ceiling(declared);
            if limit <= 0.0 {
                return Err(format!(
                    "関節 {} にトルクの定格がありません。\
                     モデルの [joint.limit] effort を書くか wbc.max_torque_nm を指定してください",
                    j.name
                ));
            }
            torque_max[vi - 6] = limit;
        }

        // 補助軸（腕）。**モデルにはアクチュエータとして入っている**が、
        // 指令を出すのはチキンヘッドで WBC ではない。位置で保持されている
        // 関節は、WBC から見ればトルク源ではなく拘束に近い — その食い違いを
        // 埋めるために、関節タスクだけ置いて τ を実機に近づける
        // （[`WbcLayer::solve`] の関節空間タスク）。
        let aux = by_name
            .get(misa_hal::joint::ARM_JOINT_NAME)
            .copied()
            .filter(|&i| model.joints[i].joint_type.nv() == 1 && model.v_idx[i] >= 6);
        let aux_v_idx = aux.map(|i| model.v_idx[i]);
        let aux_q_idx = aux.map(|i| model.q_idx[i]);

        let mass_kg = if cfg.mass_kg > 0.0 {
            cfg.mass_kg
        } else {
            model.inertias.iter().map(|i| i.mass).sum::<f64>()
        };
        if mass_kg <= 0.0 {
            return Err("機体質量が 0 です。モデルの慣性か wbc.mass_kg を確かめてください".into());
        }

        Ok(Self {
            cfg: cfg.clone(),
            model,
            nv,
            na,
            leg_v_idx,
            leg_q_idx,
            foot_joint,
            aux_v_idx,
            aux_q_idx,
            torque_max,
            weights: WbcWeights {
                base_accel: cfg.weight_base_accel,
                swing_leg: cfg.weight_joint_track,
                contact_force: cfg.weight_contact_force,
                tau_gravity: cfg.weight_tau_gravity,
                ..WbcWeights::default()
            },
            x_prev: None,
            smoothed_grf: [na::Vector3::zeros(); 4],
            grf_seeded: false,
            mass_kg,
            last_plan: [[0.0; 3]; 4],
            seeded: false,
            yaw_offset: 0.0,
            last_swing_target: [[0.0; 3]; 4],
            warned_infeasible: false,
        })
    }

    pub fn output(&self) -> WbcOutput {
        self.cfg.output
    }

    pub fn mass_kg(&self) -> f64 {
        self.mass_kg
    }

    /// 積分した参照とヨー基準を捨てる。
    ///
    /// **WBC を抜けたら必ず呼ぶ。** 覚えたままだと、次に入った瞬間に
    /// 「抜けたときの姿勢」から指令が出て脚が飛ぶ。
    pub fn reset(&mut self) {
        self.seeded = false;
        self.x_prev = None;
        self.grf_seeded = false;
    }

    /// 1 周期解く。
    pub fn solve(&mut self, obs: &WbcObservation) -> WbcPlan {
        let [roll, pitch, yaw] = obs.attitude_rad;
        if !self.seeded {
            self.last_plan = obs.target_q.legs;
            self.last_swing_target = obs.target_q.legs;
            // **IMU と歩容でヨーの原点が違う。** 歩容は 0 から積分した
            // 値を持ち、IMU は電源を入れたときの向きが 0。入った瞬間の差を
            // 覚えて噛み合わせる。
            self.yaw_offset = yaw - obs.planned_yaw_rad;
            self.seeded = true;
        }

        // ── q, v を組む ──────────────────────────────────────
        // FreeFlyer の q は [x, y, z, qx, qy, qz, qw]。**位置は 0 でよい**
        // （misarta に地面は無いので、動力学は姿勢にしか依らない）。
        // オドメトリが無いので、そもそも入れられる値も無い。
        let quat = na::UnitQuaternion::from_euler_angles(roll, pitch, yaw);
        let mut q = self.model.neutral_q();
        q[0] = 0.0;
        q[1] = 0.0;
        q[2] = 0.0;
        q[3] = quat.i;
        q[4] = quat.j;
        q[5] = quat.k;
        q[6] = quat.w;
        for leg in 0..4 {
            for k in 0..3 {
                q[self.leg_q_idx[leg][k]] = obs.measured_q.legs[leg][k];
            }
        }
        // 腕（補助軸）もモデルに入っている。重力補償トルクが正しく出るよう
        // 実測を入れる。**指令は出さない**（チキンヘッドの担当）。
        self.write_aux_q(&mut q, obs.measured_q);

        // v の胴体 6 行は**胴体座標系で [角速度; 並進速度]**（Featherstone）。
        //
        // 角速度は IMU のジャイロ（実測）。並進速度はこのすぐ下で
        // 脚オドメトリ（[`crate::estimator`]）から埋める。
        let mut v = vec![0.0f64; self.nv];
        v[0] = obs.gyro_rad_s[0];
        v[1] = obs.gyro_rad_s[1];
        v[2] = obs.gyro_rad_s[2];
        for leg in 0..4 {
            for k in 0..3 {
                let vi = self.leg_v_idx[leg][k];
                // **関節速度は頭打ちにする。** 接地の過渡で異常な q̇ が入ると
                // `J̇·v` が跳ね、それに見合う q̈ を要求して発散する。
                v[vi] = obs.measured_qd.legs[leg][k].clamp(-JOINT_V_MAX, JOINT_V_MAX);
            }
        }

        // ── 接触ヤコビアン（線形成分だけ）と J̇·v ─────────────
        //
        // FK は 1 回だけ回して足 4 本で使い回す（`compute_joint_jacobian` は
        // 呼ぶたびに内部で FK する）。
        //
        // **`v` の胴体並進 3 行は脚オドメトリで埋める。** 0 のままにすると
        // 「静止した胴体を支える」問題になり、立脚が胴体を送る動きが
        // `J̇·v` から消える。
        if let Some(v_world) = obs.body.velocity_world {
            let v_body = quat.inverse() * v_world;
            v[3] = v_body.x;
            v[4] = v_body.y;
            v[5] = v_body.z;
        }
        let data = misarta::fk::forward_kinematics(&self.model, &q);
        let mut j_contact = na::DMatrix::zeros(12, self.nv);
        let mut dj_v = na::DVector::zeros(12);
        for slot in 0..4 {
            let mi = self.foot_joint[slot];
            let j_full =
                misarta::jacobian::compute_joint_jacobian_from_data(&self.model, &q, &data, mi);
            let bias = misarta::jacobian::compute_jacobian_dot_times_v(&self.model, &q, &v, mi);
            // misarta の空間ヤコビアンの行は [角 (0..3); 線形 (3..6)]。
            // 接触が要るのは足先の**並進**速度なので 3..6 を取る。
            for r in 0..3 {
                for c in 0..self.nv {
                    j_contact[(3 * slot + r, c)] = j_full[(3 + r, c)];
                }
                dj_v[3 * slot + r] = bias[3 + r];
            }
        }

        // ── M, h ────────────────────────────────────────────
        // **`v` が確定してから。** どちらも速度に依る。
        let mass = misarta::crba::crba(&self.model, &q);
        let nle = misarta::rnea::nonlinear_effects(&self.model, &q, &v);

        // ── 参照 ────────────────────────────────────────────
        let r_wb = quat.to_rotation_matrix();
        let refs = self.references(obs, &r_wb);
        let mpc_driven = obs.mpc.is_some_and(|m| m.solved);

        // ── 関節空間の追従タスク（遊脚 + 立脚）────────────────
        //
        // `quadruped_gait::wbc` はこれを「遊脚タスク」と呼ぶが、中身は
        // アクチュエータごとの `q̈*` と有効フラグでしかないので、立脚にも
        // そのまま使える。**立脚にも置くのは、足が滑らないという制約だけ
        // では歩容の計画を通らないから**（[`crate::config::WbcConfig::stance_kp`]）。
        let mut joint_q_ddot = na::DVector::zeros(self.na);
        let mut joint_flag = vec![false; self.na];
        for leg in 0..4 {
            let (kp, kd) = if obs.stance[leg] {
                (self.cfg.stance_kp, self.cfg.stance_kd)
            } else {
                (self.cfg.swing_kp, self.cfg.swing_kd)
            };
            if kp <= 0.0 && kd <= 0.0 {
                continue;
            }
            for k in 0..3 {
                let vi = self.leg_v_idx[leg][k];
                let target = obs.target_q.legs[leg][k];
                // **目標速度の差分は立脚・遊脚で分けて持つ。** まとめると、
                // 相が変わった周期に「保持していた角 → 新しい相の角」の
                // 跳びがそのまま `q̇*` になる。
                let qd_target = if obs.dt > 1e-6 {
                    (target - self.last_swing_target[leg][k]) / obs.dt
                } else {
                    0.0
                };
                self.last_swing_target[leg][k] = target;
                let q_meas = obs.measured_q.legs[leg][k];
                let qd_meas = obs.measured_qd.legs[leg][k];
                joint_q_ddot[vi - 6] = kp * (target - q_meas) + kd * (qd_target - qd_meas);
                joint_flag[vi - 6] = true;
            }
        }
        // 補助軸（腕）。**実機では位置で保持されている**ので、WBC にも
        // 「そこに留まる」と伝える。伝えないと QP は腕を自由なトルク源と
        // 見なし、胴体のピッチを腕の反力で作る解を選ぶ。実際にはサーボが
        // 位置を保つのでその反力は出ず、その差がピッチのずれとして残る。
        if let Some(vi) = self.aux_v_idx {
            joint_q_ddot[vi - 6] = self.cfg.swing_kp * (obs.target_q.arm - obs.measured_q.arm)
                + self.cfg.swing_kd * (0.0 - obs.measured_qd.arm);
            joint_flag[vi - 6] = true;
        }

        // ── 重力補償トルク（τ ≈ 0 へ潰れるのを止める錨）────────
        let g_full = misarta::rnea::compute_gravity(&self.model, &q);
        let mut tau_gravity = na::DVector::zeros(self.na);
        for i in 6..self.nv {
            tau_gravity[i - 6] = g_full[i];
        }

        // ── 解く ────────────────────────────────────────────
        let dims = WbcDims {
            nv: self.nv,
            nc: 4,
            na: self.na,
        };
        let inputs = WbcInputs {
            dims,
            mass: &mass,
            nle: &nle,
            j_contact: &j_contact,
            dj_v: &dj_v,
            contact_flag: obs.stance,
            friction_mu: self.cfg.friction_mu,
            f_min_stance_n: self.cfg.f_min_stance_n,
            torque_max: &self.torque_max,
            a_base_des: &refs.a_base_des,
            swing_q_ddot_des: &joint_q_ddot,
            swing_actuator_flag: &joint_flag,
            f_grf_des: &refs.f_grf_des,
            tau_gravity: &tau_gravity,
        };
        let warm = WbcWarmStart {
            x_prev: self.x_prev.as_ref(),
            prox_weight: self.cfg.prox_weight,
        };
        let sol = wbc::solve_warm_with_weights(&inputs, &warm, &self.weights);
        self.x_prev = Some(sol.x_full.clone());

        // **NaN を実機へ出さない。** QP が壊れた周期は前回の解を捨てて
        // 重力補償トルクだけに落とす（脱力よりは崩れない）。
        let sane = sol.tau.iter().all(|t| t.is_finite())
            && sol.q_ddot.iter().all(|a| a.is_finite());
        if !sane {
            self.x_prev = None;
            if !self.warned_infeasible {
                self.warned_infeasible = true;
                log::error!(
                    "WBC の解が数値的に壊れました。重力補償トルクだけに落とします。\
                     接地フラグ・摩擦・トルク上限を確かめてください"
                );
            }
        }

        if std::env::var_os("MISA_WBC_DEBUG").is_some() {
            let f = |x: f64| (x * 100.0).round() / 100.0;
            eprintln!(
                "[wbc] {} st={}{}{}{} h={:.3} e=({:+.3},{:+.3},{:+.3}) rpy=({:+.2},{:+.2}) w=({:+.2},{:+.2},{:+.2}) a*ang=({:+.1},{:+.1}) a*lin=({:+.1},{:+.1},{:+.1}) qddb=({:+.1},{:+.1},{:+.1},{:+.1},{:+.1},{:+.1}) fz={:.1}",
                match obs.mpc {
                    Some(m) if m.solved => "MPC",
                    Some(_) => "MPC×",
                    None => "静的",
                },
                obs.stance[0] as u8, obs.stance[1] as u8, obs.stance[2] as u8, obs.stance[3] as u8,
                obs.body.height_m.unwrap_or(f64::NAN),
                obs.body.position_error_world.map(|e| e.x).unwrap_or(f64::NAN),
                obs.body.position_error_world.map(|e| e.y).unwrap_or(f64::NAN),
                obs.body.position_error_world.map(|e| e.z).unwrap_or(f64::NAN),
                roll, pitch,
                f(obs.gyro_rad_s[0]), f(obs.gyro_rad_s[1]), f(obs.gyro_rad_s[2]),
                f(refs.a_base_des[0]), f(refs.a_base_des[1]),
                f(refs.a_base_des[3]), f(refs.a_base_des[4]), f(refs.a_base_des[5]),
                f(sol.q_ddot[0]), f(sol.q_ddot[1]), f(sol.q_ddot[2]),
                f(sol.q_ddot[3]), f(sol.q_ddot[4]), f(sol.q_ddot[5]),
                (0..4).map(|i| sol.f_grf[3 * i + 2]).sum::<f64>(),
            );
            let mut sat = Vec::new();
            for (i, t) in sol.tau.iter().enumerate() {
                if t.abs() > self.torque_max[i] * 0.9 {
                    sat.push(format!("{i}:{t:+.2}/{:.2}", self.torque_max[i]));
                }
            }
            if !sat.is_empty() {
                eprintln!("      飽和 {}", sat.join(" "));
            }
        }

        let mut status = WbcStatus {
            stance_count: obs.stance.iter().filter(|s| **s).count(),
            mpc_driven,
            ..WbcStatus::default()
        };
        for slot in 0..4 {
            let fz = sol.f_grf[3 * slot + 2];
            if fz.is_finite() {
                status.f_z_total_n += fz;
            }
        }

        // ── 出力へ落とす ────────────────────────────────────
        //
        // **参照は歩容の計画に錨を下ろす。** 一度は q̈ を自由に積分して
        // 位置・速度の参照を作ったが、遊脚 PD の加速度（kp = 100）を二重
        // 積分すると参照が実測から離れ、離れたぶんだけ加速度が増える正の
        // 帰還になって発散した（MuJoCo の crawl で追従誤差 7.5 rad、
        // t=5.7 s で転倒）。積むのは**1 周期ぶんだけ**にして、錨は毎周期
        // 歩容の目標へ戻す。
        //
        // その結果、位置出力で WBC が効くのは主に `torque_nm`（前置トルク）
        // になる。**τ を受け取れない機体では、位置出力の WBC は歩容だけを
        // 流すのと変わらない**ことに注意（LKMTech の 0xA4 がそれ）。
        let mut legs = [[AxisPlan::default(); 3]; 4];
        let accel_cap = if self.cfg.max_joint_accel > 0.0 {
            self.cfg.max_joint_accel
        } else {
            f64::INFINITY
        };
        for leg in 0..4 {
            for k in 0..3 {
                let vi = self.leg_v_idx[leg][k];
                let (q_ddot, tau) = if sane {
                    (sol.q_ddot[vi], sol.tau[vi - 6])
                } else {
                    (0.0, tau_gravity[vi - 6])
                };
                status.tau_max_nm = status.tau_max_nm.max(tau.abs());

                let q_plan = obs.target_q.legs[leg][k];
                let qd_plan = if obs.dt > 1e-6 {
                    (q_plan - self.last_plan[leg][k]) / obs.dt
                } else {
                    0.0
                };
                self.last_plan[leg][k] = q_plan;

                let q_meas = obs.measured_q.legs[leg][k];
                // **参照に積む q̈ は頭打ちにする。** MPC の参照では姿勢が
                // 崩れ始めた瞬間に 600 rad/s² が出る（慣性が小さいため）。
                // 速度出力ではそれが 1 周期ぶんそのまま指令の跳びになり、
                // 脚が飛ぶ。
                let a = q_ddot.clamp(-accel_cap, accel_cap);
                legs[leg][k] = AxisPlan {
                    // 計画から 1 周期ぶん積む。dt が小さいので寄与は小さい
                    // （q̈ = 100 rad/s², dt = 5 ms で 1.3e-3 rad）。
                    position_rad: q_plan + 0.5 * a * obs.dt * obs.dt,
                    // 速度は WBC の寄与が効く大きさになる（同じ条件で
                    // 0.5 rad/s）。位置誤差の項が無いと関節が漂うので足す。
                    velocity_rad_s: qd_plan
                        + a * obs.dt
                        + self.cfg.velocity_track_kp * (q_plan - q_meas),
                    torque_nm: tau,
                };
            }
        }

        WbcPlan {
            mode: match self.cfg.output {
                WbcOutput::Torque => misa_core::ControlMode::Torque,
                WbcOutput::Velocity => misa_core::ControlMode::Velocity,
                WbcOutput::Position => misa_core::ControlMode::Position,
            },
            legs,
            status,
        }
    }

    /// 補助軸（腕）の実測角を q へ入れる。指令は出さない。
    fn write_aux_q(&self, q: &mut [f64], measured: &JointVec) {
        if let Some(qi) = self.aux_q_idx {
            q[qi] = measured.arm;
        }
    }

    /// `a_base_des` と `f_grf_des`。**ここが準静的な参照そのもの。**
    fn references(&mut self, obs: &WbcObservation, r_wb: &na::Rotation3<f64>) -> References {
        // **MPC の参照があればそちらを使う。** 収束していない解は使わない
        // （制約を満たさない接地力を追いかけることになる）。
        let mut refs = if let Some(mpc) = obs.mpc.filter(|m| m.solved) {
            self.mpc_references(obs, r_wb, &mpc)
        } else {
            self.quasi_static_references(obs, r_wb)
        };
        self.clamp_base_accel(&mut refs.a_base_des);
        refs
    }

    /// `a_base_des` を頭打ちにする。**角と並進で別々に、向きは保つ。**
    ///
    /// 成分ごとに切ると要求の向きが変わる（ピッチだけ切ればロールとの比が
    /// 崩れる）ので、ノルムで縮める。理由は
    /// [`crate::config::WbcConfig::max_base_accel_lin`]。
    fn clamp_base_accel(&self, a: &mut na::DVector<f64>) {
        let mut scale = |from: usize, limit: f64| {
            if limit <= 0.0 {
                return;
            }
            let v = na::Vector3::new(a[from], a[from + 1], a[from + 2]);
            let n = v.norm();
            if n > limit {
                let k = limit / n;
                for i in 0..3 {
                    a[from + i] *= k;
                }
            }
        };
        scale(0, self.cfg.max_base_accel_ang);
        scale(3, self.cfg.max_base_accel_lin);
    }

    /// MPC の解から作る参照。
    ///
    /// 姿勢の PD は**足したまま残す**。MPC の角加速度は接地力から
    /// Newton–Euler で作った純粋な前置きで、姿勢の誤差を直接見ていない。
    /// `attitude_kp` を 0 にすれば前置きだけになる。
    fn mpc_references(
        &mut self,
        obs: &WbcObservation,
        r_wb: &na::Rotation3<f64>,
        mpc: &MpcReference,
    ) -> References {
        let c = &self.cfg;
        let [roll, pitch, _yaw] = obs.attitude_rad;
        let w = obs.gyro_rad_s;

        // **接地力の予測は周期ごとに震える。** clarabel は広い零空間から
        // 少しずつ違う最適解を拾う（articara の実測で 13 → 68 → 47 N）。
        // 参照だけを鈍らせる — τ の前置きに使う生の値は歩容側が持っている。
        let alpha = c.grf_smoothing.clamp(0.0, 1.0);
        if !self.grf_seeded || alpha >= 1.0 {
            self.smoothed_grf = mpc.grf_world;
            self.grf_seeded = true;
        } else {
            for slot in 0..4 {
                self.smoothed_grf[slot] =
                    alpha * mpc.grf_world[slot] + (1.0 - alpha) * self.smoothed_grf[slot];
            }
        }
        let mut f_grf_des = na::DVector::zeros(12);
        for slot in 0..4 {
            for k in 0..3 {
                f_grf_des[3 * slot + k] = self.smoothed_grf[slot][k];
            }
        }

        // q̈ の胴体 6 行は胴体座標系。MPC は世界座標で出すので回す。
        let a_ang_body = r_wb.transpose() * mpc.accel_ang_world;
        let a_lin_body = r_wb.transpose() * mpc.accel_lin_world;
        let a_base_des = na::DVector::from_iterator(
            6,
            [
                a_ang_body.x + c.attitude_kp * (0.0 - roll) - c.attitude_kd * w[0],
                a_ang_body.y + c.attitude_kp * (0.0 - pitch) - c.attitude_kd * w[1],
                a_ang_body.z,
                a_lin_body.x,
                a_lin_body.y,
                a_lin_body.z,
            ],
        );
        References {
            a_base_des,
            f_grf_des,
        }
    }

    /// MPC が無いときの参照。**準静的。**
    fn quasi_static_references(
        &self,
        obs: &WbcObservation,
        r_wb: &na::Rotation3<f64>,
    ) -> References {
        let [roll, pitch, yaw] = obs.attitude_rad;
        let w = obs.gyro_rad_s;
        let c = &self.cfg;

        // 姿勢は水平が目標。ヨーだけは歩容が回す計画を持っているので、
        // 入った瞬間の差（`yaw_offset`）を足して噛み合わせる。
        let yaw_ref = self.yaw_offset + obs.planned_yaw_rad;
        let yaw_err = wrap_pi(yaw_ref - yaw);
        // **胴体座標系の角加速度**。roll / pitch の誤差は世界座標のオイラー
        // 角だが、傾きが小さい範囲では胴体座標の x / y 成分とみなせる。
        // `gait.body_attitude_max_rad`（0.6 rad）を超えて傾ける運用では
        // ここを回転で書き直すこと。
        let a_ang = na::Vector3::new(
            c.attitude_kp * (0.0 - roll) - c.attitude_kd * w[0],
            c.attitude_kp * (0.0 - pitch) - c.attitude_kd * w[1],
            c.yaw_kp * yaw_err - c.yaw_kd * w[2],
        );
        // 並進は脚オドメトリの推定に対する PD。**高さは位置と速度、水平は
        // 速度だけ。** 水平の位置は積分でしか出せず、滑りのぶんが溜まる
        // ので見ない（見ると、滑った先を「正しい位置」として保持しに行く）。
        //
        // 接地足が 1 本も無い相（跳躍）では推定が立たないので、そのときは
        // 「加速するな」に落とす。crawl と trot では起きない。
        let a_lin_world = match (obs.body.position_error_world, obs.body.velocity_world) {
            (Some(e), Some(v_est)) => {
                // 目標速度は指令。胴体座標なので世界へ回す。
                let v_ref =
                    r_wb * na::Vector3::new(obs.body_velocity[0], obs.body_velocity[1], 0.0);
                na::Vector3::new(
                    c.position_kp * e.x + c.velocity_kd * (v_ref.x - v_est.x),
                    c.position_kp * e.y + c.velocity_kd * (v_ref.y - v_est.y),
                    c.height_kp * e.z + c.height_kd * (v_ref.z - v_est.z),
                )
            }
            _ => na::Vector3::zeros(),
        };
        // q̈ の胴体 6 行は**胴体座標系**なので回してから入れる。
        let a_lin = r_wb.transpose() * a_lin_world;

        let a_base_des = na::DVector::from_iterator(
            6,
            [a_ang.x, a_ang.y, a_ang.z, a_lin.x, a_lin.y, a_lin.z],
        );

        // 接地力は体重の静的配分。**世界座標系の鉛直方向**。
        //
        // # 等分でよい理由
        //
        // 厳密には「重心まわりの水平モーメントが 0 になる配分」が正しく、
        // 等分はそれと重心のずれのぶんだけ違う。namiashi で測ると重心は
        // 胴体原点から (1.5, 4.7, 0.3) mm — 足の間隔（±147 / ±109 mm）に
        // 対して 3 % 以下で、モーメントを 0 にする配分を最小二乗で解く版を
        // 書いて MuJoCo で比べても**歩き方は 1 mm も変わらなかった**
        // （LinearCrawl・40 秒）。重心が明確にずれる機体（荷物を積む、
        // 腕が重い）ではここを書き換えること。
        let stance = obs.stance.iter().filter(|s| **s).count();
        let mut f_grf_des = na::DVector::zeros(12);
        if stance > 0 {
            let share = self.mass_kg * G / stance as f64;
            for slot in 0..4 {
                if obs.stance[slot] {
                    f_grf_des[3 * slot + 2] = share;
                }
            }
        }
        References {
            a_base_des,
            f_grf_des,
        }
    }
}

/// 制御ループから見た WBC。**Active のときだけ解く。**
///
/// 歩容が回っていない状態（脱力・遷移中・ポーズ再生中）で解かないのは、
/// 立脚フラグが最後に分かった値のまま止まっているため。接地していない足を
/// 接地と信じて接地力を配分すると、支えの無い方向へ胴体を押す。
///
/// ```text
///   controller.tick(..)  →  ControlOutput（歩容の目標 + 立脚フラグ）
///                              ↓
///   WbcRunner::tick(..)  →  Option<WbcPlan>（脚 12 軸ぶんの指令）
///                              ↓
///   snapshot::command(.., wbc)  →  SafetyGate  →  Plant
/// ```
pub struct WbcRunner {
    layer: WbcLayer,
    /// 直前の周期で解いたか。**抜けた瞬間に参照を捨てる**ため。
    was_active: bool,
}

impl WbcRunner {
    /// 設定が有効なら組み立てる。無効なら `None`。
    ///
    /// **モデルの不備はここで落とす。** 足リンクが無い・トルクの定格が
    /// 無いといった話は、制御ループが回り始めてから気づいても遅い。
    pub fn new(robot: &Robot, cfg: &WbcConfig) -> Result<Option<Self>, String> {
        if !cfg.enabled {
            return Ok(None);
        }
        let layer = WbcLayer::new(robot, cfg)?;
        log::info!(
            "WBC を有効にしました（出力: {}、質量 {:.2} kg、摩擦 {:.2}）",
            cfg.output.label(),
            layer.mass_kg(),
            cfg.friction_mu
        );
        Ok(Some(Self {
            layer,
            was_active: false,
        }))
    }

    pub fn output(&self) -> WbcOutput {
        self.layer.output()
    }

    /// この出力が実機へ要求する制御モード。**Plant の能力と突き合わせる**
    /// のに使う（扱えないモードで指令を出すと、黙って別の意味になる）。
    pub fn control_mode(&self) -> misa_core::ControlMode {
        match self.output() {
            WbcOutput::Torque => misa_core::ControlMode::Torque,
            WbcOutput::Velocity => misa_core::ControlMode::Velocity,
            WbcOutput::Position => misa_core::ControlMode::Position,
        }
    }

    /// モデルから求めた機体質量 [kg]。`check` の表示に使う。
    pub fn mass_kg(&self) -> f64 {
        self.layer.mass_kg()
    }

    /// 1 周期。`out` は同じ周期の [`crate::controller::Controller::tick`] の
    /// 戻り値、`obs` はそれを計算するのに使った観測。
    pub fn tick(
        &mut self,
        out: &crate::controller::ControlOutput,
        obs: &misa_core::Observation,
        measured_q: &JointVec,
        measured_qd: &JointVec,
        body: &crate::estimator::BodyState,
        dt: f64,
    ) -> Option<WbcPlan> {
        let active = out.state == crate::controller::State::Active;
        if !active {
            if self.was_active {
                self.layer.reset();
                self.was_active = false;
            }
            return None;
        }
        self.was_active = true;
        let imu = obs.imu;
        let plan = self.layer.solve(&WbcObservation {
            measured_q,
            measured_qd,
            target_q: &out.targets,
            attitude_rad: imu.map(|i| i.rpy_rad).unwrap_or([0.0; 3]),
            gyro_rad_s: imu.map(|i| i.gyro_rad_s).unwrap_or([0.0; 3]),
            planned_yaw_rad: out.planned_yaw_rad,
            body_velocity: out.body_velocity,
            body: *body,
            mpc: out.mpc,
            stance: out.stance,
            dt,
        });
        Some(plan)
    }
}

struct References {
    a_base_des: na::DVector<f64>,
    f_grf_des: na::DVector<f64>,
}

/// 重力 [m/s²]。
const G: f64 = 9.806_65;

/// 関節速度の頭打ち [rad/s]。
///
/// 接地の過渡でモータが弾かれると `J̇·v` が跳ね、それを打ち消す q̈ を
/// 要求して発散する。実運用の歩容が要求する速さ（定格 33.5 rad/s の
/// 1 割にも満たない）より十分上に取ってあるので、普通に歩いている間は
/// 当たらない。
const JOINT_V_MAX: f64 = 10.0;

fn wrap_pi(x: f64) -> f64 {
    (x + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI) - std::f64::consts::PI
}

/// 固定ベースのモデルから、universe と胴体の間に `FreeFlyer` を挟んだ
/// モデルを建て直す。
///
/// 元の `joints[i]`（`i >= 1`）は新モデルの `i + 1` になり、親の index は
/// 一律 +1 される（元の親 0 = universe は、新しい FreeFlyer である 1 を指す）。
/// **元のモデルが親を先に並べている**（`parent[i] < i`）ことに依っており、
/// `misarta::native::build_model` はそう作る。
pub fn build_floating_base_model(fixed: &Model<f64>) -> Model<f64> {
    let root_link = fixed.link_names.first().cloned().unwrap_or_default();
    let mut b = ModelBuilder::<f64>::new()
        .name(fixed.name.clone())
        .root_link_name("universe".to_string())
        .gravity(fixed.gravity);

    // 胴体は FreeFlyer の子。慣性は元の根リンクのもの。
    b = b.add_joint_with_link(
        "root_freeflyer",
        0,
        JointType::FreeFlyer,
        misarta::se3::identity(),
        fixed.inertias[0].clone(),
        root_link,
    );

    for i in 1..fixed.joints.len() {
        let j = &fixed.joints[i];
        b = b.add_joint_with_link(
            j.name.clone(),
            j.parent + 1,
            j.joint_type.clone(),
            j.placement.clone(),
            fixed.inertias[i].clone(),
            fixed.link_names[i].clone(),
        );
    }
    b.build()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 「計画どおりに 4 脚で立っている」推定。**WBC の試験は QP を見る**
    /// ので、推定そのものは [`crate::estimator`] の試験に任せて固定値にする。
    fn level_stand() -> crate::estimator::BodyState {
        crate::estimator::BodyState {
            height_m: Some(0.2),
            position_error_world: Some(na::Vector3::zeros()),
            velocity_world: Some(na::Vector3::zeros()),
            angular_velocity_world: na::Vector3::zeros(),
            stance_count: 4,
        }
    }

    fn robot() -> Robot {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../models/namiashi/namiashi.misa");
        Robot::load(path, "extend").expect("同梱モデルが読めること")
    }

    /// **浮遊ベースを挟んでも運動学は変わらない。** 関節は 1 つずつ後ろへ
    /// ずれるだけで、名前・親子関係・配置はそのまま。
    #[test]
    fn the_floating_base_model_keeps_every_joint_of_the_fixed_one() {
        let r = robot();
        let fb = build_floating_base_model(&r.model);
        assert_eq!(fb.joints.len(), r.model.joints.len() + 1);
        assert_eq!(fb.nv, r.model.nv + 6);
        for i in 1..r.model.joints.len() {
            assert_eq!(fb.joints[i + 1].name, r.model.joints[i].name);
            assert_eq!(fb.joints[i + 1].parent, r.model.joints[i].parent + 1);
            assert_eq!(fb.link_names[i + 1], r.model.link_names[i]);
        }
    }

    /// **胴体の 6 行が生えていること。** 固定ベースのままだと胴体は運動
    /// 方程式に現れず、接地力が胴体を支えるという構造が表せない。
    #[test]
    fn the_floating_base_model_carries_the_body_weight_in_its_gravity_term() {
        let r = robot();
        let fb = build_floating_base_model(&r.model);
        let q = fb.neutral_q();
        let g = misarta::rnea::compute_gravity(&fb, &q);
        let mass: f64 = fb.inertias.iter().map(|i| i.mass).sum();
        // 胴体 6 行の並びは [角; 並進] なので、並進 z は index 5。
        assert!(
            (g[5].abs() - mass * 9.81).abs() < mass * 0.5,
            "並進 z の重力項 {} が m·g ({}) と離れています",
            g[5],
            mass * 9.81
        );
    }

    #[test]
    fn every_leg_axis_and_foot_link_resolves() {
        let r = robot();
        let cfg = WbcConfig {
            enabled: true,
            ..WbcConfig::default()
        };
        let w = WbcLayer::new(&r, &cfg).expect("同梱モデルで組み立てられること");
        assert_eq!(w.na, w.nv - 6);
        assert!(w.mass_kg() > 0.0);
        // 12 軸ぶんの v index が全部違うこと（引き当てを間違えていない）。
        let mut seen: Vec<usize> = w.leg_v_idx.iter().flatten().copied().collect();
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(seen.len(), 12);
    }

    /// **静止して 4 脚接地なら、接地力の合計は体重に近い。** ここが合わ
    /// なければ参照か接地フラグが疑わしい。
    #[test]
    fn a_four_legged_stance_solves_for_roughly_the_body_weight() {
        let r = robot();
        let cfg = WbcConfig {
            enabled: true,
            ..WbcConfig::default()
        };
        let mut w = WbcLayer::new(&r, &cfg).unwrap();
        let stand = crate::robot::rest_pose(&crate::config::AppConfig::default(), &r);
        let zero = JointVec::zeros();
        let plan = w.solve(&WbcObservation {
            measured_q: &stand,
            measured_qd: &zero,
            target_q: &stand,
            attitude_rad: [0.0; 3],
            gyro_rad_s: [0.0; 3],
            planned_yaw_rad: 0.0,
            body_velocity: [0.0; 3],
            body: level_stand(),
            mpc: None,
            stance: [true; 4],
            dt: 0.005,
        });
        assert_eq!(plan.status.stance_count, 4);
        let weight = w.mass_kg() * G;
        assert!(
            (plan.status.f_z_total_n - weight).abs() < weight * 0.5,
            "接地力の合計 {:.2} N が体重 {:.2} N と離れています",
            plan.status.f_z_total_n,
            weight
        );
    }

    /// **MPC の参照があればそちらを使う。** 準静的な参照と同じ観測でも、
    /// 接地力の参照が違えば解も違う。ここが繋がっていないと、MPC 歩容を
    /// 選んでも WBC には何も届かない（実際そこが唯一の効き目）。
    #[test]
    fn an_mpc_reference_replaces_the_quasi_static_one() {
        let r = robot();
        let cfg = WbcConfig {
            enabled: true,
            prox_weight: 0.0,
            // 鈍らせると 1 周期目は種を張るだけなので、比較のため生で。
            grf_smoothing: 1.0,
            ..WbcConfig::default()
        };
        // **膝の曲がった立ち姿勢で見る。** 伏せ姿勢（`rest_pose`、全軸 0）は
        // 脚が真下の突っ張りなので、垂直力をどう配っても calf の τ は 0。
        let stand = r.stance_posture(&crate::config::AppConfig::default().gait);
        let zero = JointVec::zeros();
        let solve = |mpc: Option<MpcReference>| {
            let mut w = WbcLayer::new(&r, &cfg).unwrap();
            w.solve(&WbcObservation {
                measured_q: &stand,
                measured_qd: &zero,
                target_q: &stand,
                attitude_rad: [0.0; 3],
                gyro_rad_s: [0.0; 3],
                planned_yaw_rad: 0.0,
                body_velocity: [0.0; 3],
                body: level_stand(),
                mpc,
                stance: [true; 4],
                dt: 0.005,
            })
        };
        // **前脚だけで支える**という、静的配分とは明確に違う参照。
        let weight = 2.4 * G;
        let mpc = MpcReference {
            grf_world: [
                na::Vector3::new(0.0, 0.0, weight / 2.0),
                na::Vector3::new(0.0, 0.0, weight / 2.0),
                na::Vector3::zeros(),
                na::Vector3::zeros(),
            ],
            accel_lin_world: na::Vector3::zeros(),
            accel_ang_world: na::Vector3::zeros(),
            solved: true,
        };

        let quasi = solve(None);
        assert!(!quasi.status.mpc_driven);
        let driven = solve(Some(mpc));
        assert!(driven.status.mpc_driven);
        // 前脚に寄せた参照なので、前脚の τ が静的配分より大きくなる。
        let fl = driven.legs[0][2].torque_nm.abs();
        let fl_quasi = quasi.legs[0][2].torque_nm.abs();
        assert!(
            fl > fl_quasi,
            "MPC の参照が効いていない: FL calf τ {fl} vs 静的 {fl_quasi}"
        );
    }

    /// **収束していない MPC の解は使わない。** 制約を満たさない接地力を
    /// 追いかけることになる。
    #[test]
    fn an_unsolved_mpc_solution_falls_back_to_the_quasi_static_reference() {
        let r = robot();
        let cfg = WbcConfig {
            enabled: true,
            ..WbcConfig::default()
        };
        let mut w = WbcLayer::new(&r, &cfg).unwrap();
        let stand = crate::robot::rest_pose(&crate::config::AppConfig::default(), &r);
        let zero = JointVec::zeros();
        let plan = w.solve(&WbcObservation {
            measured_q: &stand,
            measured_qd: &zero,
            target_q: &stand,
            attitude_rad: [0.0; 3],
            gyro_rad_s: [0.0; 3],
            planned_yaw_rad: 0.0,
            body_velocity: [0.0; 3],
            body: level_stand(),
            mpc: Some(MpcReference {
                grf_world: [na::Vector3::new(0.0, 0.0, 999.0); 4],
                accel_lin_world: na::Vector3::new(0.0, 0.0, 999.0),
                accel_ang_world: na::Vector3::zeros(),
                solved: false,
            }),
            stance: [true; 4],
            dt: 0.005,
        });
        assert!(!plan.status.mpc_driven);
        // 静的配分なので、接地力の合計は体重のまま。
        let weight = w.mass_kg() * G;
        assert!((plan.status.f_z_total_n - weight).abs() < weight * 0.5);
    }

    /// **出力の選び方だけが変わり、解は変わらない。** 同じ観測に対して
    /// トルクは 3 モードとも同じ値が出る（載せる先が違うだけ）。
    #[test]
    fn every_output_mode_carries_the_same_torque() {
        let r = robot();
        let stand = crate::robot::rest_pose(&crate::config::AppConfig::default(), &r);
        let zero = JointVec::zeros();
        let mut taus = Vec::new();
        for output in [WbcOutput::Torque, WbcOutput::Velocity, WbcOutput::Position] {
            let cfg = WbcConfig {
                enabled: true,
                output,
                // warm start は前周期の解に依るので、比較のため切る。
                prox_weight: 0.0,
                ..WbcConfig::default()
            };
            let mut w = WbcLayer::new(&r, &cfg).unwrap();
            let plan = w.solve(&WbcObservation {
                measured_q: &stand,
                measured_qd: &zero,
                target_q: &stand,
                attitude_rad: [0.0; 3],
                gyro_rad_s: [0.0; 3],
                planned_yaw_rad: 0.0,
                body_velocity: [0.0; 3],
                body: level_stand(),
                mpc: None,
                stance: [true; 4],
                dt: 0.005,
            });
            assert_eq!(
                plan.mode,
                match output {
                    WbcOutput::Torque => misa_core::ControlMode::Torque,
                    WbcOutput::Velocity => misa_core::ControlMode::Velocity,
                    WbcOutput::Position => misa_core::ControlMode::Position,
                }
            );
            taus.push(plan.legs[0][0].torque_nm);
        }
        for t in &taus[1..] {
            assert!((t - taus[0]).abs() < 1e-9, "{taus:?}");
        }
    }

    /// **トルク上限は QP の中では「ほぼ」しか守られない。**
    ///
    /// `torque_limits` は優先度 0 の硬い不等式だが、HoQP はそれを
    /// 最小二乗で解くので、上限に張り付いた軸は数値誤差ぶん外へ出る
    /// （同梱モデルの立位で実測 0.13 %）。**したがって上限を最終的に
    /// 保証するのは [`misa_core::SafetyGate`] のトルククランプのほう**で、
    /// ここが見ているのは「QP が上限を認識しているか」だけ。
    ///
    /// 桁で外れたら、モデルの `effort` か接地フラグが疑わしい。
    #[test]
    fn no_axis_is_asked_for_more_torque_than_the_model_declares() {
        let r = robot();
        let cfg = WbcConfig {
            enabled: true,
            ..WbcConfig::default()
        };
        let mut w = WbcLayer::new(&r, &cfg).unwrap();
        let stand = crate::robot::rest_pose(&crate::config::AppConfig::default(), &r);
        let zero = JointVec::zeros();
        let plan = w.solve(&WbcObservation {
            measured_q: &stand,
            measured_qd: &zero,
            target_q: &stand,
            attitude_rad: [0.1, -0.1, 0.0],
            gyro_rad_s: [0.0; 3],
            planned_yaw_rad: 0.0,
            body_velocity: [0.0; 3],
            body: level_stand(),
            mpc: None,
            stance: [true, true, false, false],
            dt: 0.005,
        });
        for (leg, names) in misa_hal::joint::JOINT_NAMES.iter().enumerate() {
            for (k, name) in names.iter().enumerate() {
                let limit = r.effort_limits.get(*name).copied().unwrap_or(f64::INFINITY);
                let tau = plan.legs[leg][k].torque_nm;
                assert!(
                    tau.abs() <= limit * 1.05 + 1e-6,
                    "{name} の τ {tau} が定格 {limit} を大きく超えています"
                );
            }
        }
    }

    /// **抜けて入り直したら参照は歩容の目標から張り直す。** 前の目標を
    /// 覚えたままだと、入った瞬間の `q̇_計画` が「抜けたときの姿勢 → いまの
    /// 目標」の跳びになり、速度出力でそのまま脚が飛ぶ。
    #[test]
    fn re_entering_seeds_the_reference_from_the_plan() {
        let r = robot();
        let cfg = WbcConfig {
            enabled: true,
            output: WbcOutput::Velocity,
            // 位置誤差の項を切って、差分の種だけを見る。
            velocity_track_kp: 0.0,
            ..WbcConfig::default()
        };
        let mut w = WbcLayer::new(&r, &cfg).unwrap();
        let stand = crate::robot::rest_pose(&crate::config::AppConfig::default(), &r);
        let zero = JointVec::zeros();
        fn obs<'a>(q: &'a JointVec, zero: &'a JointVec) -> WbcObservation<'a> {
            WbcObservation {
                measured_q: q,
                measured_qd: zero,
                target_q: q,
                attitude_rad: [0.0; 3],
                gyro_rad_s: [0.0; 3],
                planned_yaw_rad: 0.0,
                body_velocity: [0.0; 3],
                body: level_stand(),
                mpc: None,
                stance: [true; 4],
                dt: 0.005,
            }
        }
        w.solve(&obs(&stand, &zero));
        w.reset();
        let mut moved = stand;
        moved.legs[0][0] += 0.3;
        let plan = w.solve(&obs(&moved, &zero));
        // 種を捨てていなければ 0.3 rad / 5 ms = 60 rad/s の速度が出る。
        assert!(
            plan.legs[0][0].velocity_rad_s.abs() < 1.0,
            "張り直していない参照から速度 {} が出ています",
            plan.legs[0][0].velocity_rad_s
        );
    }
}

