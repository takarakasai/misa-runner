//! ロボットモデルの読み込みと歩容コントローラの組み立て。
//!
//! `.misa` から `misarta::model::Model` を作り、`quadruped-gait` の自動検出で
//! 脚の運動学（リンク長・関節符号）をモデルから直接取る。歩容側の設定を
//! 手書きの数値表にしないことで、モデルを直したのにコード側が古いまま、
//! という食い違いが起きないようにする（`go2-gait-runner` と同じ方針）。

use std::collections::BTreeMap;

use misarta::model::Model;
use quadruped_gait::{
    auto_detect_kinematics_config, joint_signs, AnyGaitController, ControllerOutput, GaitConfig,
    GaitGenerator, GaitMode, GaitType, KinematicsConfig, KneePattern, VelocityCmd,
    DEFAULT_FOOT_LINKS,
};

use crate::config::{AppConfig, GaitControllerKind, GaitTuning, KneeShape};
use crate::jointvec::JointVec;
use crate::pose::PoseLibrary;
use crate::teleop::GaitSelect;

/// 読み込み済みのロボット。
pub struct Robot {
    pub model: Model<f64>,
    pub kin: KinematicsConfig,
    /// IK 出力 → モデル（URDF）符号の変換表。`q_model = q_ik * sign`。
    pub signs: [[f64; 3]; 4],
    pub poses: PoseLibrary,
    /// 運動学の自動検出に使った姿勢（`nq` 長）。
    pub home_q: Vec<f64>,
    /// モデルが宣言している可動域（`[joint.limit]` の `lower` / `upper`）。
    ///
    /// **`lower == upper` の関節は入れない。** `.misa` の `limit` は
    /// `#[serde(default)]` なので、宣言が無いと両方 0 になる。それを
    /// 「0 rad に固定」と読むと、可動域が無い関節を全部 0 へ丸めてしまう。
    ///
    /// 校正値を PC が持たない機体（ブリッジ越し）では、これが可動域の
    /// 唯一の出どころになる。
    pub limits: BTreeMap<String, (f64, f64)>,
    /// モデルが宣言する定格速度 [rad/s]。宣言の無い関節は入らない。
    ///
    /// **目標の変化率の上限に使う。** 定格より速い目標は追えないので、
    /// そこで丸めるのが正しい。歩容がここを超える要求を出していたら
    /// `dump` が知らせる。
    pub rate_limits: BTreeMap<String, f64>,
    /// モデルが宣言する定格トルク [N·m]（`[joint.limit] effort`）。宣言の
    /// 無い関節は入らない。
    ///
    /// **WBC のトルク上限（優先度 0 の硬い制約）と、安全ゲートのトルク
    /// クランプの出どころ。** 位置制御しか使っていなかった間は誰も読んで
    /// いなかったが、トルクを出す以上は上限が要る。
    pub effort_limits: BTreeMap<String, f64>,
    /// 胴体リンクの名前（`.misa` の `root`）。
    ///
    /// **機体ごとに違う**（namiashi は `trunk`、namiashi2 は `base_link`）。
    /// 決め打ちにすると、シムで姿勢と位置が NaN のまま「転倒なし」と
    /// 出てしまう。
    /// 読むのは `sim`（`--features sim`）だけ。
    #[allow(dead_code)]
    pub root_link: String,
    /// 読めなかった**当たり判定**のメッシュ。空なら健全。
    ///
    /// 落ちたメッシュは黙って消え、**衝突しないぶん動いて見えてしまう**ので、
    /// 動力学を回す前に必ず見ること。
    pub bad_meshes: Vec<String>,
}

impl Robot {
    /// `.misa` を読んで運動学を自動検出する。
    ///
    /// `kinematics_pose` は「立った姿勢」の名前。自動検出はこの姿勢での FK を
    /// 使って公称の脚の高さを決めるので、脚を伸ばし切った姿勢を渡すと歩容の
    /// 立ち位置が高くなりすぎる。
    pub fn load(misa_path: &str, kinematics_pose: &str) -> Result<Self, String> {
        let parsed = misarta::native::load(misa_path)
            .map_err(|e| format!("{misa_path} の読み込みに失敗: {e:?}"))?;
        if !parsed.report.is_empty() {
            log::warn!(
                "{misa_path} の読み込みで警告があります: {:?}",
                parsed.report
            );
        }

        // **読めなかったメッシュは黙って消える。** 当たり判定のメッシュが
        // 落ちると、そのリンクは当たり判定がほぼ無い状態で走り、**衝突が
        // 起きないぶん「うまく動いている」ように見える**。namiashi2 の分解した
        // 胴体で実際に起きた: STL が 0 バイトになっていたのに、残った箱
        // 2 個だけで立って歩いていた (2026-09-02)。
        //
        // ここでは数えるだけ。**止めるのは動力学を回す `sim` だけ**で、
        // `dump`（運動学だけ）と実機の `run` には当たり判定が要らない。
        //
        // 同じ相対パスが複数の基準から引けることがある（namiashi2 は
        // `urdf/mesh/` に 0 バイトの同名ファイルが並んでいて、実体は
        // その 1 つ上にある）ので、**中身のある候補が 1 つでもあれば良し**とする。
        let dir = std::path::Path::new(misa_path)
            .parent()
            .unwrap_or(std::path::Path::new("."));
        let mut bad_meshes: Vec<String> = Vec::new();
        for l in &parsed.file.link {
            for c in &l.collision {
                let misarta::native::schema::Geom::Mesh { file: rel, .. } = &c.geom else {
                    continue;
                };
                let sizes: Vec<u64> = [dir.join(rel), dir.join("..").join(rel)]
                    .iter()
                    .filter_map(|p| std::fs::metadata(p).ok().map(|m| m.len()))
                    .collect();
                let why = match sizes.iter().max() {
                    Some(&n) if n > 0 => continue,
                    Some(_) => "空です",
                    None => "見つかりません",
                };
                let msg = format!("{} の {rel} が{why}", l.name);
                if !bad_meshes.contains(&msg) {
                    bad_meshes.push(msg);
                }
            }
        }

        let (model, _visual, _collision) = misarta::native::build_model(&parsed.file)
            .map_err(|e| format!("モデルの構築に失敗: {e:?}"))?;

        let limits: BTreeMap<String, (f64, f64)> = parsed
            .file
            .joint
            .iter()
            .filter(|j| j.limit.lower != j.limit.upper)
            .map(|j| (j.name.clone(), (j.limit.lower, j.limit.upper)))
            .collect();

        // モデルが宣言する定格速度 [rad/s]。**目標の変化率の上限に使う。**
        // 定格を超えた目標を出しても追えないので、そこで丸めるのが正しい。
        let rate_limits: BTreeMap<String, f64> = parsed
            .file
            .joint
            .iter()
            .filter(|j| j.limit.velocity > 0.0)
            .map(|j| (j.name.clone(), j.limit.velocity))
            .collect();

        let effort_limits: BTreeMap<String, f64> = parsed
            .file
            .joint
            .iter()
            .filter(|j| j.limit.effort > 0.0)
            .map(|j| (j.name.clone(), j.limit.effort))
            .collect();

        let root_link = parsed.file.robot.root.clone();
        let poses = PoseLibrary::from_misa(&parsed.file);
        let posture = resolve_kinematics_posture(&poses, kinematics_pose);
        let home_q = build_q(&model, &posture);

        let kin = auto_detect_kinematics_config(&model, &DEFAULT_FOOT_LINKS, &home_q)
            .map_err(|errs| format!("脚の運動学を自動検出できません: {errs:?}"))?;
        let signs = joint_signs(&model, &kin)?;

        Ok(Self {
            model,
            kin,
            signs,
            poses,
            home_q,
            limits,
            rate_limits,
            effort_limits,
            root_link,
            bad_meshes,
        })
    }

    /// 歩容コントローラを組み立てる。
    ///
    /// `wbc` を見るのは MPC の摩擦係数だけ。**WBC と MPC で違う摩擦を
    /// 仮定すると、片方が出せると思った接地力をもう片方が拒む。**
    pub fn build_gait(
        &self,
        tuning: &GaitTuning,
        wbc: &crate::config::WbcConfig,
        select: GaitSelect,
    ) -> AnyGaitController {
        let cfg = base_gait_config(tuning, select);
        let mode = gait_mode_of(select, tuning);
        let mut ctrl =
            AnyGaitController::new(mode, cfg, self.kin_at_height(tuning.stance_height_m));
        // **膝の向きは機体ごとに違う。** 取れる向きはモデルの可動域が
        // 決めるので、`dump` / `sim` の可動域検査で確かめてから選ぶ。
        ctrl.set_knee_pattern(knee_pattern_of(tuning.knee_pattern));
        // LinearCrawl はこちらで胴体高さを持つ。CHAMP 系は
        // `nominal_foot_body` を見るので上の `kin_at_height` が効く。
        ctrl.set_body_height_m(tuning.stance_height_m);
        self.configure_mpc(&mut ctrl, tuning, wbc, mode);
        ctrl
    }

    /// MPC 系の歩容へ、この機体の物理パラメータを入れる。
    ///
    /// **入れないと Cheetah 級の既定値（9 kg、慣性 0.07/0.26/0.24）で
    /// 走る。** namiashi は 2.4 kg なので、接地力の予測が 4 倍近く過大に
    /// なり、それを参照にした WBC が脚を跳ね上げる。
    ///
    /// 慣性と重心は**立ち姿勢**で測る。脚を伸ばし切った姿勢（`extend`）で
    /// 測ると、歩いている間の姿とかけ離れる。
    fn configure_mpc(
        &self,
        ctrl: &mut AnyGaitController,
        tuning: &GaitTuning,
        wbc: &crate::config::WbcConfig,
        mode: GaitMode,
    ) {
        use quadruped_gait::GaitGenerator;
        if !matches!(mode, GaitMode::Mpc | GaitMode::CentroidalSrbd) {
            return;
        }
        let body = self.body_inertia_at(&self.stance_posture(tuning));

        ctrl.set_capture_point_gain(tuning.mpc_capture_point_gain_s);
        match mode {
            GaitMode::Mpc => ctrl.set_srbd_mpc_config(quadruped_gait::SrbdMpcConfig {
                horizon_steps: tuning.mpc_horizon_steps,
                dt_per_step: tuning.mpc_dt_per_step,
                mass_kg: body.mass_kg,
                // **SRBD は慣性を対角しか持たない。** 非対角項は捨てる
                // （胴体が左右対称ならもともと小さい）。
                inertia_diag_body: body.inertia_body.diagonal(),
                friction_mu: wbc.friction_mu,
                ..quadruped_gait::SrbdMpcConfig::default()
            }),
            GaitMode::CentroidalSrbd => {
                ctrl.set_centroidal_mpc_config(quadruped_gait::CentroidalMpcConfig {
                    horizon_steps: tuning.mpc_horizon_steps,
                    dt_per_step: tuning.mpc_dt_per_step,
                    mass_kg: body.mass_kg,
                    centroidal_inertia_body: body.inertia_body,
                    com_offset_body: body.com_body,
                    friction_mu: wbc.friction_mu,
                    ..quadruped_gait::CentroidalMpcConfig::default()
                })
            }
            _ => {}
        }
    }

    /// 立ち姿勢の関節角。**MPC の慣性を測るときの姿勢。**
    ///
    /// 速度 0 の歩容を 1 周期だけ `dt = 0` で回して取る（位相は進まない）。
    /// 立ち位置は `nominal_foot_body` が決めるので**コントローラの種別に
    /// 依らない**。脚を伸ばし切った `kinematics_pose` で測ると、歩いている
    /// 間の姿とかけ離れた慣性になる。
    pub fn stance_posture(&self, tuning: &GaitTuning) -> JointVec {
        use quadruped_gait::GaitGenerator;
        let mut ctrl = AnyGaitController::new(
            GaitMode::Champ,
            base_gait_config(tuning, GaitSelect::Crawl),
            self.kin_at_height(tuning.stance_height_m),
        );
        ctrl.set_knee_pattern(knee_pattern_of(tuning.knee_pattern));
        ctrl.set_velocity_cmd(velocity_cmd(0.0, 0.0, 0.0));
        let out = ctrl.tick(0.0);
        self.output_to_joints(&out, 0.0)
    }

    /// その姿勢での質量・重心まわりの慣性・重心位置（すべて胴体座標系）。
    ///
    /// **胴体を水平に置いた固定ベースのモデルで測る**ので、世界座標と胴体
    /// 座標が一致する。浮遊ベースのモデル（[`crate::wbc`]）で測っても同じ
    /// 値になる — 慣性はベースの繋ぎ方に依らない。
    pub fn body_inertia_at(&self, posture: &JointVec) -> BodyInertia {
        let q = build_q(&self.model, posture);
        let phi = misarta::centroidal::compute_centroidal_inertia(&self.model, &q);
        BodyInertia {
            mass_kg: self.model.inertias.iter().map(|i| i.mass).sum(),
            inertia_body: phi.fixed_view::<3, 3>(0, 0).into_owned(),
            com_body: misarta::centroidal::compute_com(&self.model, &q),
        }
    }

    /// 立ち高さを `stance_height_m` にした運動学設定。
    ///
    /// `LegKinematics::nominal_foot_body` は「立ったときに足がいる位置」で、
    /// 自動検出は `kinematics_pose` での順運動学からこれを決める。つまり
    /// 立ち高さは基準姿勢に引きずられる。CHAMP 系のコントローラは
    /// `set_body_height_m` を見ない（あれは LinearCrawl 専用）ので、
    /// 設定した高さをどの歩容でも効かせるにはここを書き換えるしかない。
    fn kin_at_height(&self, stance_height_m: f64) -> KinematicsConfig {
        let mut kin = self.kin.clone();
        for leg in [&mut kin.fl, &mut kin.fr, &mut kin.rl, &mut kin.rr] {
            leg.nominal_foot_body.z = -stance_height_m;
        }
        kin
    }

    /// 歩容の出力（IK 座標系）をモデル座標系の関節ベクトルへ直す。
    ///
    /// 腕は歩容の管轄外なので `arm` はそのまま持ち越す。
    /// 胴体を `[roll, pitch]` (rad) 傾けた姿勢の関節角。
    ///
    /// **足先は世界座標で動かさない。** 歩容が出した足先位置（胴体座標系）を
    /// 逆向きに回してから IK を解き直すので、接地したまま胴体だけが傾く。
    ///
    /// 歩容側に胴体姿勢の制御は無い（`set_body_attitude_observed` は
    /// FullCentroidal 専用で、既定の Champ では no-op）。ここで足すしかない。
    ///
    /// `[0, 0]` のときは [`Self::output_to_joints`] にそのまま委ねる。
    /// **無効時に 1 ビットも変わらないことを保証するため**、丸め誤差の入る
    /// 経路を通さない。
    pub fn output_to_joints_tilted(
        &self,
        out: &ControllerOutput,
        arm: f64,
        attitude_rad: [f64; 3],
    ) -> (JointVec, bool) {
        let [roll, pitch, yaw] = attitude_rad;
        if roll == 0.0 && pitch == 0.0 && yaw == 0.0 {
            return (self.output_to_joints(out, arm), out.all_reachable());
        }
        let mut q = JointVec::zeros();
        q.arm = arm;
        let mut reachable = true;
        // 胴体を +roll/+pitch/+yaw 傾ける = 胴体座標系で見た足先を逆向きに
        // 回す。順序は Rz(−yaw) → Rx(−roll) → Ry(−pitch)。
        //
        // **yaw は「足を接地したまま胴体をひねる」** 動作になる。足先は
        // 胴体中心まわりに接線方向へ動くので、hip の可動域を食う。
        let (sy_, cy) = (-yaw).sin_cos();
        let (sr, cr) = (-roll).sin_cos();
        let (sp, cp) = (-pitch).sin_cos();
        for (slot, leg_out) in out.legs.iter().enumerate() {
            let f = leg_out.foot_body;
            // Rz(−yaw)
            let (x0, y0) = (f.x * cy - f.y * sy_, f.x * sy_ + f.y * cy);
            // Rx(−roll)
            let (y1, z1) = (y0 * cr - f.z * sr, y0 * sr + f.z * cr);
            // Ry(−pitch)
            let (x2, z2) = (x0 * cp + z1 * sp, -x0 * sp + z1 * cp);
            let target = nalgebra::Vector3::new(x2, y1, z2);
            // 膝はすべて後ろ向き（`build_gait` の `KneePattern::BothBack`）。
            // ここが食い違うと逆向きに曲がった解が返る。
            let sol = quadruped_gait::solve_leg_ik(self.kin.leg(leg_out.leg), target, false);
            reachable &= sol.is_reachable();
            let (hip, thigh, calf) = sol.angles();
            let s = self.signs[slot];
            q.legs[slot] = [hip * s[0], thigh * s[1], calf * s[2]];
        }
        (q, reachable)
    }

    pub fn output_to_joints(&self, out: &ControllerOutput, arm: f64) -> JointVec {
        let mut q = JointVec::zeros();
        q.arm = arm;
        for (name, q_ik) in out.iter_joint_targets() {
            let Some((leg, k)) = misa_hal::joint::lookup(name) else {
                log::warn!("歩容が知らない関節 {name} を出力しました");
                continue;
            };
            q.legs[leg.index()][k] = q_ik * self.signs[leg.index()][k];
        }
        q
    }
}

/// 選択された歩容に対応する `quadruped-gait` の歩容種別。
/// **シムと机上再生の初期姿勢。実機では使わない。**
///
/// **脱力した姿勢は一意に決まらない。** どう置いたか、どこで摩擦が止まるかで
/// 変わるので、「電源投入時の姿勢」を名前で持つことはそもそもできない。
/// だから実機（[`run`]）はここを見ず、毎周期 `measured`（実測の関節角）を
/// 始点にする。読めていないうちは [`mode_until_read`] が脱力のまま止める。
///
/// ここが要るのは、**実測できる相手がいない**シムと `dump` だけ。物理の
/// 初期条件として何か 1 つ決める必要がある、というだけの意味しかない。
///
/// 出どころは 3 通りあり、この順に見る。
///
/// 1. `control.rest_pose` — モデルの姿勢名。明示されていればこれ
/// 2. 校正値（`zero_pose_rad`）— シリアル構成のモータ角 0
/// 3. モデルの home
///
/// ここをゼロベクトルにすると可動域の外から始まることがある（namiashi2 の
/// 旧モデルは calf が −2.7..−0.8 で 0 rad が範囲外だった）。
pub fn rest_pose(cfg: &AppConfig, robot: &Robot) -> JointVec {
    let home = robot.poses.home();
    if let Some(name) = cfg.control.rest_pose.as_deref() {
        match robot.poses.pose(name) {
            Some(p) => return robot.poses.resolve(&p.angles, home),
            None => log::warn!(
                "control.rest_pose {name:?} がモデルにありません。ほかの手がかりを使います"
            ),
        }
    }
    let Ok(serial) = cfg.hardware.serial() else {
        return home;
    };
    let mut q = home;
    for slot in misa_hal::joint::LegSlot::ALL {
        let Some(bus) = serial.bus_for(slot) else {
            continue;
        };
        for (k, m) in bus.motors.iter().enumerate().take(3) {
            q.legs[slot.index()][k] = m.zero_pose_rad;
        }
    }
    q
}

pub fn gait_type_of(select: GaitSelect) -> GaitType {
    match select {
        GaitSelect::Crawl => GaitType::Crawl,
        GaitSelect::Walk => GaitType::Walk,
        GaitSelect::Trot => GaitType::Trot,
    }
}

/// 選択された歩容に対応するコントローラ。
///
/// 既定はすべて CHAMP 系。**`LinearCrawl` は胴体を +X 直線に載せる専用の
/// プランナで、横移動 (vy) と旋回 (wz) の指令を受け付けない**ので、
/// 「前後・左右・旋回をプロポで操る」という要件には合わない。直進の
/// 安定性を追い込みたいときだけ `gait.crawl_use_linear = true` で選ぶ。
/// 設定の膝の向き → 歩容ライブラリの型。
fn knee_pattern_of(shape: KneeShape) -> KneePattern {
    match shape {
        KneeShape::BothBack => KneePattern::BothBack,
        KneeShape::MammalianForward => KneePattern::MammalianForward,
        KneeShape::MammalianReverse => KneePattern::MammalianReverse,
        KneeShape::BothForward => KneePattern::BothForward,
    }
}

pub fn gait_mode_of(select: GaitSelect, tuning: &GaitTuning) -> GaitMode {
    match tuning.controller {
        // **既定は従来どおりの解釈。** `controller` を書かない設定の挙動を
        // 変えないため、`crawl_use_linear` をここで読む。
        GaitControllerKind::Auto => match select {
            GaitSelect::Crawl if tuning.crawl_use_linear => GaitMode::LinearCrawl,
            _ => GaitMode::Champ,
        },
        GaitControllerKind::Champ => GaitMode::Champ,
        GaitControllerKind::LinearCrawl => GaitMode::LinearCrawl,
        GaitControllerKind::Mpc => GaitMode::Mpc,
        GaitControllerKind::Centroidal => GaitMode::CentroidalSrbd,
    }
}

/// その姿勢での質量・慣性・重心。[`Robot::body_inertia_at`] が作る。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyInertia {
    pub mass_kg: f64,
    /// 重心まわりの回転慣性 [kg·m²]（胴体座標系）。
    pub inertia_body: nalgebra::Matrix3<f64>,
    /// 胴体原点から見た重心 [m]（胴体座標系）。
    pub com_body: nalgebra::Vector3<f64>,
}

/// 手振りポーズは**実際の運動学で計算して `.misa` に書いてある**。
///
/// # 3 本足で立つので重心移動が要る
///
/// 立ち姿勢の足先は胴体座標系で `(±0.147, ±0.109, −0.200)`。前脚を 1 本
/// 上げると支持三角形の対角線がちょうど胴体中心を通るので、**何もしないと
/// 倒れる**（余裕を計算すると 0.00001 m）。上げる前に胴体を**後ろへ 40 mm・
/// 振る脚と反対側へ 44 mm** 逃がしてある（内向き法線 `(∓0.596, ±0.803)`
/// 方向）。
///
/// # 数値の出どころ
///
/// `solve_leg_ik` に足先位置を渡して解き、`signs` を掛けてモデル角にした
/// ものをそのまま `.misa` に書いた。**手で書いた数字ではない。** 生成に
/// 使った計算は `git log` を辿れば残っている（一度きりの作業なので
/// テストとしては残していない）。
///
/// 全 8 ポーズの可動域の余裕は 15° 以上（一番きついのは `wave_fr_a` /
/// `wave_fl_a` の hip で 15.1°）。
///
/// # 前足は地上 180 mm まで上げる
///
/// **高さは胴体を機首上げ 15°・高さ 220 mm・後方 40 mm に置いて稼いでいる。**
/// 当初は胴体を水平のままにして脚だけで 110 mm 上げていたが、そのやり方で
/// 180 mm を狙うと hip ロールを高さに食われ、余裕が 5.7° まで詰まる
/// （振った瞬間に可動域端に当たる）。胴体を起こすと高さを thigh と calf の
/// ピッチ 2 軸で作れるようになり、hip をまるごと振り幅に回せる。
///
/// 重心も後退するので支持三角形の余裕はむしろ増える（56 → 61 mm）。
/// 左右で 61 / 71 mm と違うのは、重心が中心線上に無いため。
///
/// ポーズ間で動くのは振る脚だけで、支持脚 3 本の角度は 4 ポーズとも同じ。
/// 胴体を起こす動作はすべて `ready` への 0.8 s に入っている。
///
/// その歩容が実行中の胴体高さ変更（プロポ CH3）を受け付けるか。
///
/// **`LinearCrawl` だけが受け付ける。** `AnyGaitController::set_body_height_m`
/// は他のモードでは `_ => {}` で**黙って捨てる**ので、上位からは成功と
/// 区別がつかない。既定は全歩容 `Champ` なので、既定設定では CH3 は
/// どこにも効かない。
///
/// 設定の `gait.stance_height_m` は別経路（構築時の `GaitConfig`）なので
/// こちらは全歩容で効く。**「設定で高さを変えたら姿勢が変わった」ことを
/// 「実行中の高さ変更が効く」根拠にしてはいけない**（実際に取り違えた）。
pub fn gait_supports_body_height(mode: GaitMode) -> bool {
    matches!(mode, GaitMode::LinearCrawl)
}

/// プロファイルから決まる、その歩容の基準となる [`GaitConfig`]。
///
/// **歩容ごとに基準が違う。** 周期は `trot_cycle_s` / `walk_cycle_s` /
/// `crawl_cycle_s`（未設定なら歩容ライブラリの既定）、遊脚高さは全歩容で
/// `swing_height_m`、歩幅と接地比はライブラリの既定のまま。
///
/// 実行中の上書き（[`misa_core::GaitTune`]）はここへ重ねる。**基準値を
/// 知らないと「1 段上げる」が書けない**ので、操縦側もこれを読む。
pub fn base_gait_config(tuning: &GaitTuning, select: GaitSelect) -> GaitConfig {
    let mut cfg =
        GaitConfig::for_type(gait_type_of(select)).with_swing_height(tuning.swing_height_m);
    if let Some(period) = cycle_period_of(tuning, select) {
        cfg = cfg.with_cycle_period(period);
    }
    // **歩幅は速度の上限を決める。** 理由は
    // [`crate::config::GaitTuning::step_length_m`]。
    if let Some(step) = tuning.step_length_m {
        cfg.max_step_length_m = step;
    }
    cfg
}

/// 基準の [`GaitConfig`] に実行中の上書きを重ねる。
///
/// **上書きは丸めてから入れる**（[`misa_core::GaitTune::clamped`]）。操縦側も
/// 同じ範囲で刻むので、画面の値と歩容に入る値は一致する。
pub fn tuned_gait_config(
    tuning: &GaitTuning,
    select: GaitSelect,
    tune: &misa_core::GaitTune,
) -> GaitConfig {
    let mut cfg = base_gait_config(tuning, select);
    let t = tune.clamped();
    if let Some(v) = t.cycle_period_s {
        cfg.cycle_period_s = v;
    }
    if let Some(v) = t.swing_height_m {
        cfg.swing_height_m = v;
    }
    if let Some(v) = t.step_length_m {
        cfg.max_step_length_m = v;
    }
    if let Some(v) = t.duty_factor {
        cfg.duty_factor = v;
    }
    cfg
}

/// その歩容の基準値を、そのまま操縦側の初期値として使える形で。
pub fn base_gait_tune(tuning: &GaitTuning, select: GaitSelect) -> misa_core::GaitTune {
    let c = base_gait_config(tuning, select);
    misa_core::GaitTune {
        cycle_period_s: Some(c.cycle_period_s),
        swing_height_m: Some(c.swing_height_m),
        step_length_m: Some(c.max_step_length_m),
        duty_factor: Some(c.duty_factor),
    }
}

fn cycle_period_of(tuning: &GaitTuning, select: GaitSelect) -> Option<f64> {
    match select {
        GaitSelect::Crawl => tuning.crawl_cycle_s,
        GaitSelect::Walk => tuning.walk_cycle_s,
        GaitSelect::Trot => tuning.trot_cycle_s,
    }
}

/// 操縦指令 → 歩容への速度指令。
pub fn velocity_cmd(vx: f64, vy: f64, wz: f64) -> VelocityCmd {
    VelocityCmd { vx, vy, wz }
}

/// 運動学の自動検出に使う姿勢を決める。
///
/// 指定された名前が無ければ `[home]`、それも空なら全ゼロ。落とさないのは、
/// モデルを差し替えたときにポーズ名が揃っていなくても起動できるほうが
/// 現場では役に立つため。ただし黙って変えるのは危ないので警告は出す。
fn resolve_kinematics_posture(poses: &PoseLibrary, name: &str) -> JointVec {
    if let Some(pose) = poses.pose(name) {
        return poses.resolve(&pose.angles, JointVec::zeros());
    }
    // **`home` は `[[pose]]` ではなくモデルの `[home.joint_positions]`。**
    // ここで警告を出すと「モデルに home が無い」と読めてしまうが、
    // 実際には在って、それをそのまま使っている。
    if name == "home" {
        return poses.home();
    }
    log::warn!(
        "姿勢 {name:?} がモデルにありません。[home] を使います（あるポーズ: {:?}）",
        poses.pose_names().collect::<Vec<_>>()
    );
    poses.home()
}

/// 関節ベクトルを misarta の `q`（長さ `model.nq`）へ展開する。
fn build_q(model: &Model<f64>, posture: &JointVec) -> Vec<f64> {
    let mut q = model.neutral_q();
    for (name, value) in posture.iter_named() {
        set_joint(model, &mut q, name, value);
    }
    q
}

/// 名前で 1 関節ぶんの `q` を書く。モデルに無ければ何もしない。
fn set_joint(model: &Model<f64>, q: &mut [f64], name: &str, value: f64) {
    if let Some(i) = model.joints.iter().position(|j| j.name == name) {
        q[model.q_idx[i]] = value;
    }
}

/// アプリ設定からロボットを読む。
pub fn load_from_config(cfg: &AppConfig) -> Result<Robot, String> {
    Robot::load(&cfg.control.model, &cfg.control.kinematics_pose)
}

#[cfg(test)]
mod tests {

    use super::*;

    fn tuning(controller: GaitControllerKind, crawl_use_linear: bool) -> GaitTuning {
        GaitTuning {
            controller,
            crawl_use_linear,
            ..GaitTuning::default()
        }
    }

    #[test]
    fn gait_selection_maps_to_the_documented_controllers() {
        assert_eq!(gait_type_of(GaitSelect::Crawl), GaitType::Crawl);
        assert_eq!(gait_type_of(GaitSelect::Walk), GaitType::Walk);
        assert_eq!(gait_type_of(GaitSelect::Trot), GaitType::Trot);
        // 既定では 3 種とも CHAMP 系。横移動と旋回を受けるのはこちらだけ。
        let auto_off = tuning(GaitControllerKind::Auto, false);
        let auto_on = tuning(GaitControllerKind::Auto, true);
        for select in [GaitSelect::Crawl, GaitSelect::Walk, GaitSelect::Trot] {
            assert_eq!(gait_mode_of(select, &auto_off), GaitMode::Champ);
        }
        assert_eq!(gait_mode_of(GaitSelect::Crawl, &auto_on), GaitMode::LinearCrawl);
        assert_eq!(gait_mode_of(GaitSelect::Walk, &auto_on), GaitMode::Champ);
    }

    /// **歩幅がその歩容の最高速度を決める。**
    ///
    /// `歩幅 / (周期 × 接地比)`。ライブラリの既定では crawl が 0.042 m/s
    /// しか出せず、**同梱プロファイルの `max_vx_m_s = 0.15` は届かない**。
    /// 進む量だけを見て制御の良し悪しを判断しないための歯止め。
    #[test]
    fn the_step_length_sets_the_speed_ceiling_of_each_gait() {
        let t = GaitTuning::default();
        let ceiling = |select| {
            let c = base_gait_config(&t, select);
            c.max_step_length_m / (c.cycle_period_s * c.duty_factor)
        };
        let crawl = ceiling(GaitSelect::Crawl);
        assert!(
            (crawl - 0.042).abs() < 0.002,
            "crawl の上限が {crawl:.3} m/s（0.042 のはず）"
        );
        assert!(ceiling(GaitSelect::Walk) > crawl);
        assert!(ceiling(GaitSelect::Trot) > ceiling(GaitSelect::Walk));
        // プロファイルが宣言する最高速度に crawl が届いていない。
        assert!(crawl < GaitTuning::default().max_vx_m_s);
    }

    /// 歩幅の指定が歩容へ届くこと。**届かないと上の天井を上げられない。**
    #[test]
    fn a_profile_step_length_reaches_the_gait() {
        let t = GaitTuning {
            step_length_m: Some(0.145),
            ..GaitTuning::default()
        };
        for select in [GaitSelect::Crawl, GaitSelect::Walk, GaitSelect::Trot] {
            assert_eq!(base_gait_config(&t, select).max_step_length_m, 0.145);
        }
        // 指定が無ければライブラリのプリセットのまま。
        let d = GaitTuning::default();
        assert_eq!(base_gait_config(&d, GaitSelect::Crawl).max_step_length_m, 0.06);
    }

    /// **`controller` を書いたら `crawl_use_linear` より優先する。**
    /// 両方書ける以上、どちらが勝つかを試験で固定しておく。
    #[test]
    fn an_explicit_controller_overrides_the_legacy_flag() {
        // 旧フラグが立っていても、明示した種別が勝つ。
        let t = tuning(GaitControllerKind::Mpc, true);
        for select in [GaitSelect::Crawl, GaitSelect::Walk, GaitSelect::Trot] {
            assert_eq!(gait_mode_of(select, &t), GaitMode::Mpc);
        }
        let t = tuning(GaitControllerKind::Centroidal, false);
        assert_eq!(gait_mode_of(GaitSelect::Trot, &t), GaitMode::CentroidalSrbd);
        // **歩容の型は別の軸。** MPC でも Crawl / Walk / Trot は選べる。
        assert_eq!(gait_type_of(GaitSelect::Trot), GaitType::Trot);
    }

    /// **MPC 系だけが接地力の予測を出す。** WBC の参照が変わる分岐が
    /// ここに掛かっているので、種別と噛み合っていること。
    #[test]
    fn only_the_mpc_controllers_advertise_a_grf_prediction() {
        assert!(GaitControllerKind::Mpc.has_mpc());
        assert!(GaitControllerKind::Centroidal.has_mpc());
        assert!(!GaitControllerKind::Auto.has_mpc());
        assert!(!GaitControllerKind::Champ.has_mpc());
        assert!(!GaitControllerKind::LinearCrawl.has_mpc());
    }

    /// 既定設定では CH3（胴体高さ）はどの歩容でも効かない。
    ///
    /// `AnyGaitController::set_body_height_m` が `LinearCrawl` 以外を
    /// `_ => {}` で捨てるため。**黙って捨てられるので、警告を出す側の
    /// 判定がここと食い違うと誰も気づけない。**
    #[test]
    fn only_linear_crawl_takes_a_body_height_change() {
        assert!(gait_supports_body_height(GaitMode::LinearCrawl));
        assert!(!gait_supports_body_height(GaitMode::Champ));
        // 既定 (crawl_use_linear = false) では 3 歩容とも Champ なので、
        // CH3 はどこにも効かない。
        for select in [GaitSelect::Crawl, GaitSelect::Walk, GaitSelect::Trot] {
            assert!(
                !gait_supports_body_height(gait_mode_of(
                    select,
                    &tuning(GaitControllerKind::Auto, false)
                )),
                "{select:?} で高さ変更が効くことになっている"
            );
        }
        // crawl_use_linear = true なら Crawl だけ効く。
        let on = tuning(GaitControllerKind::Auto, true);
        assert!(gait_supports_body_height(gait_mode_of(GaitSelect::Crawl, &on)));
        assert!(!gait_supports_body_height(gait_mode_of(GaitSelect::Trot, &on)));
    }

    /// **同梱プロファイルが名指しする姿勢も、モデルに在ること。**
    ///
    /// 既定値だけを見る上の試験は通るのに、`robots/testquad.toml` が
    /// 存在しない姿勢名を指していた（`start_pose = "start"`。モデルにあるのは
    /// `constrain`）。無いときは警告を出して立ち姿勢へ直行するので、
    /// **CH5 中段の初期姿勢保持が黙って効かなくなる。** MuJoCo で回して
    /// 初めて気づいた類なので、試験で押さえる。
    #[test]
    fn the_shipped_profile_names_poses_that_exist_in_the_model() {
        let text = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../robots/testquad.toml"
        ))
        .unwrap();
        let mut cfg = AppConfig::from_toml(&text).unwrap();
        cfg.control.model = shipped_model_path();
        let robot = load_from_config(&cfg).expect("同梱モデルを読めません");
        for (what, name) in [
            ("初期姿勢 control.start_pose", &cfg.control.start_pose),
            ("運動学の基準姿勢 control.kinematics_pose", &cfg.control.kinematics_pose),
        ] {
            assert!(
                robot.poses.pose(name).is_some(),
                "{what} {name:?} がモデルにありません。ある姿勢: {:?}",
                robot.poses.pose_names().collect::<Vec<_>>()
            );
        }
    }

    /// 同梱モデルが既定設定の指す名前を全部持っていること。モデルを
    /// 差し替えたときにここが落ちれば、実機で「ポーズが無い」と気づく前に
    /// 分かる。
    #[test]
    fn the_shipped_model_has_every_pose_the_default_config_names() {
        let mut cfg = AppConfig::default();
        // 既定のパスはリポジトリルート相対。テストの作業ディレクトリは
        // crate ディレクトリなので、ここだけ絶対パスにする。
        cfg.control.model = shipped_model_path();
        let robot = load_from_config(&cfg).expect("同梱モデルを読めません");
        assert!(
            robot.poses.pose(&cfg.control.start_pose).is_some(),
            "初期姿勢 {:?} がモデルにありません",
            cfg.control.start_pose
        );
        assert!(
            robot.poses.pose(&cfg.control.kinematics_pose).is_some(),
            "運動学の基準姿勢 {:?} がモデルにありません",
            cfg.control.kinematics_pose
        );
        assert!(
            robot.poses.sequence(&cfg.poses.greeting).is_some()
                || robot.poses.pose(&cfg.poses.greeting).is_some(),
            "挨拶動作 {:?} がモデルにありません",
            cfg.poses.greeting
        );
    }

    /// **同梱プロファイルは、モデルが宣言する可動域の中だけで歩くこと。**
    ///
    /// これが無かったあいだ、namiashi2 に `knee_pattern = "<>"` を入れて掃引し、
    /// 「いちばん速く前へ進む設定」として採ってしまった (2026-09-01)。namiashi2 の
    /// calf は `-2.705..-0.838` で常に負なので、後脚に正の膝角を要求する `<>`
    /// は物理的に取れない。**それでも MuJoCo はそれらしく動く** — 脚が可動域に
    /// 当たって長さが変わり、歩行に見えるものが出る。数字だけ見ていると
    /// 良い結果に見えるので、ここで落とす。
    ///
    /// namiashi2 のモデルはリポジトリの外（`../mdls/`）にあるので、無い環境では
    /// その 1 台を飛ばす。**あるのに落ちる、が拾いたい状態。**
    #[test]
    fn every_shipped_profile_stays_inside_the_model_limits() {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../..");
        let mut checked = 0;
        for entry in std::fs::read_dir(format!("{root}/robots")).unwrap() {
            let path = entry.unwrap().path();
            if path.extension().and_then(|e| e.to_str()) != Some("toml") {
                continue;
            }
            let text = std::fs::read_to_string(&path).unwrap();
            let mut cfg = AppConfig::from_toml(&text).unwrap();
            // プロファイルのモデルパスはリポジトリルート相対。
            if !cfg.control.model.starts_with('/') {
                cfg.control.model = format!("{root}/{}", cfg.control.model);
            }
            if !std::path::Path::new(&cfg.control.model).exists() {
                println!("飛ばす: {} （モデル {} が無い）", cfg.name, cfg.control.model);
                continue;
            }
            let robot = match load_from_config(&cfg) {
                Ok(r) => r,
                Err(e) => {
                    println!("飛ばす: {} （読めません: {e}）", cfg.name);
                    continue;
                }
            };
            checked += 1;
            let model_limits = robot.limits.clone();
            let rest = rest_pose(&cfg, &robot);
            let layout = crate::snapshot::axis_layout(&cfg).unwrap();
            let dt = 1.0 / cfg.control.rate_hz;
            let limits = crate::snapshot::safety_config(&cfg, &layout, &model_limits, &robot.rate_limits, &robot.effort_limits, dt, 5.0);
            for select in [GaitSelect::Crawl, GaitSelect::Walk, GaitSelect::Trot] {
                let mut controller = crate::controller::Controller::new(robot_for(&cfg), cfg.clone());
                let mut intent = misa_core::Intent {
                    mode: crate::teleop::ModeRequest::Walk,
                    gait: select,
                    link_ok: true,
                    aux_rad: vec![None],
                    ..misa_core::Intent::default()
                };
                let mut violations = Vec::new();
                for i in 0..(8.0 / dt) as usize {
                    if controller.state() == crate::controller::State::Active {
                        intent.velocity = misa_core::Velocity {
                            vx_m_s: cfg.gait.max_vx_m_s * 0.3,
                            vy_m_s: 0.0,
                            wz_rad_s: 0.0,
                        };
                    }
                    let out = controller.tick(&intent, &rest, [0.0; 3], dt);
                    crate::dump::check_limits(
                        &limits,
                        &layout,
                        &out.targets,
                        i as f64 * dt,
                        &mut violations,
                    );
                }
                assert!(
                    violations.is_empty(),
                    "{} の {:?} が可動域を {} 件破っています。最初の 3 件:\n  {}",
                    cfg.name,
                    select,
                    violations.len(),
                    violations
                        .iter()
                        .take(3)
                        .cloned()
                        .collect::<Vec<_>>()
                        .join("\n  ")
                );
            }
        }
        assert!(checked > 0, "プロファイルを 1 つも検査できていません");
    }

    /// 上の試験用。`Controller::new` がモデルを move するので歩容ごとに読み直す。
    fn robot_for(cfg: &AppConfig) -> Robot {
        load_from_config(cfg).unwrap()
    }

    /// 同梱モデルの絶対パス（`crates/misa-runner` から見たリポジトリルート）。
    fn shipped_model_path() -> String {
        format!("{}/../../models/testquad/testquad.misa", env!("CARGO_MANIFEST_DIR"))
    }

    #[test]
    fn the_stance_height_is_written_into_the_nominal_foot_position() {
        let robot = Robot::load(&shipped_model_path(), "extend").unwrap();
        let kin = robot.kin_at_height(0.21);
        for leg in [&kin.fl, &kin.fr, &kin.rl, &kin.rr] {
            assert!((leg.nominal_foot_body.z + 0.21).abs() < 1e-12);
        }
    }

    #[test]
    fn a_missing_kinematics_pose_falls_back_to_home() {
        let mut home = JointVec::zeros();
        home.legs[0][1] = 0.5;
        let lib = PoseLibrary::new(vec![], vec![], home);
        assert_eq!(resolve_kinematics_posture(&lib, "nope"), home);
    }
}


