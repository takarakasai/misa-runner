//! articara の MuJoCo を [`Plant`] にする。
//!
//! **2 つ目の `Plant` 実装。** 動力学を確かめるためだけでなく、抽象の検証と
//! してここにある。実装が 1 つしかないトレイトは、それが正しい継ぎ目かどうか
//! 誰にも分からない。
//!
//! # 何が確かめられて、何が確かめられないか
//!
//! 確かめられるのは**制御則**。転ぶか、トルクが足りるか、可動域に収まるか、
//! 接地が滑るか。確かめられないのは**タイミング**で、RS485 の往復遅れ・
//! バスのジッタ・モータの一次遅れ・受信断は再現されない。namiashi が実際に
//! 痛い目を見たのはむしろ後者側なので、ここを通ったから実機が通るとは
//! 考えないこと。
//!
//! # 脱力できない
//!
//! MJCF の位置アクチュエータには「脱力」が無いので、
//! [`misa_core::ControlMode::Idle`] の軸は**その場の角度で保持**する。
//! 実機の脱力は荷重で崩れるが、ここでは立ったままになる。脱力からの復帰を
//! 検証する用途にはそのまま使えない。
//!
//! # ビルド
//!
//! MuJoCo 3.8 の共有ライブラリが要る:
//!
//! ```sh
//! export MUJOCO_DYNAMIC_LINK_DIR=$HOME/.mujoco/mujoco-3.8.0/lib
//! ```

use std::path::Path;

use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use articara::mujoco_sim::MujocoSim;
use articara::rbd::model::ActuatorMode;
use articara::robot::RobotModel;
use misa_core::{
    AxisId, AxisTable, Command, ControlMode, Imu, Observation, Plant, PlantCaps,
};

/// 重力 [m/s²]。加速度の埋めに使う。
const G: f64 = 9.806_65;

/// シミュレータの立ち上げ方。
pub struct SimOptions {
    /// 読み込むモデル（`.misa`）。
    pub misa_path: String,
    /// 制御周期 [s]。1 tick でこの時間ぶん物理を進める。
    pub control_period_s: f64,
    /// 位置アクチュエータの PD。**追従の質はここで決まる**ので、実機の
    /// 追従と比べるときは必ず併せて記録すること。
    pub actuator_kp: f64,
    pub actuator_kv: f64,
    /// モデルの `effort`（＝アクチュエータが出せるトルクの上限）に掛ける係数。
    ///
    /// **WBC の `wbc.torque_scale` と同じ値を渡すこと。** あちらは「QP が
    /// 計画してよいトルク」、こちらは「実際に出るトルク」で、揃っていないと
    /// QP は出ないトルクを当てにした解を出す（実測で crawl の進む量が
    /// 0.315 → 0.242 m に落ちた）。`.misa` の `effort` は連続定格なので、
    /// 瞬間の出力を見たいときは両方を同じだけ上げる。
    pub torque_scale: f64,
    /// 速度制御のゲイン [N·m/(rad/s)]。**位置制御の `actuator_kv` とは別。**
    ///
    /// 位置制御では `kv` は減衰項（`kp` と対で効く）だが、速度制御では
    /// `τ = kv·(q̇* − q̇)` の**唯一のゲイン**になる。位置制御向けの
    /// 1.0 のままだと、calf の重力負荷 0.7 N·m を支えるのに 0.7 rad/s の
    /// 速度誤差が要るほど柔らかく、**歩容ではなくこのゲインが挙動を
    /// 決めてしまう**（MuJoCo の crawl で後ろへ 0.5 m 走った）。
    ///
    /// 実機の LKMTech は速度ループをドライバの中に持っていて、ここより
    /// ずっと硬い。**シムで速度出力を見るときは必ず上げること。**
    pub velocity_kv: f64,
    /// 胴体の初期高さ [m]。低すぎると床にめり込んだ状態から始まる。
    pub base_height_m: f64,
    /// 初期姿勢（関節名 → 角度 [rad]）。ここから物理が始まる。
    pub home: Vec<(String, f64)>,
    /// 接地摩擦 `[滑り, ねじれ, 転がり]`。`None` で articara の既定
    /// `[0.7, 0.005, 0.0001]`（ゴム対床の実測 0.4〜1.0 の真ん中）。
    ///
    /// **足が滑ると歩容は成立しない。** 接地中の足の滑りが指令速度と同じ
    /// 桁なら、歩容をどう振っても進む量は合わない。
    pub friction: Option<[f64; 3]>,
    /// 物理の刻み [s]。`None` で MuJoCo の既定（2 ms）。
    ///
    /// **重い機体では下げないと立てない。** ここの PD は articara が Rust 側で
    /// 計算して `motor` に流す**明示的**な速度フィードバックなので、
    /// `actuator_kv < 2·I/dt` でしか安定しない（`I` は関節自身の慣性）。
    /// 既定の 2 ms だと使える `kv` が位置保持に要る値を下回り、支えきれずに
    /// 沈むか、`kv` を上げると発散する。刻みを半分にすると使える `kv` が倍になる。
    pub timestep_s: Option<f64>,
    /// MuJoCo の `<option impratio>`。摩擦拘束の硬さの比。**既定（None = 1）
    /// では接地足が荷重の下で地面を這う**（namiashi の trot で 0.1〜0.25 m/s
    /// 進行方向へ流れた）。MuJoCo の推奨は 10 以上 + `cone = "elliptic"`。
    /// 接地モデルの当て方が結果を変えるので、感度を見るための軸として残す。
    pub impratio: Option<f64>,
    /// MuJoCo の `<option cone>`（`"pyramidal"` | `"elliptic"`）。
    pub cone: Option<String>,
    /// 足を「接地」と報告する垂直力の閾値 [N]。
    ///
    /// MuJoCo は接触の有無を幾何で知っているが、**かすっただけの遊脚を
    /// 接地と報告すると WBC の硬い接地拘束が壊れる**。articara の
    /// `ContactDrivenPhase` と同じく力で切る（あちらは 5 N）。0 以下なら
    /// 幾何の接触そのまま。
    pub contact_threshold_n: f64,
    /// 接地を見る足リンク。並びは脚の順。
    pub feet: Vec<String>,
    /// 胴体リンク。姿勢と角速度をここから読む。
    pub root_link: String,
}

impl Default for SimOptions {
    fn default() -> Self {
        Self {
            misa_path: String::new(),
            control_period_s: 0.005,
            actuator_kp: 60.0,
            actuator_kv: 1.0,
            torque_scale: 1.0,
            velocity_kv: 20.0,
            base_height_m: 0.30,
            home: Vec::new(),
            friction: None,
            timestep_s: None,
            impratio: None,
            cone: None,
            contact_threshold_n: 5.0,
            feet: ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
                .iter()
                .map(|s| s.to_string())
                .collect(),
            root_link: "trunk".into(),
        }
    }
}

/// MuJoCo の中の実機。
pub struct MujocoPlant {
    model: RobotModel,
    sim: MujocoSim,
    axes: AxisTable,
    caps: PlantCaps,
    /// 軸 → `RobotModel` の関節添字。**組み立て時に 1 回だけ引く。**
    joint_idx: Vec<usize>,
    feet: Vec<String>,
    contact_threshold_n: f64,
    root_link: String,
    /// 前周期の胴体速度（世界座標）と時刻。加速度計を差分で作るため。
    prev_base_vel_world: Option<([f64; 3], f64)>,
    /// 制御モードで切り替えるゲイン。**同じ `actuator_kv` の欄を、位置と
    /// 速度で違う意味に使う**ので、両方を控えて毎周期入れ直す。
    position_kp: f64,
    position_kv: f64,
    velocity_kv: f64,
    /// 1 tick で進める MuJoCo のフレーム数。
    frames_per_tick: u32,
    #[cfg(feature = "render")]
    render: Option<Render>,
}

/// オフスクリーン描画の設定。
#[cfg(feature = "render")]
pub struct RenderOptions {
    pub outdir: String,
    pub width: u32,
    pub height: u32,
    /// カメラの方位 [deg]。90 で真横（x-z 面）、180 で真後ろ。
    pub azimuth: f64,
    pub elevation: f64,
    /// 胴体からの距離 [m]。**機体の大きさで変える。**
    pub distance: f64,
    /// 注視点の高さ [m]。
    pub look_z: f64,
    /// 固定カメラの注視点 `[x, y]` [m]。追従カメラでは使わない。
    /// **移動の中点に置く**と、始点も終点も画面に収まる。
    pub look_xy: [f64; 2],
    /// 胴体を追わずにその場に固定する。
    ///
    /// **地面が無地なので、追従カメラだと歩いても止まって見える。** 進んだ
    /// ことを絵で見せたいときは固定にして、`distance` を移動量ぶん広く取る。
    /// その代わり機体は小さくなるので、姿勢や脚の動きを見るときは追従のまま。
    pub fixed: bool,
}

#[cfg(feature = "render")]
struct Render {
    renderer: mujoco::renderer::MjRenderer,
    outdir: String,
    frame: usize,
}

impl MujocoPlant {
    pub fn new(axes: AxisTable, opts: &SimOptions) -> Result<Self, String> {
        let mut model = RobotModel::from_misa(Path::new(&opts.misa_path))
            .map_err(|e| format!("{} を読めません: {e}", opts.misa_path))?;

        // 位置制御のアクチュエータに揃える。読み込んだままだとトルク源で、
        // 関節角の目標を渡しても誰も追わない。
        for j in model.joints.iter_mut().filter(|j| j.joint_type != "fixed") {
            j.actuator_mode = ActuatorMode::Position;
            j.actuator_kp = opts.actuator_kp;
            j.actuator_kv = opts.actuator_kv;
            // **アクチュエータの上限もここで決まる。** `apply_controller` が
            // `joint.effort` でトルクを頭打ちにするので、連続定格のままだと
            // 「瞬間はもっと出る」を試せない。
            if opts.torque_scale > 0.0 {
                j.effort *= opts.torque_scale;
            }
        }
        for (name, q) in &opts.home {
            let idx = *model
                .joint_map
                .get(name.as_str())
                .ok_or_else(|| format!("初期姿勢の関節 {name} がモデルにありません"))?;
            model.joint_positions[idx] = *q;
        }
        model.rebuild_misarta_model();

        // **軸表の順に関節添字を引いておく。** 毎周期名前で引くと、
        // 200 Hz × 13 軸ぶんのハッシュ引きが載る。
        let mut joint_idx = Vec::with_capacity(axes.len());
        for a in axes.axes() {
            let idx = *model
                .joint_map
                .get(a.name.as_str())
                .ok_or_else(|| format!("関節 {} がモデルにありません", a.name))?;
            joint_idx.push(idx);
        }

        let mjcf = MjcfExportOptions {
            base_pos: Some([0.0, 0.0, opts.base_height_m]),
            ground_plane: Some(GroundPlaneCfg {
                z: 0.0,
                half_size: 5.0,
                roll: 0.0,
                pitch: 0.0,
            }),
            add_actuators: true,
            // **物理の刻みは制御周期を割り切る値にする。** MuJoCo の既定
            // 2 ms のままだと 5 ms の周期に 3 フレーム（= 6 ms）進めることに
            // なり、制御が 5 ms と信じている 1 周期に物理は 6 ms 流れる。
            // 速度は 5/6 に読まれ、歩容は物理時間で 20 % 遅く回り、重力の
            // 効きが軽く見える（2026-09-06 まで気付かず、その間の sim の
            // 数字はこの歪みを含む）。指定が無ければ 2 ms 以下で周期を
            // 割り切る最大の刻みにする（5 ms → 1.667 ms × 3）。
            timestep: Some(opts.timestep_s.unwrap_or_else(|| {
                let n = (opts.control_period_s / 0.002).ceil().max(1.0);
                opts.control_period_s / n
            })),
            impratio: opts.impratio,
            cone: match opts.cone.as_deref() {
                Some("elliptic") => Some("elliptic"),
                Some("pyramidal") => Some("pyramidal"),
                Some(other) => return Err(format!("未知の cone {other:?}（pyramidal|elliptic）")),
                None => None,
            },
            default_friction: opts.friction.unwrap_or([0.7, 0.005, 0.0001]),
            ..MjcfExportOptions::default()
        };
        let mut sim = MujocoSim::new(&model, mjcf).map_err(|e| format!("MuJoCo: {e}"))?;
        // 直近のフレームしか読まないので、リングは最小で足りる。
        sim.set_trace_max(2);

        let timestep = sim.timestep();
        let frames_per_tick = ((opts.control_period_s / timestep).round() as u32).max(1);
        let drift = frames_per_tick as f64 * timestep - opts.control_period_s;
        if drift.abs() > 1e-6 {
            log::warn!(
                "MuJoCo の刻み {:.4} ms が制御周期 {:.2} ms を割り切りません。1 tick で物理が {:+.3} ms ずれます（--timestep で直せる）",
                timestep * 1e3,
                opts.control_period_s * 1e3,
                drift * 1e3
            );
        }
        log::info!(
            "MuJoCo timestep {:.4} ms、制御周期 {:.2} ms → 1 tick あたり {} フレーム",
            timestep * 1e3,
            opts.control_period_s * 1e3,
            frames_per_tick
        );

        let caps = PlantCaps {
            // **アクチュエータは全部 `<motor>` で、PD は articara が Rust 側で
            // 回している。** したがってモードは毎周期切り替えられる（MJCF を
            // 書き直す必要がない）。実機と同じ 3 モードを名乗れる。
            modes: vec![
                ControlMode::Position,
                ControlMode::Velocity,
                ControlMode::Torque,
            ],
            has_imu: true,
            // **接地は実機と違ってちゃんと分かる。** シムの取り柄の 1 つ。
            has_contacts: true,
            driven: vec![true; axes.len()],
        };

        Ok(Self {
            model,
            sim,
            axes,
            caps,
            joint_idx,
            feet: opts.feet.clone(),
            contact_threshold_n: opts.contact_threshold_n,
            root_link: opts.root_link.clone(),
            prev_base_vel_world: None,
            position_kp: opts.actuator_kp,
            position_kv: opts.actuator_kv,
            velocity_kv: opts.velocity_kv,
            frames_per_tick,
            #[cfg(feature = "render")]
            render: None,
        })
    }

    /// オフスクリーン描画を始める（`--features render`）。
    ///
    /// **GUI は要らない。** EGL でヘッドレスに描いて PNG を並べ、あとで
    /// ffmpeg にまとめる。articara の GUI へ Zenoh で流すのとは別経路で、
    /// あちらは関節角だけ（接地も地面も出ない）。
    #[cfg(feature = "render")]
    pub fn start_recording(&mut self, opts: &RenderOptions) -> Result<(), String> {
        use mujoco::renderer::MjRenderer;
        use mujoco::prelude::*;

        std::fs::create_dir_all(&opts.outdir)
            .map_err(|e| format!("{} を作れません: {e}", opts.outdir))?;
        let model = self.sim.mj_model();
        let mut renderer = MjRenderer::builder()
            .width(opts.width)
            .height(opts.height)
            .num_visual_user_geom(0)
            .num_visual_internal_geom(0)
            .rgb(true)
            .depth(false)
            .build(model.clone())
            .map_err(|e| format!("オフスクリーン描画を作れません（EGL）: {e:?}"))?;

        let body = model
            .body(&self.root_link)
            .ok_or_else(|| format!("胴体リンク {} がモデルにありません", self.root_link))?;
        let mut cam = if opts.fixed {
            MjvCamera::new_free(&model)
        } else {
            MjvCamera::new_tracking(body.id)
        };
        cam.azimuth = opts.azimuth;
        cam.elevation = opts.elevation;
        cam.distance = opts.distance;
        cam.lookat = [opts.look_xy[0], opts.look_xy[1], opts.look_z];
        renderer.set_camera(cam);

        self.render = Some(Render {
            renderer,
            outdir: opts.outdir.clone(),
            frame: 0,
        });
        Ok(())
    }

    /// 1 フレーム保存する。呼ぶ間隔が動画のフレームレートになる。
    #[cfg(feature = "render")]
    pub fn capture(&mut self) -> Result<(), String> {
        let Some(r) = self.render.as_mut() else {
            return Ok(());
        };
        r.renderer
            .sync_data(self.sim.mj_data_mut())
            .map_err(|e| format!("描画の同期に失敗: {e:?}"))?;
        r.renderer
            .render()
            .map_err(|e| format!("描画に失敗: {e:?}"))?;
        let path = format!("{}/frame_{:05}.png", r.outdir, r.frame);
        r.renderer
            .save_rgb(&path)
            .map_err(|e| format!("{path} を書けません: {e:?}"))?;
        r.frame += 1;
        Ok(())
    }

    /// 保存したフレーム数。
    #[cfg(feature = "render")]
    pub fn frames(&self) -> usize {
        self.render.as_ref().map_or(0, |r| r.frame)
    }

    /// 胴体のワールド位置。転倒判定や進んだ距離を見るのに使う。
    pub fn base_position(&self) -> Option<[f64; 3]> {
        self.sim.body_world_position(&self.root_link)
    }

    pub fn sim(&self) -> &MujocoSim {
        &self.sim
    }

    /// 足のワールド位置。並びは [`SimOptions::feet`]。
    pub fn foot_positions(&self) -> Vec<Option<[f64; 3]>> {
        self.feet
            .iter()
            .map(|f| self.sim.body_world_position(f))
            .collect()
    }

    /// 足 4 本の**真の**垂直接地力 [N]（診断用。接地推定の答え合わせ）。
    pub fn foot_forces(&self) -> Option<[f64; 4]> {
        if self.feet.len() != 4 {
            return None;
        }
        let feet: [&str; 4] = [
            self.feet[0].as_str(),
            self.feet[1].as_str(),
            self.feet[2].as_str(),
            self.feet[3].as_str(),
        ];
        Some(self.sim.contact_force_per_foot(&feet))
    }

    /// **いま地面に触れている、足ではないリンクの名前。**
    ///
    /// 四脚が足以外で体重を支えていると、歩容が空振りしていても
    /// 「歩いている」ように見える。namiashi2 は車輪付きなので実際に起きた:
    /// 立ち高さを指定しても胴体が 0.211 m から下がらず、足は 1 度も
    /// 接地しないまま車輪で転がっていた (2026-09-01)。**足の接地率だけ
    /// 見ていると、これは「遊脚率 100%」という良さそうな数字に化ける。**
    pub fn non_foot_ground_contacts(&self) -> Vec<String> {
        let mut out: Vec<String> = Vec::new();
        for c in self.sim.contacts() {
            if c.is_self_collision() {
                continue;
            }
            // 世界との接触は片側の名前が空。触れているほうを取る。
            let body = if c.body1.is_empty() { &c.body2 } else { &c.body1 };
            if body.is_empty() || self.feet.iter().any(|f| f.eq_ignore_ascii_case(body)) {
                continue;
            }
            if !out.iter().any(|b| b == body) {
                out.push(body.clone());
            }
        }
        out.sort();
        out
    }
}

impl Plant for MujocoPlant {
    fn axes(&self) -> &AxisTable {
        &self.axes
    }

    fn capabilities(&self) -> &PlantCaps {
        &self.caps
    }

    fn arm(&mut self) -> Result<(), String> {
        Ok(())
    }

    fn disarm(&mut self) -> Result<(), String> {
        Ok(())
    }

    fn exchange(&mut self, cmd: &Command, obs: &mut Observation) -> Result<(), String> {
        // ── 出す ──────────────────────────────────────────────
        for (i, &ji) in self.joint_idx.iter().enumerate() {
            let Some(a) = cmd.get(AxisId::new(i as u16)) else {
                continue;
            };
            // **モードは軸ごと・周期ごとに切り替える。** `apply_controller` は
            // 毎周期 `joints[ji].actuator_mode` を読むので、ここを書き換える
            // だけで制御則が変わる。MJCF の側は `<motor>` のままでよい。
            match a.mode {
                // 位置アクチュエータに脱力は無いので、その場で保持する。
                ControlMode::Idle => {
                    let q = self.model.joint_positions[ji];
                    self.model.joints[ji].actuator_mode = ActuatorMode::Position;
                    self.sim.set_position_target(ji, q);
                    self.sim.set_position_target_velocity(ji, 0.0);
                    self.sim.set_torque_feedforward(ji, 0.0);
                }
                ControlMode::Torque => {
                    self.model.joints[ji].actuator_mode = ActuatorMode::Torque;
                    self.sim.set_torque_target(ji, a.torque_ff_nm);
                }
                ControlMode::Velocity => {
                    self.model.joints[ji].actuator_mode = ActuatorMode::Velocity;
                    // **速度制御の kv は位置制御のものと別物。** 詳しくは
                    // [`SimOptions::velocity_kv`]。
                    self.model.joints[ji].actuator_kv = self.velocity_kv;
                    self.sim.set_velocity_target(ji, a.velocity_rad_s);
                }
                // 位置と MIT。**τ は前置として足す**ので、WBC の解を位置
                // 出力で回したときも接地力ぶんの力は出る（実機の MIT と
                // 同じ形）。位置制御しか持たない機体では無視される値。
                ControlMode::Position | ControlMode::Impedance => {
                    self.model.joints[ji].actuator_mode = ActuatorMode::Position;
                    // **指令に kp/kd が載っていればそれを使う。** ブリッジ越しの
                    // 機体（keel）は `[hardware.mit_gains]` を毎周期載せるので、
                    // シムでも同じ関節別のゲインで回る（実機は calf だけ kp の
                    // 上限が低い、という事情をシムで見られる）。載っていない
                    // 機体（シリアル）は `--kp` / `--kv` の一律の値。
                    if a.kp_nm_per_rad > 0.0 {
                        self.model.joints[ji].actuator_kp = a.kp_nm_per_rad;
                        self.model.joints[ji].actuator_kv = a.kd_nm_s_per_rad;
                    } else {
                        self.model.joints[ji].actuator_kp = self.position_kp;
                        self.model.joints[ji].actuator_kv = self.position_kv;
                    }
                    self.sim.set_position_target(ji, a.position_rad);
                    self.sim.set_torque_feedforward(ji, a.torque_ff_nm);
                }
            }
        }

        // ── 進める ────────────────────────────────────────────
        self.sim
            .step_n_frames(&mut self.model, self.frames_per_tick, true);

        // ── 受け取る ──────────────────────────────────────────
        let Some(frame) = self.sim.trace().last() else {
            return Err("MuJoCo が 1 フレームも記録していません".into());
        };
        obs.time = misa_core::Time::from_secs_f64(frame.time);

        for (i, &ji) in self.joint_idx.iter().enumerate() {
            let Some(a) = obs.get_mut(AxisId::new(i as u16)) else {
                continue;
            };
            a.position_rad = frame.q.get(ji).copied().unwrap_or(0.0);
            a.velocity_rad_s = frame.qvel.get(ji).copied().unwrap_or(0.0);
            a.torque_nm = frame.tau.get(ji).copied();
            a.health.valid = true;
            a.health.age = std::time::Duration::ZERO;
            a.health.fault_raw = 0;
        }

        let rpy = self
            .sim
            .body_world_orientation(&self.root_link)
            .map(|q| {
                let (r, p, y) = q.euler_angles();
                [r, p, y]
            })
            .unwrap_or([0.0; 3]);
        // **ジャイロは胴体座標系で返す。** 実機の IMU はストラップダウンで
        // 胴体に固定されているので、`Imu::gyro_rad_s` は胴体座標という約束。
        // MuJoCo は世界座標で持っているので回してから入れる。**回さずに
        // 入れていた（2026-09-06 まで）** — 傾きが小さいうちは差が出ないが、
        // 姿勢を使う制御（WBC・MPC）を入れると効いてくる。
        let gyro_world = self
            .sim
            .body_world_angular_velocity(&self.root_link)
            .unwrap_or([0.0; 3]);
        let r_wb = nalgebra::Rotation3::from_euler_angles(rpy[0], rpy[1], rpy[2]);
        let gyro_body = r_wb.transpose() * nalgebra::Vector3::from(gyro_world);
        let gyro = [gyro_body.x, gyro_body.y, gyro_body.z];
        // 加速度計。**モデルに IMU サイトが無い**ので、胴体速度（世界座標）
        // の差分で並進加速度を作り、重力を足して胴体座標へ回す
        // （実機のストラップダウン IMU と同じ約束: 重力込み・胴体座標）。
        // 最初の周期は差分が取れないので重力だけ。
        let v_world = self
            .sim
            .body_world_linear_velocity(&self.root_link)
            .unwrap_or([0.0; 3]);
        let a_world = match self.prev_base_vel_world {
            Some((v0, t0)) if frame.time > t0 => {
                let inv = 1.0 / (frame.time - t0);
                nalgebra::Vector3::new(
                    (v_world[0] - v0[0]) * inv,
                    (v_world[1] - v0[1]) * inv,
                    (v_world[2] - v0[2]) * inv,
                )
            }
            _ => nalgebra::Vector3::zeros(),
        };
        self.prev_base_vel_world = Some((v_world, frame.time));
        let accel_body = r_wb.transpose() * (a_world + nalgebra::Vector3::new(0.0, 0.0, G));
        obs.imu = Some(Imu {
            rpy_rad: rpy,
            gyro_rad_s: gyro,
            accel_m_s2: [accel_body.x, accel_body.y, accel_body.z],
            age: std::time::Duration::ZERO,
        });

        // 接地。足リンクが世界と触れているか。
        for c in obs.contacts.iter_mut() {
            *c = Some(false);
        }
        if self.contact_threshold_n > 0.0 && self.feet.len() == 4 {
            // **力で切る。** 幾何の接触は遊脚がかすっただけでも立つ。
            let feet: [&str; 4] = [
                self.feet[0].as_str(),
                self.feet[1].as_str(),
                self.feet[2].as_str(),
                self.feet[3].as_str(),
            ];
            let fz = self.sim.contact_force_per_foot(&feet);
            for (i, f) in fz.iter().enumerate() {
                if let Some(slot) = obs.contacts.get_mut(i) {
                    *slot = Some(*f > self.contact_threshold_n);
                }
            }
        } else {
            for contact in self.sim.contacts() {
                if contact.is_self_collision() {
                    continue;
                }
                for (i, foot) in self.feet.iter().enumerate() {
                    let hit = contact.body1.eq_ignore_ascii_case(foot)
                        || contact.body2.eq_ignore_ascii_case(foot);
                    if hit {
                        if let Some(slot) = obs.contacts.get_mut(i) {
                            *slot = Some(true);
                        }
                    }
                }
            }
        }
        Ok(())
    }
}
