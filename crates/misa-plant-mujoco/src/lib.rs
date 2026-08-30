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
            base_height_m: 0.30,
            home: Vec::new(),
            friction: None,
            timestep_s: None,
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
    root_link: String,
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
            timestep: opts.timestep_s,
            default_friction: opts.friction.unwrap_or([0.7, 0.005, 0.0001]),
            ..MjcfExportOptions::default()
        };
        let mut sim = MujocoSim::new(&model, mjcf).map_err(|e| format!("MuJoCo: {e}"))?;
        // 直近のフレームしか読まないので、リングは最小で足りる。
        sim.set_trace_max(2);

        let timestep = sim.timestep();
        let frames_per_tick = ((opts.control_period_s / timestep).round() as u32).max(1);
        log::info!(
            "MuJoCo timestep {:.4} ms、制御周期 {:.2} ms → 1 tick あたり {} フレーム",
            timestep * 1e3,
            opts.control_period_s * 1e3,
            frames_per_tick
        );

        let caps = PlantCaps {
            modes: vec![ControlMode::Position],
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
            root_link: opts.root_link.clone(),
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
            match a.mode {
                // 位置アクチュエータに脱力は無いので、その場で保持する。
                ControlMode::Idle => {
                    let q = self.model.joint_positions[ji];
                    self.sim.set_position_target(ji, q);
                }
                _ => self.sim.set_position_target(ji, a.position_rad),
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
        let gyro = self
            .sim
            .body_world_angular_velocity(&self.root_link)
            .unwrap_or([0.0; 3]);
        // **真の加速度計ではない。** 重力を胴体座標へ回しただけで、並進加速
        // は入っていない。姿勢しか使っていない現状では足りるが、加速度を
        // 使う制御を入れるなら、モデルに IMU サイトを足して
        // `MujocoSim::imu_readings` から取ること。
        let (sr, cr) = rpy[0].sin_cos();
        let (sp, cp) = rpy[1].sin_cos();
        obs.imu = Some(Imu {
            rpy_rad: rpy,
            gyro_rad_s: gyro,
            accel_m_s2: [-sp * G, sr * cp * G, cr * cp * G],
            age: std::time::Duration::ZERO,
        });

        // 接地。足リンクが世界と触れているか。
        for c in obs.contacts.iter_mut() {
            *c = Some(false);
        }
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
        Ok(())
    }
}
