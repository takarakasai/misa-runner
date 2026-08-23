//! 動力学込みで歩容を確かめる。
//!
//! `dump` と同じ台本を、[`misa_plant_mujoco::MujocoPlant`] の上で回す。
//! `dump` が「指令がそのまま実現したら関節角はどうなるか」なのに対し、
//! こちらは**重力と接触の中で本当に立っていられるか**を見る。
//!
//! # これは 2 つ目の Plant 実装でもある
//!
//! 実装が 1 つしかないトレイトは、それが正しい継ぎ目か誰にも分からない。
//! ここで `SerialPlant` と同じ `exchange` を通して同じ制御則が回ることが、
//! 抽象の側の検証になっている。
//!
//! # 見られないもの
//!
//! RS485 の往復遅れ・バスのジッタ・モータの一次遅れ・受信断は再現されない。
//! **脱力も再現されない**（位置アクチュエータにはその概念が無いので、
//! `Idle` の軸はその場で保持される）。

use std::time::{Duration, Instant};

use misa_core::{Pilot as _, Plant as _};
use misa_plant_mujoco::{MujocoPlant, SimOptions};

use crate::config::AppConfig;
use crate::controller::{Controller, State};
use crate::teleop::{GaitSelect, ModeRequest};
use misa_core::{Intent, Velocity};
use crate::viz;
use crate::Cli;

pub fn run(cfg: &AppConfig, cli: &Cli) -> Result<(), String> {
    let gait = match cli.str("gait").unwrap_or("trot") {
        "crawl" => GaitSelect::Crawl,
        "walk" => GaitSelect::Walk,
        "trot" => GaitSelect::Trot,
        other => return Err(format!("未知の歩容 {other:?}（crawl|walk|trot）")),
    };
    let vx = cli.f64("vx").unwrap_or(0.0);
    let vy = cli.f64("vy").unwrap_or(0.0);
    let wz = cli.f64("wz").unwrap_or(0.0);
    let seconds = cli.f64("secs").unwrap_or(6.0);
    let every = cli.usize("every").unwrap_or(200).max(1);
    let viz_cfg = crate::viz_config(cli);
    // **可視化するなら実時間で流す。** 全力で回すと 10 秒ぶんが 1 秒で
    // 終わって、目でも手でも追えない。操縦するときも同じ。
    let realtime = cli.flag("realtime") || viz_cfg.enabled || cli.str("pilot") == Some("sbus");

    // **Pilot を先に開く。** 受信機が無いのにモデルを読んでから落ちると、
    // 待たされたうえで原因が最後に出る。
    // **台本もプロポも同じ Pilot。** どちらから入っても制御則は同じ経路を
    // 通るので、プロポの解釈（チャンネル・不感帯・エクスポ）を実機を
    // 壊さずに確かめられる。
    let mut pilot: Box<dyn misa_core::Pilot> = match cli.str("pilot").unwrap_or("script") {
        "script" => Box::new(crate::pilot::ScriptPilot::new(Intent {
            mode: ModeRequest::Walk,
            gait,
            aux_rad: vec![None],
            link_ok: true,
            ..Intent::default()
        })),
        "sbus" => {
            // 受信機だけ開く。脚バスも IMU も MuJoCo の側にある。
            let map = misa_hal::ch348::PortMap::discover().map_err(|e| e.to_string())?;
            let p = crate::pilot::SbusPilot::connect_with(cfg, &map, false)?;
            println!("プロポから操縦します（CH5 が脱力位置で待機）");
            Box::new(p)
        }
        other => return Err(format!("未知の pilot {other:?}（script|sbus）")),
    };
    let scripted = cli.str("pilot").unwrap_or("script") == "script";

    let robot = crate::robot::load_from_config(cfg)?;
    let model_limits = robot.limits.clone();
    let layout = crate::snapshot::axis_layout(cfg)?;
    let dt = 1.0 / cfg.control.rate_hz;

    // **物理の初期姿勢は伏せ姿勢。** 実機は電源投入時にそこにいるので、
    // 立ち上がりの軌道を同じ始点から見るため。
    let crouch = crate::robot::rest_pose(cfg, &robot);
    let home: Vec<(String, f64)> = layout
        .table
        .axes()
        .iter()
        .enumerate()
        .filter_map(|(i, a)| {
            let leg = i / 3;
            let k = i % 3;
            // 脚だけ。補助軸（腕・車輪）の初期姿勢はモデルの既定に任せる。
            (i < crate::snapshot::AxisLayout::LEG_AXES).then(|| (a.name.clone(), crouch.legs[leg][k]))
        })
        .collect();

    let opts = SimOptions {
        misa_path: cfg.control.model.clone(),
        control_period_s: dt,
        actuator_kp: cli.f64("kp").unwrap_or(60.0),
        actuator_kv: cli.f64("kv").unwrap_or(1.0),
        base_height_m: cli.f64("base-height").unwrap_or(0.20),
        home,
        root_link: robot.root_link.clone(),
        ..SimOptions::default()
    };
    let mut plant = MujocoPlant::new(layout.table.clone(), &opts)?;

    let mut controller = Controller::new(robot, cfg.clone());
    let mut shadow_gate = misa_core::SafetyGate::new(crate::snapshot::safety_config(cfg, &layout, &model_limits, dt, 5.0));
    let recorder = match cli.str("record") {
        Some(path) => {
            let header = misa_core::record::Header {
                format_version: misa_core::record::FORMAT_VERSION,
                robot: cfg.name.clone(),
                axes: layout.table.axes().iter().map(|a| a.name.clone()).collect(),
                rate_hz: cfg.control.rate_hz,
            };
            println!("毎周期を {path} に記録します");
            Some(crate::record::Recorder::create(path, &header)?)
        }
        None => None,
    };


    let mut obs = misa_core::Observation::empty(layout.table.len(), 4);
    plant.exchange(&misa_core::Command::idle(layout.table.len()), &mut obs)?;

    println!(
        "MuJoCo で歩容 {} / v=({vx:+.3}, {vy:+.3}, {wz:+.3}) / {:.0} Hz / {seconds:.1} s",
        gait.label(),
        cfg.control.rate_hz
    );
    println!("t[s]   状態         胴体 z[m]  roll   pitch  接地");

    let steps = (seconds / dt).ceil() as usize;
    let mut min_z = f64::INFINITY;
    let mut script_velocity = Velocity::ZERO;
    let mut publisher = crate::runner::open_viz(&viz_cfg)?;
    let period = Duration::from_secs_f64(dt);
    let mut next = Instant::now();
    let mut fell = None;

    for i in 0..steps {
        let t = i as f64 * dt;
        // 台本のときだけ、立ち上がってから速度を入れる。プロポのときは
        // 操縦者が入れるので触らない。
        if scripted && controller.state() == State::Active {
            script_velocity = Velocity {
                vx_m_s: vx,
                vy_m_s: vy,
                wz_rad_s: wz,
            };
        }
        let mut cmd = pilot.poll(obs.time);
        if scripted {
            cmd.velocity = script_velocity;
        }
        let measured = jointvec_from(&obs);
        let attitude = obs.imu.map(|m| m.rpy_rad).unwrap_or([0.0; 3]);
        let out = controller.tick(&cmd, &measured, attitude, dt);

        let outgoing = crate::snapshot::command(
            &layout,
            &out.targets,
            cfg.hardware.default_max_speed_rad_s(),
            out.leg_mode == misa_hal::joint::JointMode::Idle,
        );
        if let Some(rec) = recorder.as_ref() {
            let mut shadow = outgoing.clone();
            let verdict =
                shadow_gate.apply(&mut shadow, &obs, Duration::from_secs_f64(dt));
            rec.push(misa_core::record::Frame {
                seq: i as u64,
                time: obs.time,
                intent: cmd.clone(),
                observation: obs.clone(),
                command: outgoing.clone(),
                verdict,
            });
        }
        plant.exchange(&outgoing, &mut obs)?;

        let z = plant.base_position().map(|p| p[2]).unwrap_or(f64::NAN);
        min_z = min_z.min(z);
        let att = obs.imu.map(|m| m.rpy_rad).unwrap_or([0.0; 3]);
        // **転倒は姿勢で見る。** 高さだけだとしゃがんだ姿勢と区別が付かない。
        if fell.is_none() && (att[0].abs() > 1.0 || att[1].abs() > 1.0) {
            fell = Some(t);
        }
        // **planned（指令）と measured（MuJoCo の実測）を両方流す。**
        // 受け側はゴーストで重ねて描くので、追従できていない軸が目で分かる。
        if let Some(p) = publisher.as_mut() {
            let body = controller.body_view();
            let measured_body = viz::BodyView {
                rp: [att[0], att[1]],
                ..body
            };
            let planned = out.targets;
            p.maybe_publish(|seq| {
                viz::Frames::both(
                    viz::frame(seq, t, &planned, &body),
                    viz::frame(seq, t, &measured, &measured_body),
                )
            });
        }

        if realtime {
            next += period;
            let now = Instant::now();
            if next > now {
                std::thread::sleep(next - now);
            } else {
                next = now;
            }
        }

        if i % every == 0 {
            let feet: String = obs
                .contacts
                .iter()
                .map(|c| match c {
                    Some(true) => '■',
                    Some(false) => '□',
                    None => '?',
                })
                .collect();
            println!(
                "{t:5.2}  {:<12} {z:8.3}  {:+.3} {:+.3}  {feet}",
                out.state.label(),
                att[0],
                att[1]
            );
        }
    }

    if let Some(rec) = recorder {
        let dropped = rec.dropped();
        match rec.finish() {
            Ok(n) if dropped == 0 => println!("{n} 周期を記録しました"),
            Ok(n) => println!("{n} 周期を記録しました（{dropped} 周期は取りこぼし）"),
            Err(e) => return Err(e),
        }
    }

    let end = plant.base_position().unwrap_or([f64::NAN; 3]);
    println!(
        "\n終端 胴体位置 ({:+.3}, {:+.3}, {:+.3})  最低高さ {min_z:.3} m",
        end[0], end[1], end[2]
    );
    match fell {
        Some(t) => Err(format!("**転倒しました**（t={t:.2} s で胴体が 1 rad 以上傾いた）")),
        None => {
            println!("転倒なし");
            Ok(())
        }
    }
}

fn jointvec_from(obs: &misa_core::Observation) -> crate::jointvec::JointVec {
    let mut q = crate::jointvec::JointVec::zeros();
    for leg in 0..4 {
        for k in 0..3 {
            if let Some(a) = obs.get(misa_core::AxisId::new((leg * 3 + k) as u16)) {
                q.legs[leg][k] = a.position_rad;
            }
        }
    }
    if let Some(a) = obs.get(misa_core::AxisId::new(12)) {
        q.arm = a.position_rad;
    }
    q
}

