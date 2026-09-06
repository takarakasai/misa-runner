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

use misa_core::Plant as _;
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
    let realtime = cli.flag("realtime")
        || viz_cfg.enabled
        || matches!(cli.str("pilot"), Some("sbus") | Some("ros2") | Some("keys"));

    // **Pilot を先に開く。** 受信機が無いのにモデルを読んでから落ちると、
    // 待たされたうえで原因が最後に出る。
    // **台本もプロポも同じ Pilot。** どちらから入っても制御則は同じ経路を
    // 通るので、プロポの解釈（チャンネル・不感帯・エクスポ）を実機を
    // 壊さずに確かめられる。
    // **歩容パラメータの上書き。** 与えなかった項目はプロファイルのまま。
    // キーボード（`--pilot keys`）で触るのと同じ経路を通るので、台本で
    // 詰めた値をそのままプロファイルへ書き戻せる。
    let tune = misa_core::GaitTune {
        cycle_period_s: cli.f64("cycle"),
        swing_height_m: cli.f64("swing"),
        step_length_m: cli.f64("step-length"),
        duty_factor: cli.f64("duty"),
    }
    .clamped();
    // **途中で替える。** 歩きながら替えても跳ねないかを見るため。
    let tune_at = cli.f64("tune-at");

    let mut pilot: Box<dyn misa_core::Pilot> = match cli.str("pilot").unwrap_or("script") {
        "script" => Box::new({
            let mut p = crate::pilot::ScriptPilot::new(Intent {
            mode: ModeRequest::Walk,
            gait,
            aux_rad: vec![None],
            link_ok: true,
            // 胴体の傾きを打ち消すようにヘッド軸を動かす。ヘッド軸を持た
            // ない機体（namiashi2 は車輪 4 軸だけ）では立てても何も起きない。
            stabilize_head: cli.flag("chicken"),
            // 胴体を傾けたまま歩く。**足は接地したまま胴体だけ回る**ので、
            // チキンヘッドが効いているかはここを振ると見える。
            // `gait.body_attitude_max_rad` が 0 なら効かない（既定）。
            body_attitude_rad: [
                cli.f64("tilt-roll").unwrap_or(0.0),
                cli.f64("tilt-pitch").unwrap_or(0.0),
                cli.f64("tilt-yaw").unwrap_or(0.0),
            ],
            // 台本は最初から入れる（`--tune-at` があるときは後で入る）。
            gait_tune: if tune_at.is_some() {
                misa_core::GaitTune::default()
            } else {
                tune
            },
            ..Intent::default()
            });
            if let Some(at) = tune_at {
                p.tune_at(at, tune);
                println!("{at:.1} s で歩容パラメータを替えます");
            }
            p
        }),
        // **キーボード。** MuJoCo を見ながら手で動かす用。押しっぱなしは
        // 端末から取れないので、押すたびに 1 段ずつ足す形になっている。
        "keys" => Box::new(crate::pilot_keys::KeyPilot::open(cfg, gait)?),
        "sbus" => {
            // 受信機だけ開く。脚バスも IMU も MuJoCo の側にある。
            let map = misa_hal::ch348::PortMap::discover().map_err(|e| e.to_string())?;
            let p = crate::pilot::SbusPilot::connect_with(cfg, &map, false)?;
            println!("プロポから操縦します（CH5 が脱力位置で待機）");
            Box::new(p)
        }
        #[cfg(feature = "ros2")]
        "ros2" => {
            let p = crate::pilot_ros2::Ros2Pilot::connect(cfg)?;
            println!("ROS 2 から操縦します（cmd_vel + サービス 5 本。既定は脱力）");
            Box::new(p)
        }
        other => {
            return Err(format!(
                "未知の pilot {other:?}（script|keys|sbus{}）",
                if cfg!(feature = "ros2") { "|ros2" } else { "" }
            ))
        }
    };
    let scripted = cli.str("pilot").unwrap_or("script") == "script";
    // プロポと ROS は操縦者が止めるまで回す。台本は --secs で切る。
    let interactive = !scripted;

    let robot = crate::robot::load_from_config(cfg)?;
    // **当たり判定のメッシュが欠けたまま動力学を回さない。** 落ちたメッシュ
    // のリンクは何にも当たらなくなるので、結果は「うまく動いている」ように
    // 見えてしまう。namiashi2 でこれに引っかかった (2026-09-02)。
    if !robot.bad_meshes.is_empty() {
        return Err(format!(
            "当たり判定のメッシュを {} 件読めません。**このまま回すと当たり判定の\
             無いリンクができて、衝突しないぶん動いて見えてしまいます。**\n  {}",
            robot.bad_meshes.len(),
            robot.bad_meshes.join("\n  ")
        ));
    }
    let model_limits = robot.limits.clone();
    let model_rates = robot.rate_limits.clone();
    let model_efforts = robot.effort_limits.clone();
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
        // **QP の上限とアクチュエータの上限を揃える。** 揃えないと、QP は
        // 出ないトルクを当てにした解を出す。
        torque_scale: cfg.wbc.torque_scale,
        // **速度制御のゲインは位置制御のものと別。** 詳しくは
        // `SimOptions::velocity_kv`。
        velocity_kv: cli.f64("kv-velocity").unwrap_or(20.0),
        base_height_m: cli.f64("base-height").unwrap_or(0.20),
        timestep_s: cli.f64("timestep"),
        contact_threshold_n: cfg.wbc.contact_force_threshold_n,
        friction: cli.f64("friction").map(|f| [f, 0.005, 0.0001]),
        home,
        root_link: robot.root_link.clone(),
        ..SimOptions::default()
    };
    let mut plant = MujocoPlant::new(layout.table.clone(), &opts)?;

    // **絵を PNG で落とす。** articara の GUI（--viz）は関節角だけで、
    // 接地も地面も出ない。動画にするならこちら。
    #[cfg(feature = "render")]
    let mut video = match cli.str("video") {
        Some(dir) => {
            plant.start_recording(&misa_plant_mujoco::RenderOptions {
                outdir: dir.to_string(),
                // **既定は 640x480。** MuJoCo のオフスクリーンバッファの既定が
                // これで、超えると描かれた領域だけが左上に寄って黒帯が出る。
                // 大きくするならモデルの <visual><global offwidth/offheight>
                // が要るが、articara のエクスポータはそれを出していない。
                width: cli.usize("width").unwrap_or(640) as u32,
                height: cli.usize("height").unwrap_or(480) as u32,
                azimuth: cli.f64("cam-az").unwrap_or(120.0),
                elevation: cli.f64("cam-el").unwrap_or(-12.0),
                // **機体の大きさで変える。** namiashi2 は namiashi より大きい。
                distance: cli.f64("cam-dist").unwrap_or(1.6),
                look_z: cli.f64("cam-z").unwrap_or(0.22),
                look_xy: [cli.f64("cam-x").unwrap_or(0.0), cli.f64("cam-y").unwrap_or(0.0)],
                fixed: cli.flag("cam-fixed"),
            })?;
            println!("MuJoCo の絵を {dir} へ {} fps で落とします", cli.f64("fps").unwrap_or(30.0));
            Some(cli.f64("fps").unwrap_or(30.0))
        }
        None => None,
    };
    #[cfg(not(feature = "render"))]
    let video: Option<f64> = match cli.str("video") {
        Some(_) => return Err("このビルドには render が入っていません（--features render）".into()),
        None => None,
    };
    let mut next_frame_at = 0.0_f64;

    // **シムでは補助軸もこちらの指令で動く。** 実機の namiashi は腕が
    // 受信機直結で駆動できないが、MuJoCo の中では動かせるので、
    // チキンヘッドの検証はここでしかできない。
    let head_driven = layout
        .head
        .is_some_and(|id| plant.capabilities().driven.get(id.index()) == Some(&true));
    let mut wbc = crate::wbc::WbcRunner::new(&robot, &cfg.wbc)?;
    let mut estimator = crate::estimator::BodyEstimator::new(&robot, cfg.gait.estimator);
    let mut controller = Controller::with_arm(robot, cfg.clone(), head_driven);
    // **可動域は `dump` と同じ表で、同じ関数で見る。**
    //
    // ここを入れる前は `sim` だけが素通りしていた。可動域を破る膝の向きを
    // 設定に入れたまま掃引して、「速く前へ進む」設定として選んでしまった
    // ことがある (2026-09-01)。実際には後脚の膝が可動域に当たって脚の
    // 長さが変わっていただけで、歩行ではなかった。**動力学が付くと
    // それらしく動いてしまうぶん、シムのほうが誤魔化されやすい。**
    let limits = crate::snapshot::safety_config(cfg, &layout, &model_limits, &model_rates, &model_efforts, dt, 5.0);
    let mut violations: Vec<String> = Vec::new();
    let mut shadow_gate = misa_core::SafetyGate::new(limits.clone());
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

    let mut clear_max = [f64::NEG_INFINITY; 4];
    let mut clear_min = [f64::INFINITY; 4];
    let mut foot_prev: [Option<[f64; 3]>; 4] = [None; 4];
    let mut slip_sum = [0.0f64; 4];
    let mut slip_n = [0usize; 4];
    let mut yaw_prev = 0.0f64;
    let mut yaw_total = 0.0f64;
    let mut track_sum = vec![0.0f64; layout.table.len()];
    let mut track_max = vec![0.0f64; layout.table.len()];
    let mut track_n = 0usize;
    let mut ground_contacts: std::collections::BTreeMap<String, usize> = Default::default();
    let start = plant.base_position().unwrap_or([0.0; 3]);
    let start_yaw = obs.imu.map(|m| m.rpy_rad[2]).unwrap_or(0.0);

    println!(
        "MuJoCo で歩容 {} / v=({vx:+.3}, {vy:+.3}, {wz:+.3}) / {:.0} Hz / {seconds:.1} s",
        gait.label(),
        cfg.control.rate_hz
    );
    // **操縦しているときは指令も出す。** 台本なら固定なので出さない。
    let show_pilot = !scripted;
    println!(
        "t[s]   状態         胴体 z[m]  roll   pitch  yaw    接地{}",
        if show_pilot { "  操縦" } else { "" }
    );

    // `--secs 0` で操縦者が止めるまで（Ctrl-C）。
    let steps = if seconds <= 0.0 && interactive {
        usize::MAX
    } else {
        (seconds / dt).ceil() as usize
    };
    let mut min_z = f64::INFINITY;
    let mut script_velocity = Velocity::ZERO;
    let mut publisher = crate::runner::open_viz(&viz_cfg)?;
    let period = Duration::from_secs_f64(dt);
    let mut next = Instant::now();
    let mut fell = None;

    for i in 0..steps {
        let t = i as f64 * dt;
        // **操縦者の終了要求。** キーボードは端末を raw モードにするので
        // Ctrl-C が SIGINT にならない。ここを見ないと止められない。
        if pilot.quit_requested() {
            println!("\n操縦者が終了を要求しました（{t:.1} s）");
            break;
        }
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
        let mut out = controller.tick(&cmd, &measured, attitude, dt);
        if cfg.wbc.use_measured_contact {
            out.stance = crate::estimator::stance_with_measured_contact(out.stance, &obs);
        }
        let measured_qd = crate::estimator::velocities_from(&obs);
        let gyro = obs.imu.map(|m| m.gyro_rad_s).unwrap_or([0.0; 3]);
        if controller.state() != State::Active {
            estimator.reset();
        }
        let body = estimator.estimate(
            &measured,
            &measured_qd,
            &out.targets,
            attitude,
            gyro,
            obs.imu.map(|m| m.accel_m_s2),
            out.stance,
            dt,
        );
        controller.observe_body(&body);

        crate::dump::check_limits(&limits, &layout, &out.targets, t, &mut violations);

        let plan = wbc
            .as_mut()
            .and_then(|w| w.tick(&out, &obs, &measured, &measured_qd, &body, dt));
        // 調査用: WBC の τ と Plant が実際に掛けたトルクを脚ごとに並べる
        // （README「調べ方」）。位置出力で立ち止まらせれば Plant 側は重力補償
        // そのものなので、モデルの検算になる。
        let tau_every = std::env::var("MISA_WBC_TAU")
            .ok()
            .and_then(|v| v.parse::<usize>().ok())
            .filter(|e| *e > 0);
        if tau_every.is_some_and(|e| i % e == 0) {
            if let Some(p) = plan.as_ref() {
                let mut line = format!("[tau] t={t:.3} h={:.3}", body.height_m.unwrap_or(f64::NAN));
                for leg in 0..2 {
                    line += &format!(" q{leg}[{:+.2},{:+.2},{:+.2}]", measured.legs[leg][0], measured.legs[leg][1], measured.legs[leg][2]);
                }
                for leg in 0..4 {
                    let mut w = Vec::new();
                    let mut m = Vec::new();
                    for k in 0..3 {
                        w.push(format!("{:+.2}", p.legs[leg][k].torque_nm));
                        let tm = obs
                            .get(misa_core::AxisId::new((leg * 3 + k) as u16))
                            .and_then(|a| a.torque_nm)
                            .unwrap_or(f64::NAN);
                        m.push(format!("{tm:+.2}"));
                    }
                    line += &format!(" L{leg} wbc[{}] meas[{}]", w.join(","), m.join(","));
                }
                eprintln!("{line}");
            }
        }
        let outgoing = crate::snapshot::command(
            &layout,
            &out.targets,
            cfg.hardware.default_max_speed_rad_s(),
            out.leg_mode == misa_hal::joint::JointMode::Idle,
            cfg.hardware.mit_gains(),
            plan.as_ref(),
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

        // **追従誤差は歩幅と比べて意味を持つ。** 誤差が歩幅を超えていれば、
        // 歩容をどう振っても結果は動力学の都合で決まる。立ち上がりは
        // 大きく外れて当たり前なので、歩容に入ってからだけ数える。
        if out.state == State::Active {
            for (i, e) in track_sum.iter_mut().enumerate() {
                let id = misa_core::AxisId::new(i as u16);
                let (Some(c), Some(o)) = (outgoing.get(id), obs.get(id)) else {
                    continue;
                };
                let d = (c.position_rad - o.position_rad).abs();
                *e += d;
                if d > track_max[i] {
                    track_max[i] = d;
                }
            }
            track_n += 1;
        }

        // **遊脚が地面から離れているか。**
        //
        // 指令の `swing_height_m` だけ上がっていなければ、足は遊脚のあいだ
        // 地面を前へ引きずり、胴体を後ろへ押す。**接地率だけ見ていても
        // 「浮いていない」とは分かるが、どれだけ足りないかは分からない。**
        if out.state == State::Active {
            for (i, p) in plant.foot_positions().iter().enumerate() {
                let Some(p) = p else { continue };
                if obs.contacts.get(i).copied().flatten() == Some(false) && p[2] > clear_max[i] {
                    clear_max[i] = p[2];
                }
                if p[2] < clear_min[i] {
                    clear_min[i] = p[2];
                }
            }
        }

        // **接地している足が地面の上を滑っていないか。**
        //
        // 歩容は「接地中の足を胴体に対して -v で引く」ことで前へ進む。足が
        // 地面に対して静止していれば胴体はぴったり v で進む。**滑っていれば、
        // 指令より速くも遅くもなる**ので、進んだ距離が指令と合わないときに
        // 追従誤差と並べて見る値。
        {
            let fp = plant.foot_positions();
            for (i, p) in fp.iter().enumerate() {
                let (Some(p), Some(prev)) = (p, foot_prev[i]) else {
                    foot_prev[i] = *p;
                    continue;
                };
                if out.state == State::Active && obs.contacts.get(i).copied().flatten() == Some(true)
                {
                    let d = ((p[0] - prev[0]).powi(2) + (p[1] - prev[1]).powi(2)).sqrt();
                    slip_sum[i] += d;
                    slip_n[i] += 1;
                }
                foot_prev[i] = Some(*p);
            }
        }

        // **ヨーは ±π で折り返す。** そのままだと旋回の総量も向きも読めない
        // （+0.5 rad/s を 14 秒で +7 rad 回るのに、表示は -171° になる）。
        // 差分を畳んで足し込む。
        {
            let y = obs.imu.map(|m| m.rpy_rad[2]).unwrap_or(0.0);
            let mut d = y - yaw_prev;
            while d > std::f64::consts::PI {
                d -= std::f64::consts::TAU;
            }
            while d < -std::f64::consts::PI {
                d += std::f64::consts::TAU;
            }
            yaw_total += d;
            yaw_prev = y;
        }

        let z = plant.base_position().map(|p| p[2]).unwrap_or(f64::NAN);
        min_z = min_z.min(z);
        // **足以外が地面に触れていたら、それは歩行ではない。** 数えておいて
        // 最後に出す。毎周期出すと流れてしまう。
        for body in plant.non_foot_ground_contacts() {
            *ground_contacts.entry(body).or_insert(0usize) += 1;
        }
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

        // フレームは時刻で刻む。制御周期と動画のフレームレートは別物。
        if let Some(fps) = video {
            if t >= next_frame_at {
                #[cfg(feature = "render")]
                plant.capture()?;
                next_frame_at += 1.0 / fps.max(1.0);
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
                "{t:5.2}  {:<12} {z:8.3}  {:+.3} {:+.3} {:+.3}  {feet}{}",
                out.state.label(),
                att[0],
                att[1],
                att[2],
                if show_pilot {
                    format!("  {}", pilot.status_line())
                } else {
                    String::new()
                }
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

    #[cfg(feature = "render")]
    if video.is_some() {
        println!("{} フレーム書きました", plant.frames());
    }
    // **`render` 無しでは `video` は None のまま使われない。** 未使用の
    // 警告だけを消す（`&mut` を取ると、あちらでは `mut` が付いていないので
    // ビルドが落ちる。実際 `--features sim` 単体が通らなくなっていた）。
    let _ = &video;

    let end = plant.base_position().unwrap_or([f64::NAN; 3]);
    // **世界座標の移動量だけでは「前へ歩いたか」は分からない。** 機体が
    // ヨーしていれば前進が世界の −x に出る。歩容の指令は機体座標なので、
    // 出発時の向きへ射影した前後・左右も添える。
    let (dx, dy) = (end[0] - start[0], end[1] - start[1]);
    let (s0, c0) = start_yaw.sin_cos();
    let (fwd, lat) = (dx * c0 + dy * s0, -dx * s0 + dy * c0);
    let yaw_end = obs.imu.map(|m| m.rpy_rad[2]).unwrap_or(f64::NAN);
    println!(
        "\n終端 胴体位置 ({:+.3}, {:+.3}, {:+.3})  最低高さ {min_z:.3} m",
        end[0], end[1], end[2]
    );
    println!(
        "機体座標の移動 前後 {fwd:+.3} m / 左右 {lat:+.3} m  回った量 {:+.1}°（いまの向き {:+.1}°）",
        yaw_total.to_degrees(),
        yaw_end.to_degrees()
    );
    if clear_max.iter().any(|v| v.is_finite()) {
        let s: String = (0..4)
            .map(|i| {
                format!(
                    "{} {:.3}  ",
                    ["FL", "FR", "RL", "RR"][i],
                    clear_max[i] - clear_min[i]
                )
            })
            .collect();
        println!("遊脚で上がった高さ [m]（gait.swing_height_m に届いているか）  {s}");
    }
    if slip_n.iter().any(|&n| n > 0) {
        let s: String = (0..4)
            .map(|i| {
                format!(
                    "{} {:.3}  ",
                    ["FL", "FR", "RL", "RR"][i],
                    slip_sum[i] / (slip_n[i].max(1) as f64) / dt
                )
            })
            .collect();
        println!("\n接地中の足の滑り [m/s]（0 に近いほど良い）  {s}");
    }
    if track_n > 0 {
        println!("\n追従誤差（歩容中 {track_n} 周期の平均 / 最大） [rad]");
        for (leg, names) in misa_hal::joint::JOINT_NAMES.iter().enumerate() {
            let mut line = String::new();
            for (k, jn) in names.iter().enumerate() {
                let i = leg * 3 + k;
                line.push_str(&format!(
                    "{:<6}{:.3}/{:.3}  ",
                    jn.get(3..jn.len() - 6).unwrap_or(jn),
                    track_sum[i] / track_n as f64,
                    track_max[i]
                ));
            }
            println!("  {}  {line}", ["FL", "FR", "RL", "RR"][leg]);
        }
    }
    if !ground_contacts.is_empty() {
        let total = steps.min(1 + (seconds / dt) as usize);
        println!("\n**足以外が接地しています**（歩行ではなく、これに乗っているかもしれません）:");
        let mut rows: Vec<_> = ground_contacts.iter().collect();
        rows.sort_by_key(|(_, n)| std::cmp::Reverse(**n));
        for (body, n) in rows.iter().take(8) {
            println!("  {body}  {:.0}% の周期", 100.0 * **n as f64 / total as f64);
        }
    }
    if !violations.is_empty() {
        println!("\n可動域を超えた指令が {} 件あります:", violations.len());
        for v in violations.iter().take(20) {
            println!("  {v}");
        }
    }
    match fell {
        Some(t) => Err(format!("**転倒しました**（t={t:.2} s で胴体が 1 rad 以上傾いた）")),
        None if !violations.is_empty() => Err(format!(
            "可動域を超える指令が {} 件出ています。この結果を歩容の良し悪しの\
             判断に使わないでください（脚が可動域に当たって長さが変わります）",
            violations.len()
        )),
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

