//! `dump` — 実機なしで歩容を再生し、関節角が可動域に収まるか確かめる。
//!
//! モータを繋ぐ前にここで可動域を超える指令が出ていないか見ておくのが、
//! いちばん安い安全確認になる。状態機械そのもの（脱力 → 初期姿勢 → 立ち姿勢
//! → 歩容）を通すので、遷移の途中で行き過ぎる場合もここに出る。

use std::time::{Duration, Instant};

use misa_hal::imu::ImuSample;
use misa_hal::joint::JOINT_NAMES;

use crate::config::AppConfig;
use crate::controller::{Controller, State};
use crate::jointvec::JointVec;
use crate::teleop::{GaitSelect, ModeRequest};
use misa_core::{Intent, Velocity};
use crate::viz::{self, VizConfig};
use crate::Cli;

pub fn run(cfg: &AppConfig, cli: &Cli) -> Result<(), String> {
    let gait = match cli.str("gait").unwrap_or("crawl") {
        "crawl" => GaitSelect::Crawl,
        "walk" => GaitSelect::Walk,
        "trot" => GaitSelect::Trot,
        other => return Err(format!("未知の歩容 {other:?}（crawl|walk|trot）")),
    };
    let vx = cli.f64("vx").unwrap_or(0.05);
    let vy = cli.f64("vy").unwrap_or(0.0);
    let wz = cli.f64("wz").unwrap_or(0.0);
    let seconds = cli.f64("secs").unwrap_or(4.0);
    // 胴体を傾けたときに可動域へ収まるかを、実機に触れずに確かめる。
    // **傾けると脚の可動域を食う**ので、`body_attitude_max_rad` を上げる前に
    // ここで当たりを取る。
    let tilt = [
        cli.f64("tilt-roll").unwrap_or(0.0),
        cli.f64("tilt-pitch").unwrap_or(0.0),
        cli.f64("tilt-yaw").unwrap_or(0.0),
    ];
    let every = cli.usize("every").unwrap_or(20).max(1);
    let viz_cfg = crate::viz_config(cli);
    // 可視化するときは実時間で流さないと早送りになる。--realtime は
    // --viz の有無に関わらず指定できる（表示を目で追いたいときにも使う）。
    let realtime = cli.flag("realtime") || viz_cfg.enabled;

    let robot = crate::robot::load_from_config(cfg)?;
    // Controller へ move する前に控えておく。
    let model_limits = robot.limits.clone();
    let model_rates = robot.rate_limits.clone();
    let rest = crate::robot::rest_pose(cfg, &robot);
    let mut controller = Controller::new(robot, cfg.clone());
    let dt = 1.0 / cfg.control.rate_hz;
    let imu = level_imu();
    // **実測値の代わりに「伏せ姿勢」を食わせる。**
    //
    // 脱力からの遷移は実測値を始点に張るので、ここをゼロにすると
    // **実機では起こらない軌道**が出る。実際、hip がゼロのまま動かない
    // ように見えて「hip は動かない」と誤読した (2026-08-22)。
    //
    // 伏せ姿勢のモデル角は定義上そのまま `zero_pose_rad`（電源投入時に
    // モータ角 0 = 伏せ、`q_model = sign * 0 + zero_pose_rad`）。
    let measured = rest;

    let mut cmd = Intent {
        velocity: Velocity::ZERO,
        height_offset_m: 0.0,
        aux_rad: vec![None],
        // **`Stand` ではない。** CH5 中段は初期姿勢で保持する仕様になったので、
        // `Stand` のままだとそこで止まって歩容へ進まない。速度は
        // `State::Active` に入ってから入れるので、最初から `Walk` でよい。
        mode: ModeRequest::Walk,
        gait,
        play_pose: false,
        // **`chicken_head` は立てない。** 姿勢は `body_attitude_rad` を直接
        // 渡すので不要で、立てると「腕が駆動できない」警告が出るだけ。
        // CH8 は実機で CH1/CH3 を読み替えるためのスイッチであって、
        // ここでは通る道が違う。
        stabilize_head: false,
        body_attitude_rad: tilt,
        link_ok: true,
        ..Intent::default()
    };

    println!(
        "歩容 {} / v=({vx:+.3}, {vy:+.3}, {wz:+.3}) / {:.0} Hz / {seconds:.1} s{}",
        gait.label(),
        cfg.control.rate_hz,
        if tilt == [0.0; 3] {
            String::new()
        } else {
            format!(
                " / 胴体 roll {:+.1}° pitch {:+.1}° yaw {:+.1}°",
                tilt[0].to_degrees(),
                tilt[1].to_degrees(),
                tilt[2].to_degrees()
            )
        }
    );
    println!("t[s]   状態         {}", header());

    let mut publisher = open_viz(&viz_cfg)?;

    // **実機なしで記録が採れる。** ここが CI に載る回帰試験の土台で、
    // 制御ループを触る改修は「dump を録って差分する」で検証できる。
    // ゲートは `run` と同じく影で回すだけ（出力は捨てる）。
    let layout = crate::snapshot::axis_layout(cfg)?;
    let limits = crate::snapshot::safety_config(cfg, &layout, &model_limits, &model_rates, dt, 5.0);
    let mut shadow_gate = misa_core::SafetyGate::new(limits.clone());
    let recorder = match cli.str("record") {
        Some(path) => {
            let header = misa_core::record::Header {
                format_version: misa_core::record::FORMAT_VERSION,
                robot: cfg.name.clone(),
                axes: layout.table.axes().iter().map(|a| a.name.clone()).collect(),
                rate_hz: 1.0 / dt,
            };
            let rec = crate::record::Recorder::create(path, &header)?;
            println!("毎周期を {path} に記録します");
            Some(rec)
        }
        None => None,
    };

    let mut violations: Vec<String> = Vec::new();
    let mut peak_rate = [0.0f64; 12];
    let mut prev_targets: Option<JointVec> = None;
    let steps = (seconds / dt).ceil() as usize;
    let period = Duration::from_secs_f64(dt);
    let mut next = Instant::now();
    for i in 0..steps {
        let t = i as f64 * dt;
        // 立ち上がってから歩き出す。遷移が終わるまで速度は入れない。
        if controller.state() == State::Active {
            cmd.mode = ModeRequest::Walk;
            cmd.velocity = Velocity {
                vx_m_s: vx,
                vy_m_s: vy,
                wz_rad_s: wz,
            };
        }
        let out = controller.tick(&cmd, &measured, imu.rpy_rad, dt);
        check_limits(&limits, &layout, &out.targets, t, &mut violations);
        // **歩容が要求する目標の変化率を測る。** 安全ゲートの上限を超えて
        // いたら、実機ではゲートが丸めて歩容が崩れる。namiashi2 で実際に起きた:
        // trot が calf に 16 rad/s を要求していて、モータの定格 10.47 も
        // 設定の 3.0 も超えていた (2026-09-02)。
        if let Some(p) = prev_targets.as_ref() {
            for leg in 0..4 {
                for k in 0..3 {
                    let i = leg * 3 + k;
                    let r = (out.targets.legs[leg][k] - p.legs[leg][k]).abs() / dt;
                    if r > peak_rate[i] {
                        peak_rate[i] = r;
                    }
                }
            }
        }
        prev_targets = Some(out.targets);
        if i % every == 0 {
            println!("{t:5.2}  {:<12} {}", out.state.label(), row(&out.targets));
        }
        if let Some(rec) = recorder.as_ref() {
            let time = misa_core::Time::from_secs_f64(t);
            // 実機を持たないので観測は「指令がそのまま実現した」ことにする。
            // 動力学は入っていない。ここが埋まるのは MuJoCo の Plant が
            // 入ってから。
            let mut obs = misa_core::Observation::empty(layout.table.len(), 4);
            obs.time = time;
            for (i, a) in obs_axes(&measured).into_iter().enumerate() {
                let slot = obs.get_mut(misa_core::AxisId::new(i as u16)).unwrap();
                slot.position_rad = a;
                slot.health.valid = true;
            }
            let mut shadow = crate::snapshot::command(
                &layout,
                &out.targets,
                cfg.hardware.default_max_speed_rad_s(),
                out.leg_mode == misa_hal::joint::JointMode::Idle,
                cfg.hardware.mit_gains(),
            );
            let verdict = shadow_gate.apply(&mut shadow, &obs, period);
            rec.push(misa_core::record::Frame {
                seq: i as u64,
                time,
                intent: cmd.clone(),
                observation: obs,
                command: shadow,
                verdict,
            });
        }

        if let Some(p) = publisher.as_mut() {
            // 実機を持たない机上再生なので planned だけ。受け側はゴーストを
            // 描かず、この 1 本でモデルを駆動する。
            let body = controller.body_view();
            p.maybe_publish(|seq| viz::Frames::planned(viz::frame(seq, t, &out.targets, &body)));
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
    }

    // **歩容の要求が安全ゲートの上限に収まっているか。**
    //
    // 収まっていないと、実機ではゲートが目標を鈍らせて歩容が崩れる。
    // 「シムでは歩けたのに実機で歩けない」がここから来る。
    {
        let mut over: Vec<String> = Vec::new();
        println!("\n歩容が要求する目標の変化率と、安全ゲートの上限 [rad/s]");
        for (leg, names) in JOINT_NAMES.iter().enumerate() {
            let mut line = String::new();
            for (k, jn) in names.iter().enumerate() {
                let i = leg * 3 + k;
                let cap = limits
                    .axes
                    .get(i)
                    .map(|a| a.max_target_rate_rad_s)
                    .unwrap_or(0.0);
                let mark = if cap > 0.0 && peak_rate[i] > cap {
                    over.push(format!("{jn} は {:.2} 要求、上限 {cap:.2}", peak_rate[i]));
                    "✗"
                } else {
                    " "
                };
                line.push_str(&format!(
                    "{:<6}{:5.2}/{}{}  ",
                    jn.get(3..jn.len() - 6).unwrap_or(jn),
                    peak_rate[i],
                    if cap > 0.0 {
                        format!("{cap:.2}")
                    } else {
                        "無制限".into()
                    },
                    mark
                ));
            }
            println!("  {}  {line}", ["FL", "FR", "RL", "RR"][leg]);
        }
        if !over.is_empty() {
            println!(
                "\n**歩容が安全ゲートの上限を超えています（{} 軸）。** 実機では\
                 ゲートが目標を鈍らせて歩容が崩れます:",
                over.len()
            );
            for o in over.iter().take(6) {
                println!("  {o}");
            }
            println!(
                "  上限は hardware.max_target_rate_rad_s と、モデルが宣言する\
                 定格速度の厳しいほう。歩容側（swing_height_m / *_cycle_s）を\
                 緩めるか、上限を見直してください"
            );
        }
    }

    if violations.is_empty() {
        println!(
            "\n可動域: すべて範囲内（{}/{} 軸に可動域の宣言あり）",
            bounded_axes(&limits),
            limits.axes.len()
        );
        Ok(())
    } else {
        println!("\n可動域を超えた指令が {} 件あります:", violations.len());
        // 全部出すと埋もれるので先頭だけ。件数は上に出してある。
        for v in violations.iter().take(20) {
            println!("  {v}");
        }
        Err("可動域を超える指令が出ています。gait の stance_height_m / \
             swing_height_m と config の可動域を見直してください"
            .into())
    }
}


fn open_viz(cfg: &VizConfig) -> Result<Option<viz::Publisher>, String> {
    if !cfg.enabled {
        return Ok(None);
    }
    viz::Publisher::new(cfg).map(Some)
}

fn header() -> String {
    let mut s = String::new();
    for names in JOINT_NAMES.iter() {
        s.push_str(&format!("{:<21}", &names[0][..2]));
    }
    s
}

/// 関節ベクトルを軸表の並びへ。脚 12 + 腕 1。
fn obs_axes(q: &JointVec) -> Vec<f64> {
    let mut v = Vec::with_capacity(13);
    for leg in q.legs.iter() {
        v.extend_from_slice(leg);
    }
    v.push(q.arm);
    v
}

fn row(q: &JointVec) -> String {
    let mut s = String::new();
    for leg in q.legs.iter() {
        s.push_str(&format!("{:+.3} {:+.3} {:+.3}  ", leg[0], leg[1], leg[2]));
    }
    s
}

/// 可動域を超えた指令を拾う。
///
/// **見るのは `SafetyGate` に渡すのと同じ [`AxisLimits`]。** 実測値
/// （プロファイル）があればそれ、無ければモデルの `[joint.limit]`、
/// どちらも無ければ無制限。ここで別々に設定を読み直すと、ゲートが丸める
/// 範囲と検証する範囲がずれる。
pub(crate) fn check_limits(
    limits: &misa_core::SafetyConfig,
    layout: &crate::snapshot::AxisLayout,
    q: &JointVec,
    t: f64,
    out: &mut Vec<String>,
) {
    let mut check = |id: misa_core::AxisId, value: f64| {
        let Some(lim) = limits.axes.get(id.index()) else {
            return;
        };
        if value < lim.min_rad || value > lim.max_rad {
            let name = layout.table.name(id).unwrap_or("?");
            out.push(format!(
                "t={t:5.2}s {name} = {value:+.4} rad（範囲 {:+.3}..{:+.3}）",
                lim.min_rad, lim.max_rad
            ));
        }
    };
    for leg in 0..4 {
        for k in 0..3 {
            check(
                misa_core::AxisId::new((leg * 3 + k) as u16),
                q.legs[leg][k],
            );
        }
    }
    if let Some(id) = layout.head {
        check(id, q.arm);
    }
}

/// 可動域の宣言がある軸の数。「すべて範囲内」がどれだけの検証に基づくかを
/// 添えるため。**無制限の軸ばかりで「範囲内」と言うのは何も言っていない。**
fn bounded_axes(limits: &misa_core::SafetyConfig) -> usize {
    limits
        .axes
        .iter()
        .filter(|a| a.min_rad.is_finite() && a.max_rad.is_finite())
        .count()
}

fn level_imu() -> ImuSample {
    ImuSample {
        rpy_rad: [0.0; 3],
        gyro_rad_s: [0.0; 3],
        accel_m_s2: [0.0, 0.0, 9.80665],
        temperature_c: 25.0,
        stamp: std::time::Instant::now(),
    }
}
