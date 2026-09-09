//! `misa-run stance` — 基準姿勢（立ち姿勢）を実機の観測から作る。
//!
//! **手で 12 個の数を写す作業を残さない**（doc/reference_stance.md R3）。
//! 記録（`--record` の `.rec`）か、実機の観測そのものから関節角を採り、モデルの
//! 順運動学で足先位置にして、プロファイルの `gait.stance_feet_body` に書く。
//!
//! ```text
//! misa-run stance capture --robot robots/x.toml --from-record run.rec [--at 5.0]
//! misa-run stance capture --robot robots/x.toml --secs 3          # 実機を読む（指令は脱力）
//!     [--symmetrize] [--write robots/x.toml]
//! ```
//!
//! 左右の平均（`--symmetrize`）は**明示したときだけ**取り、取ったことは
//! `gait.stance_symmetrize = true` としてプロファイルに残る（R4）。測った前後の
//! 傾きは `gait.stance_pitch_rad` に「意図した傾き」として書く（§4）ので、
//! 採用した姿勢そのものでは警告が出ない。出どころは `gait.stance_note` に残す。

use std::time::{Duration, Instant};

use nalgebra::Vector3;

use crate::config::{AppConfig, StanceFeet};
use crate::jointvec::JointVec;
use crate::robot::Robot;
use crate::Cli;

pub fn run(cfg: &AppConfig, cli: &Cli, backends: &[&dyn crate::Backend]) -> Result<(), String> {
    match cli.positionals.get(1).map(|s| s.as_str()) {
        Some("capture") => capture(cfg, cli, backends),
        Some(other) => Err(format!("未知の stance サブコマンド {other:?}（capture）")),
        None => Err("stance のサブコマンドを指定してください（capture）".into()),
    }
}

fn capture(cfg: &AppConfig, cli: &Cli, backends: &[&dyn crate::Backend]) -> Result<(), String> {
    let robot = crate::robot::load_from_config(cfg)?;
    let (q, origin) = match cli.str("from-record") {
        Some(path) => (posture_from_record(path, cli.f64("at"))?, format!("記録 {path}")),
        None => {
            let secs = cli.f64("secs").unwrap_or(3.0).clamp(0.5, 60.0);
            (posture_from_plant(cfg, backends, secs)?, format!("実機の観測 {secs:.1} s の平均"))
        }
    };
    println!("関節角（モデル座標 [rad]）:");
    for (name, v) in q.iter_named() {
        println!("  {name:16} {v:+.4}");
    }
    let feet_raw = robot.feet_from_posture(&q);
    let symmetrize = cli.flag("symmetrize");
    let feet = if symmetrize { Robot::symmetrize_feet(feet_raw) } else { feet_raw };

    // 採用したときの姿を、同じ検査で見せる。
    let mut tuning = cfg.gait.clone();
    tuning.stance_pose = None;
    tuning.stance_feet_body = Some(to_feet(feet));
    tuning.stance_symmetrize = false; // もう平均してあるので二重に取らない
    let rep0 = robot.stance_report(&tuning);
    tuning.stance_pitch_rad = rep0.pitch_rad;
    let rep = robot.stance_report(&tuning);
    println!();
    if symmetrize {
        println!(
            "左右を平均しました（平均前の差: 前 x {:.0} mm / z {:.0} mm、後 x {:.0} mm / z {:.0} mm）",
            (feet_raw[0].x - feet_raw[1].x).abs() * 1e3,
            (feet_raw[0].z - feet_raw[1].z).abs() * 1e3,
            (feet_raw[2].x - feet_raw[3].x).abs() * 1e3,
            (feet_raw[2].z - feet_raw[3].z).abs() * 1e3,
        );
    }
    println!("{}", rep.describe());
    let current = robot.stance_report(&cfg.gait);
    println!(
        "いまのプロファイルの基準姿勢: 高さ {:.3} m / スパン {:.3} m / 幅 {:.3} m / 中心 x {:+.3} m / 傾き {:+.2}°",
        current.height_m, current.span_m, current.width_m, current.center_x_m, current.pitch_rad.to_degrees()
    );
    if !rep.errors.is_empty() {
        return Err("この姿勢は基準姿勢として受け入れられません（上のエラー）。書き出しません".into());
    }

    match cli.str("write") {
        Some(path) => {
            let mut out = cfg.clone();
            out.gait.stance_pose = None;
            out.gait.stance_feet_body = Some(to_feet(feet));
            out.gait.stance_symmetrize = symmetrize;
            out.gait.stance_pitch_rad = (rep.pitch_rad * 1e4).round() / 1e4;
            out.gait.stance_note = Some(format!(
                "stance capture {} から{}（{}）",
                origin,
                if symmetrize { "、左右を平均" } else { "" },
                chrono_free_date()
            ));
            out.validate()?;
            let text = out.to_toml()?;
            std::fs::write(path, text).map_err(|e| format!("{path} に書けません: {e}"))?;
            println!("{path} に gait.stance_feet_body / stance_symmetrize / stance_pitch_rad / stance_note を書きました（コメントは保たれません）");
        }
        None => println!("（--write PATH を付けるとプロファイルに書き戻します）"),
    }
    Ok(())
}

fn to_feet(f: [Vector3<f64>; 4]) -> StanceFeet {
    let r = |v: Vector3<f64>| [(v.x * 1e4).round() / 1e4, (v.y * 1e4).round() / 1e4, (v.z * 1e4).round() / 1e4];
    StanceFeet { fl: r(f[0]), fr: r(f[1]), rl: r(f[2]), rr: r(f[3]) }
}

/// 依存を増やさずに日付を残す（秒精度は要らない）。
fn chrono_free_date() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    // 1970-01-01 からの日数 → 年月日（グレゴリオ暦、UTC）。
    let days = (secs / 86_400) as i64;
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    format!("{y:04}-{m:02}-{d:02} UTC")
}

/// 記録から 1 フレームの関節角。`at` 秒以降の最初のフレーム、無ければ最後。
fn posture_from_record(path: &str, at: Option<f64>) -> Result<JointVec, String> {
    let (_header, frames) = crate::record::read(path)?;
    if frames.is_empty() {
        return Err(format!("{path} にフレームがありません"));
    }
    let t0 = frames[0].time.as_secs_f64();
    let frame = match at {
        Some(at) => frames
            .iter()
            .find(|f| f.time.as_secs_f64() - t0 >= at)
            .unwrap_or_else(|| frames.last().unwrap()),
        None => frames.last().unwrap(),
    };
    println!(
        "記録 {path} の t = {:.2} s（{} フレーム中）の観測を使います",
        frame.time.as_secs_f64() - t0,
        frames.len()
    );
    Ok(jointvec_from(&frame.observation))
}

/// 実機（または橋の向こう）を `secs` 秒読んで平均する。**指令は脱力のまま。**
fn posture_from_plant(cfg: &AppConfig, backends: &[&dyn crate::Backend], secs: f64) -> Result<JointVec, String> {
    let mut plant: Box<dyn misa_core::Plant> = match &cfg.hardware {
        misa_hal::config::HardwareConfig::Serial(_) => {
            let map = misa_hal::ch348::PortMap::discover().map_err(|e| e.to_string())?;
            Box::new(crate::plant::SerialPlant::connect_with(cfg, &map)?)
        }
        misa_hal::config::HardwareConfig::Ros2(_) => match backends.iter().find_map(|b| b.connect(cfg)) {
            Some(r) => r?.0,
            None => {
                return Err("この実行ファイルは kind = \"ros2\" の機体を繋げません。機体側のリポジトリの実行ファイルを使ってください".into())
            }
        },
    };
    let layout = crate::snapshot::axis_layout(cfg)?;
    let idle = misa_core::Command::idle(layout.table.len());
    let mut obs = misa_core::Observation::default();
    let period = Duration::from_secs_f64(1.0 / cfg.control.rate_hz);
    let end = Instant::now() + Duration::from_secs_f64(secs);
    let mut sum = JointVec::zeros();
    let mut n = 0usize;
    while Instant::now() < end {
        plant.exchange(&idle, &mut obs)?;
        let all_ok = (0..12).all(|i| obs.get(misa_core::AxisId::new(i as u16)).is_some_and(|a| a.health.valid));
        if all_ok {
            let q = jointvec_from(&obs);
            for leg in 0..4 {
                for k in 0..3 {
                    sum.legs[leg][k] += q.legs[leg][k];
                }
            }
            sum.arm += q.arm;
            n += 1;
        }
        std::thread::sleep(period);
    }
    let _ = plant.disarm();
    if n == 0 {
        return Err("脚 12 軸の観測が 1 度も揃いませんでした（モータ電源・バス・ブリッジを確認）".into());
    }
    let mut q = JointVec::zeros();
    for leg in 0..4 {
        for k in 0..3 {
            q.legs[leg][k] = sum.legs[leg][k] / n as f64;
        }
    }
    q.arm = sum.arm / n as f64;
    println!("{n} フレームを平均しました");
    Ok(q)
}

fn jointvec_from(obs: &misa_core::Observation) -> JointVec {
    let mut q = JointVec::zeros();
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
