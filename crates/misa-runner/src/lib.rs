//! 四脚ロボットの実機制御。**対応機は robots/ のプロファイルで決まる。**
//!
//! # ライブラリとして使う
//!
//! **機体固有の繋ぎ方は [`Backend`] として外から差す。** 別リポジトリの
//! クレートが `Plant` と `Pilot` を作って渡せば、`misa-run` と同じ
//! サブコマンドがそのまま使える。namiashi2 の STM ブリッジ（`low_command_msgs` /
//! `low_state_msgs` に束縛される）がその 1 例で、**この crate には入れない**
//! ——入れると、操縦だけしたい機体でも向こうの独自メッセージが要るため。
//!
//! ```ignore
//! fn main() -> std::process::ExitCode {
//!     misa_runner::main_with(&[&MyBackend])
//! }
//! ```
//!
//! ```text
//! misa-run ports                    CH348 のポートを UART 番号つきで一覧
//! misa-run config [--out PATH]      既定設定を TOML で書き出す
//! misa-run check                    設定とモデルを検証（実機に触れない）
//! misa-run dump [--gait ..] [--vx]  歩容を実機なしで再生し関節角を検証
//! misa-run calib <sub>              符号・ゼロ点・可動域を実機で確定する
//! misa-run imu | sbus | legs        実機の受信 / 状態だけを観測（動かさない）
//! misa-run run                      制御ループ（プロポ操縦）
//! ```
//!
//! 立ち上げの順番は上から下。`check` → `ports` → `imu` / `sbus` / `legs` が
//! 通ってから `run` に行くと、どこで詰まったのかが常に 1 段で分かる。

pub mod calib;
pub mod chicken;
pub mod config;
pub mod controller;
pub mod diag;
pub mod dump;
pub mod jointvec;
pub mod pilot;
pub mod pilot_keys;
#[cfg(feature = "ros2")]
pub mod pilot_ros2;
pub mod plant;
pub mod pose;
pub mod record;
pub mod robot;
#[cfg(feature = "sim")]
pub mod sim;
pub mod runner;
pub mod snapshot;
pub mod teleop;
pub mod viz;

pub use config::AppConfig;

// **Backend を書く側が依存を 1 つで済むように再エクスポートする。**
// 別リポジトリのクレートがこれらを自前で引くと、git の版が食い違ったときに
// 「同じ名前の別の型」になって、噛み合わない理由が分かりにくい。
pub use misa_core;
pub use misa_hal;


/// **機体との繋ぎ方を外から足す口。**
///
/// `Plant` と `Pilot` は misa-core のトレイトで、実機・シム・再生を同じ穴に
/// 通すためのもの。ここはその「どれを使うか」をプロファイルの内容から決める
/// 部分で、**機体固有のメッセージ型に束縛されるのはこの実装の中だけ**になる。
///
/// namiashi2 の STM ブリッジのように相手の独自メッセージ（`low_command_msgs` /
/// `low_state_msgs`）に縛られる繋ぎ方は、**別リポジトリのクレートに置いて
/// ここから差す。** そうしないと、cmd_vel で操縦したいだけの機体でも
/// 向こうの ws が要るようになる。
pub trait Backend {
    /// このプロファイルを扱えるなら `Plant` と `Pilot` を作る。
    ///
    /// **扱えないなら `None`。** 呼び出し側は次の Backend を当たる。
    /// 扱えるが失敗した（繋がらない等）ときは `Some(Err(..))`。
    fn connect(
        &self,
        cfg: &AppConfig,
    ) -> Option<Result<(Box<dyn misa_core::Plant>, Box<dyn misa_core::Pilot>), String>>;

    /// `bridge` 相当の往復確認。**指令は脱力のまま**であること。
    /// 持たないなら `None`。
    fn diagnose(&self, _cfg: &AppConfig, _secs: Option<f64>) -> Option<Result<(), String>> {
        None
    }
}

/// `misa-run` の中身。**Backend を差して呼ぶ。**
///
/// 戻り値は終了コード。**75 は「起動条件が整っていないだけ」**で、
/// systemd はこれだけを再起動の対象にする（`misa-run.service`）。
/// 制御ループ中のクラッシュ（1）で自動再起動すると脚が再び動き出す。
pub fn main_with(backends: &[&dyn Backend]) -> std::process::ExitCode {
    // 既定は info。うるさければ `RUST_LOG=warn`、追い込むときは `debug`。
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format_timestamp_millis()
        .init();

    let cli = Cli::parse(std::env::args().skip(1));
    if cli.wants_help() {
        print_help();
        return std::process::ExitCode::SUCCESS;
    }

    if let Err(e) = dispatch(&cli, backends) {
        // **起動条件が整っていないだけの失敗は 75 で返す。**
        // systemd はこれだけを再起動の対象にする（`misa-run.service`）。
        // 制御ループ中のクラッシュ（1）で自動再起動すると脚が再び動き出す。
        match e.strip_prefix(runner::RETRYABLE) {
            Some(msg) => {
                eprintln!("待機: {msg}");
                return std::process::ExitCode::from(75);
            }
            None => {
                eprintln!("エラー: {e}");
                return std::process::ExitCode::FAILURE;
            }
        }
    }
    std::process::ExitCode::SUCCESS
}

fn dispatch(cli: &Cli, backends: &[&dyn Backend]) -> Result<(), String> {
    // **知らないフラグは何もしないうちに弾く。** 綴り違いを黙って無視すると
    // 「指定したつもりの設定が効かないまま実機が動く」になる。
    cli.validate_flags()?;
    let command = cli.command();
    match command {
        // 設定を読まずに済むものを先に。
        "ports" => return diag::ports(),
        "config" => return write_config(cli),
        "replay" => return replay(cli),
        _ => {}
    }

    let cfg = load_config(cli)?;
    match command {
        "check" => diag::check(&cfg),
        // **`bridge` を持つのは Backend。** 機体固有の往復確認なので、
        // どの Backend も名乗り出なければ「この実行ファイルには無い」。
        "bridge" => backends
            .iter()
            .find_map(|b| b.diagnose(&cfg, secs_or_forever(cli, 10.0)))
            .unwrap_or_else(|| {
                Err("この実行ファイルは bridge を持ちません。\
                     機体側のリポジトリの実行ファイルを使ってください"
                    .into())
            }),
        "dump" => dump::run(&cfg, cli),
        #[cfg(feature = "sim")]
        "sim" => sim::run(&cfg, cli),
        #[cfg(not(feature = "sim"))]
        "sim" => Err("このビルドには sim が入っていません（--features sim で有効化）".into()),
        "calib" => calib::run(&cfg, cli),
        "imu" => diag::imu(&cfg, secs_or_forever(cli, 10.0)),
        "sbus" => diag::sbus(&cfg, secs_or_forever(cli, 10.0), cli.flag("plain")),
        "legs" => diag::legs(&cfg, secs_or_forever(cli, 10.0), &viz_config(cli)),
        "run" => {
            let robot = robot::load_from_config(&cfg)?;
            let opts = runner::RunOptions {
                allow_no_sbus: cli.flag("allow-no-sbus"),
                skip_zero: cli.flag("skip-zero"),
                status_interval_s: cli.f64("status").unwrap_or(1.0),
                viz: viz_config(cli),
                record: cli.str("record").map(|s| s.to_string()),
            };
            runner::run(cfg, robot, opts, backends)
        }
        other => Err(format!(
            "未知のコマンド {other:?}。`misa-run --help` を見てください"
        )),
    }
}

/// 観測コマンドの継続時間。無限なら `None`（Ctrl-C まで）。
///
/// 立ち上げ中は「手で動かしながら眺める」ので秒数を決め打ちできない。
/// **`--secs 0`（以下）と `--forever` の両方**で無限になる。手が覚えている方を
/// 打てばよく、どちらでも同じ。
///
/// 0 を「無限」に割り当てるのは、U_BOOT_TIMEOUT=0 が「即起動」ではなく
/// 「無限に待つ」で紛らわしいのと同じ形ではある（`doc/boot_config.md` の
/// U-Boot の節）。ただしこちらは**起動直後に「Ctrl-C まで」と画面に出る**ので、
/// 意図と違えばその場で分かる。U-Boot 側は無言でハングするのが問題だった。
pub fn secs_or_forever(cli: &Cli, default: f64) -> Option<f64> {
    if cli.flag("forever") {
        return None;
    }
    let secs = cli.f64("secs").unwrap_or(default);
    (secs > 0.0).then_some(secs)
}

/// `--viz` 系のオプションを読む。
pub fn viz_config(cli: &Cli) -> viz::VizConfig {
    let d = viz::VizConfig::default();
    viz::VizConfig {
        enabled: cli.flag("viz"),
        key_planned: cli.str("viz-key").unwrap_or(&d.key_planned).to_string(),
        key_measured: cli
            .str("viz-key-measured")
            .unwrap_or(&d.key_measured)
            .to_string(),
        rate_hz: cli.f64("viz-rate").unwrap_or(d.rate_hz),
        endpoint: cli.str("viz-endpoint").map(|s| s.to_string()),
    }
}

/// ロボットのプロファイルを読む。
///
/// 正しい綴りは `--robot`。`--config` は改名前からの綴りで、systemd の
/// unit や手元のスクリプトが使っているので当面は受け付ける。両方あれば
/// `--robot` を採る。
fn load_config(cli: &Cli) -> Result<AppConfig, String> {
    let path = cli.str("robot").or_else(|| cli.str("config"));
    match path {
        Some(path) => {
            let cfg = AppConfig::load(path)?;
            log::info!("ロボット {} のプロファイル {path} を読みました", cfg.name);
            Ok(cfg)
        }
        None => {
            log::info!("--robot の指定がないので組み込みの既定値を使います");
            let cfg = AppConfig::default();
            cfg.validate()?;
            Ok(cfg)
        }
    }
}

fn write_config(cli: &Cli) -> Result<(), String> {
    let text = AppConfig::default().to_toml()?;
    match cli.str("out") {
        Some(path) => {
            std::fs::write(path, &text).map_err(|e| format!("{path} に書けません: {e}"))?;
            println!("{path} に既定設定を書き出しました");
        }
        None => print!("{text}"),
    }
    Ok(())
}

/// 記録を読む。引数 1 つで要約、2 つで指令の差分。
///
/// **差分に許容差は無い。** 見たいのは「値が近いか」ではなく「同じ計算を
/// したか」なので、1 bit でも違えば食い違いとして出す。
fn replay(cli: &Cli) -> Result<(), String> {
    let paths: Vec<&str> = cli.positionals().iter().skip(1).map(|s| s.as_str()).collect();
    match paths.as_slice() {
        [one] => {
            let (h, frames) = record::read(one)?;
            let span = frames
                .last()
                .map(|f| f.time.as_secs_f64() - frames[0].time.as_secs_f64())
                .unwrap_or(0.0);
            println!("ロボット      {}", h.robot);
            println!("軸            {} 本", h.axes.len());
            println!("制御周期      {:.0} Hz", h.rate_hz);
            println!("周期数        {}", frames.len());
            println!("記録時間      {span:.2} s");

            let touched = frames.iter().filter(|f| !f.verdict.is_clean()).count();
            println!("丸めた周期    {touched}");
            let clamped = frames.iter().filter(|f| !f.verdict.clamped.is_empty()).count();
            let limited = frames
                .iter()
                .filter(|f| !f.verdict.rate_limited.is_empty())
                .count();
            let held = frames
                .iter()
                .filter(|f| f.verdict.held_for_stale_observation)
                .count();
            let faulted = frames.iter().filter(|f| !f.verdict.faulted.is_empty()).count();
            // 傾きは丸めていないので「丸めた周期」には入らない。別に数える。
            let tilted = frames.iter().filter(|f| f.verdict.tilt_rad.is_some()).count();
            println!("  可動域      {clamped}");
            println!("  スルーレート {limited}");
            println!("  観測が古い  {held}");
            println!("  異常ビット  {faulted}");
            if tilted > 0 {
                let worst = frames
                    .iter()
                    .filter_map(|f| f.verdict.tilt_rad)
                    .fold(0.0f64, f64::max);
                println!("傾き超過      {tilted} 周期（最大 {:.0}°）", worst.to_degrees());
            }
            Ok(())
        }
        [a, b] => {
            let (ha, fa) = record::read(a)?;
            let (hb, fb) = record::read(b)?;
            if ha.axes != hb.axes {
                return Err("軸の並びが違う記録どうしは比べられません".into());
            }
            let limit = cli.usize("limit").unwrap_or(20);
            let d = misa_core::diff_commands(&fa, &fb, limit);
            if d.is_empty() {
                println!(
                    "食い違いなし（{} 周期を比較。指令は 1 bit も変わっていません）",
                    fa.len().min(fb.len())
                );
                return Ok(());
            }
            println!("食い違い {} 件（先頭 {limit} 件まで）", d.len());
            for x in &d {
                let name = ha
                    .axes
                    .get(x.axis.index())
                    .map(|s| s.as_str())
                    .unwrap_or("?");
                println!(
                    "  seq {:>7}  {name:<16} {:<16} {} -> {}",
                    x.seq, x.field, x.left, x.right
                );
            }
            Err("指令が食い違っています".into())
        }
        _ => Err("使い方: misa-run replay LOG [LOG2] [--limit N]".into()),
    }
}

fn print_help() {
    println!(
        r#"misa-run — 四脚ロボットの実機制御アプリ

使い方:
  misa-run <コマンド> [オプション]

コマンド:
  ports                     CH348 のポートを物理 UART 番号つきで一覧（何も開かない）
  config [--out PATH]       既定設定を TOML で出力
  check                     設定とモデルを検証（実機に触れない）
  dump                      歩容を実機なしで再生し、関節角と可動域を検証
                            [--cycle S] [--swing M] [--step-length M] [--duty D]
                            **`sim` で詰めた歩容パラメータの要求レートを
                            ここで確かめる**（定格とゲートに収まるか）
  imu    [--secs S]         IMU の値を表示（モータには触れない）
  sbus   [--secs S] [--plain]
                            プロポ入力と解釈結果を表示（同上）
                            既定は再描画表示。--plain で 1 行 / 更新の逐次出力
  legs   [--secs S] [--viz] 脚バスと IMU の状態を表示（**指令は送らない**）
                            --viz で**実測角と IMU 姿勢**を measured キーへ配信

  imu / sbus / legs は --secs 0（以下）または --forever で Ctrl-C まで回り続ける。
  calib  <sub>              符号・ゼロ点・可動域を実機で確定して設定に書き戻す
  run    [--record PATH]    制御ループ（プロポ操縦）
                            --record で毎周期を記録する（別スレッドで書く）
  sim    [--gait G] [--vx V] MuJoCo で動力学込みに回す（--features sim のビルド）
         [--secs S] [--kp K] [--kv K] [--base-height M] [--record PATH]
         [--pilot keys]            **キーボードで操縦する。** --viz と併せて
                                   articara に出せば、見ながら動かせる。
                                   押しっぱなしは端末から取れないので、押す
                                   たびに 1 段ずつ足す（space で速度 0）
         [--cam-fixed]             カメラを固定する（既定は胴体を追う）。
                                   **地面が無地なので、追従だと歩いても
                                   止まって見える。** 進んだことを見せる用
         [--friction MU]           接地摩擦（既定 0.7）。**足が滑ると歩容は
                                   成立しない。**「接地中の足の滑り」を見る
         [--timestep S]            物理の刻み [s]（既定 MuJoCo の 2 ms）
                                   **重い機体では下げないと立てない。** PD が
                                   明示的なので kv < 2·I/dt でしか安定しない
         [--cycle S] [--swing M]   **歩容パラメータを上書きする。** 与えな
         [--step-length M]         かった項目はプロファイルのまま。周期は
         [--duty D]                揺れに、歩幅は速度の出方に効く
         [--tune-at S]             上の上書きを S 秒から入れる（**歩きながら
                                   替える**。位相は保たれるので跳ねない）
  bridge [--secs S]         ROS 2 ブリッジとの往復を確認（**指令は脱力のまま**）
  replay LOG [LOG2]         記録の要約。2 つ渡すと指令を差分する
                            [--limit N] 差分の表示件数（既定 20）

共通オプション:
  --robot PATH              ロボットのプロファイル TOML
                            （省略時は組み込みの既定値。--config は旧綴り）

dump のオプション:
  --gait crawl|walk|trot    歩容（既定 crawl）
  --vx V --vy V --wz V      速度指令（既定 vx=0.05）
  --secs S                  再生時間（既定 4）
  --every N                 N 周期ごとに 1 行出す（既定 20）
  --realtime                実時間で流す（--viz で articara に見せるとき用）
  --tilt-roll R --tilt-pitch P --tilt-yaw Y
                            胴体を傾けた姿勢で可動域を確かめる [rad]
                            （実機の CH8 + CH1/CH3 と同じ経路を通る）

calib のサブコマンド:
  scan  [--leg FL] [--max-id N]         応答するモータ id を数える（指令なし）
  move  --leg FL --joint thigh          1 軸だけ小さく動かして sign を決める
        [--deg D] [--speed R] [--assume y|n] [--write PATH]
  range --leg FL --joint thigh          脱力させ、手で動かして可動域を測る
        [--secs S | --forever] [--margin RAD] [--write PATH]
        --secs 0（以下）/ --forever なら Ctrl-C で確定
        （打ち切っても集計と --write は走る）
  zero  [--pose NAME] [--write PATH]    全軸ゼロ出し + zero_pose_rad を記録
  clear-multiturn [--leg FL] [--joint thigh]
                                        マルチターンを 0 に戻す（電源 OFF/ON 相当）
  single-turn     [--leg FL]            単回転絶対角 0x94 を読む（**読むだけ**）
                                        電源 OFF/ON をまたいで一致する唯一の値
  clear-error     [--leg FL] [--dry-run]
                                        ドライバの異常フラグを消す 0x9B
                                        **原因が残っている間は消えない**
                                        --dry-run なら何も送らず状態だけ見る
  pid             [--leg FL] [--joint thigh]
                                        PID ゲインを読む（0x30 / 0xC0 自動）
        [--set-position-kp N] [--set-position-ki N] [--set-position-kd N]
        [--set-speed-kp N]    [--set-speed-ki N]    [--set-speed-kd N]
        [--set-current-kp N]  [--set-current-ki N]  [--set-current-kd N]
                                        指定した項だけ書く（0〜2000）
                                        **0xC1、RAM のみ。電源で元に戻る**
        [--set-torque-limit N]          トルク電流リミット 0x1E（0〜2000）
                                        押し返す力そのものを頭打ちにする
                                        **柔らかくするなら電流ループの Kd**
  restart         [--leg FL] [--joint thigh]
                                        ドライバを再起動 0x07（電源再投入と等価）
                                        **マルチターン原点がリセットされる**

  1 度に 1 軸しか投入せず、既定の振り幅は 5°・速度 0.3 rad/s。
  --write を付けたときだけ設定ファイルへ書き戻す。

ライブ可視化のオプション（dump / run / legs 共通）:
  --viz                     各周期の姿勢を Zenoh へ配信し articara に描かせる
                            run は指令+実測、legs は実測のみ、dump は指令のみ
  --viz-key KEY             指令ストリームのキー（既定 go2/gait/planned）
  --viz-key-measured KEY    実測ストリームのキー（既定 go2/gait/measured）
  --viz-rate HZ             配信レート（既定 100）
  --viz-endpoint EP         例 tcp/127.0.0.1:7447（マルチキャスト不可の環境）

run のオプション:
  --allow-no-sbus           受信機なしでも起動する（ベンチで脚を浮かせた時のみ）
  --skip-zero               起動時のゼロ出しを省略する
  --status S                状態表示の間隔 [s]（0 で表示しない、既定 1）

立ち上げの順番:
  misa-run check  →  ports  →  imu / sbus / legs
    →  calib scan  →  calib range/move（12 軸ぶん）  →  calib zero  →  run

articara で見る:
  1) misa-run dump --gait trot --vx 0.1 --secs 60 --realtime --viz \
       --viz-endpoint tcp/127.0.0.1:7447
  2) 別端末で articara を起動しモデル models/namiashi/namiashi.misa を開く
     （cd ../articara && cargo run --release --features viz）
  3) Live gait feed パネルで同じキー / エンドポイントを入れて Start
"#
    );
}

/// 綴り違いらしき既知フラグを探す。編集距離 1〜2 か、前方一致。
fn nearest_flag(given: &str) -> Option<&'static str> {
    BOOL_FLAGS
        .iter()
        .chain(VALUE_FLAGS.iter())
        .filter(|known| {
            known.starts_with(given)
                || given.starts_with(**known)
                || edit_distance(given, known) <= 2
        })
        .min_by_key(|known| edit_distance(given, known))
        .copied()
}

/// レーベンシュタイン距離。フラグ名しか比べないので素朴な実装で足りる。
fn edit_distance(a: &str, b: &str) -> usize {
    let (a, b): (Vec<char>, Vec<char>) = (a.chars().collect(), b.chars().collect());
    let mut prev: Vec<usize> = (0..=b.len()).collect();
    let mut cur = vec![0usize; b.len() + 1];
    for (i, ca) in a.iter().enumerate() {
        cur[0] = i + 1;
        for (j, cb) in b.iter().enumerate() {
            let cost = usize::from(ca != cb);
            cur[j + 1] = (prev[j] + cost).min(prev[j + 1] + 1).min(cur[j] + 1);
        }
        std::mem::swap(&mut prev, &mut cur);
    }
    prev[b.len()]
}

/// 素朴なコマンドライン解析。`--key value` と `--key=value`、
/// [`BOOL_FLAGS`] に載っているものは値を取らない存在フラグ。
pub struct Cli {
    pub positionals: Vec<String>,
    flags: std::collections::HashMap<String, String>,
}

/// 値を取るフラグ。**[`BOOL_FLAGS`] と合わせて、これが全部**。
///
/// 綴りを間違えたフラグを黙って受け取ると、**指定したつもりの設定が効かない
/// まま実機が動く**。`--sec 5`（`--secs` の綴り違い）を受け取って既定値で
/// 走り続ける、といった事故になるので、知らないフラグは起動時に弾く。
const VALUE_FLAGS: &[&str] = &[
    "robot",
    "record",
    "limit",
    "kp",
    "kv",
    "base-height",
    "timestep",
    "friction",
    // 歩容パラメータの実行中の上書き（`sim`）。
    "cycle",
    "swing",
    "step-length",
    "duty",
    "tune-at",
    "pilot",
    "video",
    "fps",
    "width",
    "height",
    "cam-az",
    "cam-el",
    "cam-dist",
    "cam-z",
    "cam-x",
    "cam-y",
    "config",
    "secs",
    "gait",
    "vx",
    "vy",
    "wz",
    "every",
    "out",
    "leg",
    "joint",
    "deg",
    "speed",
    "assume",
    "write",
    "max-id",
    "margin",
    "status",
    "viz-key",
    "viz-key-measured",
    "viz-rate",
    "viz-endpoint",
    "set-position-kp",
    "set-position-ki",
    "set-position-kd",
    "set-speed-kp",
    "set-speed-ki",
    "set-speed-kd",
    "set-current-kp",
    "set-current-ki",
    "set-current-kd",
    "set-torque-limit",
    "tilt-roll",
    "tilt-pitch",
    "tilt-yaw",
];

/// 値を取らないフラグ。ここに無いものは次のトークンを値として食う。
const BOOL_FLAGS: &[&str] = &[
    "help",
    "dry-run",
    "allow-no-sbus",
    "skip-zero",
    "viz",
    "realtime",
    "plain",
    "forever",
    "chicken",
    "cam-fixed",
];

impl Cli {
    pub fn parse(args: impl Iterator<Item = String>) -> Self {
        let mut positionals = Vec::new();
        let mut flags = std::collections::HashMap::new();
        let mut it = args.peekable();
        while let Some(arg) = it.next() {
            let Some(name) = arg.strip_prefix("--") else {
                if arg == "-h" {
                    flags.insert("help".into(), "true".into());
                } else {
                    positionals.push(arg);
                }
                continue;
            };
            if let Some((k, v)) = name.split_once('=') {
                flags.insert(k.to_string(), v.to_string());
            } else if BOOL_FLAGS.contains(&name) {
                flags.insert(name.to_string(), "true".into());
            } else {
                flags.insert(name.to_string(), it.next().unwrap_or_default());
            }
        }
        Self { flags, positionals }
    }

    /// 知らないフラグが混ざっていないか。
    ///
    /// **綴り違いを黙って無視しない。** `--sec 5` を受け取って既定の
    /// 秒数で走り続ける、`--vis` で可視化が出ないまま悩む、といった
    /// 事故を起動時に止める。
    pub fn validate_flags(&self) -> Result<(), String> {
        let mut unknown: Vec<&str> = self
            .flags
            .keys()
            .map(|k| k.as_str())
            .filter(|k| !BOOL_FLAGS.contains(k) && !VALUE_FLAGS.contains(k))
            .collect();
        if unknown.is_empty() {
            return Ok(());
        }
        unknown.sort_unstable();
        let hints: Vec<String> = unknown
            .iter()
            .map(|k| match nearest_flag(k) {
                Some(near) => format!("--{k}（もしかして --{near}?）"),
                None => format!("--{k}"),
            })
            .collect();
        Err(format!(
            "知らないオプション: {}\n--help で一覧が出ます",
            hints.join(", ")
        ))
    }

    pub fn command(&self) -> &str {
        self.positionals
            .first()
            .map(|s| s.as_str())
            .unwrap_or("check")
    }

    pub fn wants_help(&self) -> bool {
        self.flags.contains_key("help") || self.positionals.iter().any(|p| p == "help")
    }

    pub fn positionals(&self) -> &[String] {
        &self.positionals
    }

    pub fn str(&self, key: &str) -> Option<&str> {
        self.flags
            .get(key)
            .map(|s| s.as_str())
            .filter(|s| !s.is_empty())
    }

    pub fn f64(&self, key: &str) -> Option<f64> {
        self.str(key).and_then(|s| s.parse().ok())
    }

    pub fn usize(&self, key: &str) -> Option<usize> {
        self.str(key).and_then(|s| s.parse().ok())
    }

    pub fn flag(&self, key: &str) -> bool {
        self.flags.get(key).map(|v| v != "false").unwrap_or(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cli(args: &[&str]) -> Cli {
        Cli::parse(args.iter().map(|s| s.to_string()))
    }

    #[test]
    fn the_default_command_is_the_harmless_one() {
        // 引数なしで実機が動き出さないこと。
        assert_eq!(cli(&[]).command(), "check");
    }

    #[test]
    fn both_flag_spellings_parse() {
        let c = cli(&["run", "--status", "2.5", "--config=/tmp/a.toml"]);
        assert_eq!(c.command(), "run");
        assert_eq!(c.f64("status"), Some(2.5));
        assert_eq!(c.str("config"), Some("/tmp/a.toml"));
    }

    #[test]
    fn a_bool_flag_does_not_eat_the_next_token() {
        let c = cli(&["run", "--allow-no-sbus", "--secs", "3"]);
        assert!(c.flag("allow-no-sbus"));
        assert_eq!(c.f64("secs"), Some(3.0));
    }

    #[test]
    fn help_is_recognised_in_every_spelling() {
        assert!(cli(&["--help"]).wants_help());
        assert!(cli(&["-h"]).wants_help());
        assert!(cli(&["help"]).wants_help());
        assert!(!cli(&["run"]).wants_help());
    }
}
