//! 実機の制御ループ。
//!
//! 周期の考え方は [`misa_hal::legs`] のとおり: 脚バス 4 本はそれぞれ自由
//! 走行していて、このループは共有スロットに目標を書き最新値を読むだけ。
//! したがってここで守るべきは「一定周期で回ること」だけで、バスの応答を
//! 待つ必要はない。

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use misa_hal::arm::ArmServo;
use misa_hal::ch348::PortMap;
use misa_hal::imu::ImuReader;
use misa_hal::joint::{JointMode, LegSlot, LEG_JOINT_KINDS};
use misa_hal::legs::{BusRequest, LegArray};

use crate::config::AppConfig;
use crate::controller::{Controller, State};
use crate::jointvec::JointVec;
use crate::robot::Robot;

use crate::viz::{self, VizConfig};

/// 実機に繋いだ一式。
pub struct Hardware {
    pub legs: LegArray,
    pub imu: ImuReader,
    pub arm: Box<dyn ArmServo>,
}

impl Hardware {
    /// 全ポートを開く。失敗したらそこまでに開いたものは drop で閉じる。
    ///
    /// **UART 番号の探索は最初に 1 回だけ。** 探索はデバイスを `open` する
    /// ので、1 本開くたびに調べ直すと 2 本目以降が自分自身の `EBUSY` で
    /// 失敗する（実機で踏んだ）。
    /// **探索は呼び出し側で 1 回だけ。** 探索はデバイスを `open` するので、
    /// 1 本開くたびに調べ直すと 2 本目以降が自分自身の `EBUSY` で失敗する
    /// （実機で踏んだ）。受信機（`SbusPilot`）とも同じ地図を共有する。
    pub fn connect_with(cfg: &AppConfig, map: &PortMap) -> Result<Self, String> {
        let serial = cfg.hardware.serial().map_err(|e| e.to_string())?;
        let legs = LegArray::connect_with(serial, map, &cfg.name).map_err(|e| e.to_string())?;
        for bus in legs.buses() {
            log::info!("脚 {} → {}", bus.leg().prefix(), bus.port());
        }
        let imu = ImuReader::connect_with(&serial.imu, map).map_err(|e| e.to_string())?;
        log::info!("IMU → {}", imu.port());
        let arm = misa_hal::arm::connect(&serial.arm).map_err(|e| e.to_string())?;
        Ok(Self {
            legs,
            imu,
            arm,
        })
    }

    /// 12 軸の実測角を関節ベクトルにまとめる。
    pub fn measured(&self, arm: f64) -> JointVec {
        let states = self.legs.states();
        let mut q = JointVec::zeros();
        for (leg, leg_states) in states.iter().enumerate() {
            for (k, s) in leg_states.iter().enumerate() {
                q.legs[leg][k] = s.position_rad;
            }
        }
        q.arm = arm;
        q
    }
}

/// 観測が何周期ぶん古くなったら「見えていない」と判断するか。
///
/// 制御周期の 5 倍。1〜2 周期の取りこぼしは RS485 では普通に起きるので、
/// そこで止めると使い物にならない。一方で 5 周期（200 Hz なら 25 ms）
/// 読めていないのは、バスが詰まっているか無応答のモータがいる。
const STALE_TICKS: f64 = 5.0;

/// 「まだ起動条件が整っていない」失敗に付ける前置き。
///
/// systemd に**再試行してよい失敗**を伝えるためのもの。`main` がこれを見て
/// 終了コード 75 を返し、ユニット側は 75 のときだけ再起動する。
///
/// # なぜ区別するのか
///
/// 起動条件（S.BUS の受信、CH5 が脱力位置）が整わないのは**正常な待ち**で、
/// プロポの電源を入れれば解消する。一方、制御ループ中のクラッシュを自動で
/// 再起動すると**脚が再び動き出す**。同じ異常終了でも、片方は再試行が正しく、
/// もう片方は人が見に行くべき。
///
/// 一律 `Restart=on-failure` にすると後者まで拾い、一律 `Restart=no` にすると
/// 本番で「プロポを後から入れたのに立ち上がらない」になる。
pub const RETRYABLE: &str = "[retryable] ";

/// マルチターン原点が伏せ姿勢を指しているかの検算。**警告するだけで止めない。**
///
/// # 何を見ているのか
///
/// このロボットの角度規約は `q_model = sign * q_motor + zero_pose_rad` で、
/// **モータ角 0 が伏せ姿勢**と決めてある。モータのマルチターンカウンタは
/// 電源投入時に 0 になるので、これは「伏せ姿勢でモータ電源を入れること」と
/// 同義。守られていれば、起動直後の実測モデル角は 12 軸とも
/// `zero_pose_rad` に一致するはずで、一致しなければ原点が別の姿勢に
/// 張られている。その場合 `zero_pose_rad` は 12 軸すべて無効で、
/// 関節角は全部ずれる。
///
/// # なぜ止めないのか
///
/// 本番会場では PC を繋げないので、止めても脱力したまま動かないだけで
/// 理由が分からない。閾値を外した場合に**会場で立てなくなる**リスクの方が
/// 大きいと判断した（2026-08-22 の運用判断）。ベンチで基準の正しさを
/// 確かめる道具として使う。
///
/// # 閾値
///
/// 0.35 rad (20°)。伏せと立脚では thigh/calf が 1.0〜1.4 rad 違うので、
/// 「別の姿勢で電源が入った」ケースとは明確に分かれる。一方、伏せ姿勢を
/// 手で作るときのばらつきはこれより十分小さい。
/// 電源投入後の初回起動でだけ、いまの姿勢をマルチターン原点にする。
///
/// 意図と危険は [`crate::config::ControlConfig::zero_multiturn_on_boot`] を見ること。
/// ここでは「一度きり」をどう保証しているかだけ書く。
fn zero_multiturn_once(cfg: &AppConfig, hw: &Hardware) -> Result<(), String> {
    // tmpfs で 1777。**再起動で必ず消える**のがこの仕組みの土台。
    // `/run` は root:root 755 でサービスユーザ（takara）が書けない。
    // `/tmp` はディスク上のこともあり、再起動で消える保証がない。
    // **ロボット名で分ける。** 脚バスの flock と同じ理由（同一ホストで
    // 2 台動かしたとき、一方の目印がもう一方の張り直しを抑止しないように）。
    let marker = format!("/dev/shm/misa-multiturn-zeroed-{}", cfg.name);
    let marker = marker.as_str();
    const SETTLE: Duration = Duration::from_millis(300);

    if !cfg.control.zero_multiturn_on_boot {
        return Ok(());
    }
    if std::path::Path::new(marker).exists() {
        log::info!(
            "マルチターン原点は今回の電源投入で既に張り直し済みです。\
             いまの原点をそのまま使います（目印 {marker}）"
        );
        return Ok(());
    }

    // **目印を先に作る。** 後だと、張り直しに失敗した回や、この直後に
    // 落ちた回で目印が残らず、次の再起動で**立脚中に張り直してしまう**。
    // 作れなかったら「一度きり」を保証できないので、モータには触らずに諦める。
    std::fs::File::create(marker)
        .map_err(|e| format!("{marker} を作れません: {e}（一度きりを保証できないので中止します）"))?;

    log::warn!(
        "**いまの姿勢をマルチターン原点にします。**（電源投入後の初回起動）\
         伏せ姿勢であることを前提にしています。違う姿勢なら、いますぐ Ctrl-C か\
         電源を切って、伏せ姿勢にしてからやり直してください"
    );
    hw.legs
        .request_all(BusRequest::ClearMultiTurn)
        .map_err(|e| format!("マルチターン原点の張り直しに失敗: {e}"))?;
    // 各バスがコマンドを送ってフレームを張り直すまで待つ。要求はキュー経由で
    // 非同期なので、待たずに wait_anchored を呼ぶと**張り直す前の**
    // anchored=true を見てしまう。
    std::thread::sleep(SETTLE);
    hw.legs
        .wait_anchored(Duration::from_secs(3))
        .map_err(|e| format!("{e}（マルチターンフレームの張り直しに失敗しました）"))?;
    hw.legs
        .wait_first_read(Duration::from_secs(2))
        .map_err(|e| format!("{e}（張り直し後の読み出しに失敗しました）"))?;
    log::info!("マルチターン原点を張り直しました。この姿勢が伏せ姿勢になります");
    Ok(())
}

fn verify_crouch_frame(cfg: &AppConfig, hw: &Hardware) {
    const TOL_RAD: f64 = 0.35;

    let measured = hw.measured(0.0);
    let mut worst = 0.0f64;
    let mut worst_at = String::new();
    let mut rows = Vec::new();
    for leg in LegSlot::ALL {
        let Some(bus) = cfg.hardware.serial().ok().and_then(|h| h.bus_for(leg)) else {
            return;
        };
        for k in 0..3 {
            let Some(motor) = bus.motors.get(k) else {
                return;
            };
            let q = measured.legs[leg.index()][k];
            let d = q - motor.zero_pose_rad;
            if d.abs() > worst {
                worst = d.abs();
                worst_at = format!("{} {}", leg.prefix(), LEG_JOINT_KINDS[k]);
            }
            rows.push(format!(
                "{} {}: 実測 {:+.3} / 伏せ {:+.3} / 差 {:+.3} rad",
                leg.prefix(),
                LEG_JOINT_KINDS[k],
                q,
                motor.zero_pose_rad,
                d
            ));
        }
    }

    if worst <= TOL_RAD {
        log::info!(
            "マルチターン原点は伏せ姿勢と整合しています（最大ずれ {:.3} rad @ {}）",
            worst,
            worst_at
        );
        return;
    }
    log::warn!(
        "**マルチターン原点が伏せ姿勢と合っていません**（最大ずれ {:.3} rad @ {}、許容 {:.2}）。\
         伏せ以外の姿勢でモータ電源が入った可能性があります。この場合 zero_pose_rad は\
         12 軸すべて無効で、関節角は全部ずれます。**伏せ姿勢に直してモータ電源を入れ直すか、\
         伏せ姿勢で `calib clear-multiturn` を実行してください。**",
        worst,
        worst_at,
        TOL_RAD
    );
    for row in rows {
        log::warn!("  {row}");
    }
}


/// 追従誤差と可動域逸脱の監視。**検出して言うだけで、何もしない。**
///
/// # 何のために要るのか
///
/// 2026-08-22 の過負荷では、脱調したモータが大電流を引いて電源が電流制限に
/// 落ち、12 軸中 9 軸が低電圧保護に入った。**低電圧保護は時間で自然に解除
/// されるが、待って直るのはフラグだけで、ロボットは既に倒れている。**
/// 落とさないことが目的で、その前兆が**指令と実測が開いていくこと**。
///
/// # なぜ「言うだけ」なのか
///
/// **指令を実測から離れすぎないよう頭打ちにするのが本命の対策**だが、
/// 歩行中の正常な追従誤差を知らないまま閾値を決めると普通の動作まで
/// クランプされる。まずここで実測する。
///
/// 逸脱側も同じで、逸脱したまま脱力させるのは危ない（hip はメカ端まで
/// 余裕がある一方でケーブルが先に限界を迎える。脱力すると外力に任せる
/// ことになり自力で戻れない）。かといって自動で戻す動作を挟むと暴走時に
/// 危険が増す。**決めきれていないので、まず測る。**
struct Watch {
    /// この集計区間での最悪の追従誤差 (rad) と、その軸。
    worst: f64,
    worst_at: Option<(LegSlot, usize)>,
    /// 逸脱している軸を一度だけ言うためのフラグ。毎秒 12 行は読まれない。
    warned_excursion: bool,
    /// 12 軸の可動域。毎周期 config を引き直さないため。
    limits: [[(f64, f64); 3]; 4],
}

impl Watch {
    /// **可動域の出どころは 2 つある。** 校正でこちらが採った実測値
    /// （シリアル構成）と、モデルの `[joint.limit]`。
    ///
    /// 実測値があればそちらを優先する。無ければモデル。**両方を見ないと、
    /// ブリッジ越しの機体では ±∞ のままになって逸脱警告が一度も出ない**
    /// （namiashi2 が実際にそうだった。2026-09-04 に気づいて直した）。
    fn new(cfg: &AppConfig, model_limits: &std::collections::BTreeMap<String, (f64, f64)>) -> Self {
        let mut limits = [[(f64::NEG_INFINITY, f64::INFINITY); 3]; 4];
        for (leg_i, slot) in LegSlot::ALL.iter().enumerate() {
            for k in 0..3 {
                if let Some(v) = model_limits.get(misa_hal::joint::JOINT_NAMES[leg_i][k]) {
                    limits[leg_i][k] = *v;
                }
            }
            let Some(bus) = cfg.hardware.serial().ok().and_then(|h| h.bus_for(*slot)) else {
                continue;
            };
            for (m, d) in bus.motors.iter().zip(limits[leg_i].iter_mut()) {
                *d = (m.min_rad, m.max_rad);
            }
        }
        Self {
            worst: 0.0,
            worst_at: None,
            warned_excursion: false,
            limits,
        }
    }

    /// 1 周期ぶん。**目標を送っている間だけ意味がある**ので脱力中は呼ばない。
    fn tick(&mut self, targets: &JointVec, measured: &JointVec) {
        let mut out = Vec::new();
        for (leg_i, slot) in LegSlot::ALL.iter().enumerate() {
            for k in 0..3 {
                let m = measured.legs[leg_i][k];
                let e = (targets.legs[leg_i][k] - m).abs();
                if e > self.worst {
                    self.worst = e;
                    self.worst_at = Some((*slot, k));
                }
                let (lo, hi) = self.limits[leg_i][k];
                if m < lo || m > hi {
                    out.push(format!(
                        "{} {} {:+.1}°（範囲 [{:+.0}, {:+.0}]）",
                        slot.prefix(),
                        misa_hal::joint::LEG_JOINT_KINDS[k],
                        m.to_degrees(),
                        lo.to_degrees(),
                        hi.to_degrees()
                    ));
                }
            }
        }
        if out.is_empty() {
            self.warned_excursion = false;
        } else if !self.warned_excursion {
            self.warned_excursion = true;
            // **脱力させない。** 逸脱したまま力を抜くと外力に任せることに
            // なり、自力で戻れない。operator が判断できるよう言うだけ。
            log::error!(
                "  可動域を出ています（指令は出し続けます）: {}",
                out.join(" / ")
            );
        }
    }

    /// 集計区間の表示用文字列。読んだら最悪値は消す。
    fn take(&mut self) -> String {
        let out = match self.worst_at {
            Some((leg, k)) => format!(
                "{:.3}rad({} {})",
                self.worst,
                leg.prefix(),
                misa_hal::joint::LEG_JOINT_KINDS[k]
            ),
            None => "-".to_string(),
        };
        self.worst = 0.0;
        self.worst_at = None;
        out
    }
}

/// `run` サブコマンドの起動オプション。
pub struct RunOptions {
    /// S.BUS を待たずに起動する（受信機なしのベンチ確認用）。
    ///
    /// 受信が無い間は操縦指令がフェイルセーフ（速度 0 / 起立要求）になるので、
    /// **これを付けると electrically 立ち上がってしまう**。ベンチで脚を浮かせて
    /// いるときにだけ使うこと。
    pub allow_no_sbus: bool,
    /// 起動時にゼロ出しを行わない（すでに出してある場合）。
    pub skip_zero: bool,
    /// 状態表示の間隔 (s)。0 で表示しない。
    pub status_interval_s: f64,
    /// ライブ可視化（articara へ Zenoh 配信）。
    pub viz: VizConfig,
    /// 毎周期を記録する先。`None` なら記録しない。
    ///
    /// 書き込みは別スレッドで、詰まったら**捨てて数える**。制御周期は
    /// 待たせない（[`crate::record`]）。
    pub record: Option<String>,
}

impl Default for RunOptions {
    fn default() -> Self {
        Self {
            allow_no_sbus: false,
            skip_zero: false,
            status_interval_s: 1.0,
            viz: VizConfig::default(),
            record: None,
        }
    }
}

/// 制御ループ本体。Ctrl-C か致命的エラーで戻る。
/// 実機だけの立ち上げ手順。
///
/// 受信機を待ち、CH5 が脱力位置か確かめ、マルチターンの原点を張り、
/// 伏せ姿勢と照合する。**どれもバスを直接握る構成にしか無い**ので、
/// ブリッジ越しの機体はここを通らない。
fn start_serial(
    cfg: &AppConfig,
    opts: &RunOptions,
    plant: crate::plant::SerialPlant,
    pilot: &crate::pilot::SbusPilot,
) -> Result<crate::plant::SerialPlant, String> {
    // 受信機を待つ。プロポが無い状態で起立させないための入口チェック。
    match pilot.wait_ready(Duration::from_secs(3)) {
        Ok(()) => log::info!("S.BUS 受信を確認しました"),
        Err(e) if opts.allow_no_sbus => {
            log::warn!("S.BUS が来ていません ({e})。--allow-no-sbus 指定のため続行します")
        }
        Err(e) => {
            return Err(format!(
                "{RETRYABLE}{e}。送信機の電源とポートを確認してください（ベンチで脚を浮かせて \
                 いるなら --allow-no-sbus）"
            ))
        }
    }
    match plant.hw().imu.wait_ready(Duration::from_secs(2)) {
        Ok(_) => log::info!("IMU を確認しました"),
        // IMU はチキンヘッドと姿勢フィードバックにしか使っていないので、
        // 無くても歩容そのものは回る。止めずに警告に留める。
        Err(e) => log::warn!("IMU が来ていません ({e})。水平・静止として扱います"),
    }

    // 位置の基準は**モータの電源 ON マルチターンフレーム**で、バススレッドが
    // 起動時に自動で確立する。ここで姿勢を作る必要はない。
    //
    // かつては起動のたびに rezero していたため、**そのときの姿勢が原点**に
    // なっていた。異常終了から再起動すると崩れた姿勢が原点になる、という
    // 危うさもあった。いまは電源を入れ直さない限り原点は動かない。
    if !opts.skip_zero {
        plant.hw().legs
            .wait_anchored(Duration::from_secs(3))
            .map_err(|e| format!("{e}（モータの電源とボーレートを確認してください）"))?;
        log::info!("マルチターンフレームを確立しました");
    }

    // **12 軸が一度でも読めるまで制御ループに入らない。**
    //
    // `wait_anchored` はフレーム確立で返るが、その時点では共有状態の
    // `JointState` がまだ既定値（`position_rad = 0.0`, `ok = false`）のことが
    // ある。書かれるのは次のトランザクション周回。
    //
    // 脱力からの遷移は**実測値を始点**に張る（`Controller::tick_relaxed`）。
    // 0 を掴むと、実際には −2.7 rad にある calf の目標がいきなり 0 になり、
    // **最初の起立で暴れる**。`--skip-zero` でも省略しない — 読めない 12 軸を
    // 相手に制御ループを回すこと自体が危ない。
    plant.hw().legs
        .wait_first_read(Duration::from_secs(2))
        .map_err(|e| format!("{e}（12 軸すべてが応答している必要があります）"))?;
    log::info!("12 軸の初回読み出しを確認しました");

    // **CH5 が「脱力」でなければ起動しない。**
    //
    // モードスイッチは毎周期そのまま指令になるので、起立や歩行の位置で
    // 起動すると**何の操作もなしにその場で立ち上がる**。立ち上げ手順は
    // 「CH5 が脱力位置で起動する」ことを最初の確認項目にしているが、
    // 人手のチェックリストだけに任せる話ではない。
    //
    // `--allow-no-sbus` のときは見ない。受信が無い＝フェイルセーフ＝起立が
    // その指定の意味そのもので、そこで止めても意味がない。
    if !opts.allow_no_sbus {
        let sbus = pilot.state();
        if cfg.teleop.mode.position(&sbus) != 0 {
            return Err(format!(
                "{RETRYABLE}CH5（モード）が脱力位置にありません（いま {} 段目 / raw {}）。                 **脱力に戻してから起動してください。** このまま起動すると                 操作なしで立ち上がります",
                cfg.teleop.mode.position(&sbus),
                cfg.teleop
                    .mode
                    .channel
                    .checked_sub(1)
                    .and_then(|i| sbus.channels.get(i).copied())
                    .unwrap_or(0),
            ));
        }
        log::info!("CH5 は脱力位置です");
    }

    // **原点の張り直しは CH5 の確認より後。** 張り直しはそのときの姿勢を
    // 無条件に原点にするので、起動を続ける気が無い回でやってはいけない。
    zero_multiturn_once(cfg, plant.hw())?;
    verify_crouch_frame(cfg, plant.hw());

    Ok(plant)
}

/// **読めていない軸があるうちは脱力のまま。**
///
/// 読めていない軸の観測は 0 のままで、`measured` はそれをそのまま実測として
/// 渡す。脱力からの遷移はこの実測を始点に張るので、実際には畳まれている脚を
/// 「伸び切っている」と思って軌道を作る（namiashi2 は関節角 0 が脚を伸ばし切った
/// 姿勢）。**脱力姿勢は一意に決まらないから、名前で持つのではなく実測を待つ。**
///
/// 途中で来なくなるのは別の話。そちらは安全ゲートが `max_observation_age` で
/// 見て、目標を進めずその場で保持する（脱力へは落とさない — 荷重のかかった
/// 四足を脱力させると崩れる）。
fn mode_until_read(want: misa_core::ModeRequest, unread: bool) -> misa_core::ModeRequest {
    if unread {
        misa_core::ModeRequest::Relax
    } else {
        want
    }
}

pub fn run(
    cfg: AppConfig,
    robot: Robot,
    opts: RunOptions,
    backends: &[&dyn crate::Backend],
) -> Result<(), String> {
    // **繋ぎ方はプロファイルの `kind` が決める。** ここから下は Plant と
    // Pilot のトレイト越しにしか触らないので、実機でもブリッジ越しでも
    // 同じループが回る。
    let (mut plant, mut pilot): (Box<dyn misa_core::Plant>, Box<dyn misa_core::Pilot>) =
        match &cfg.hardware {
            misa_hal::config::HardwareConfig::Serial(_) => {
                // **探索は 1 回だけ。** 脚バス・IMU・受信機で同じ地図を使う。
                let map = PortMap::discover().map_err(|e| e.to_string())?;
                let plant = crate::plant::SerialPlant::connect_with(&cfg, &map)?;
                let pilot =
                    crate::pilot::SbusPilot::connect_with(&cfg, &map, opts.allow_no_sbus)?;
                // 実機だけの立ち上げ手順（受信機を待つ、CH5 の位置、
                // マルチターンの原点、伏せ姿勢との照合）。
                let plant = start_serial(&cfg, &opts, plant, &pilot)?;
                (Box::new(plant), Box::new(pilot))
            }
            // **ブリッジ越しの繋ぎ方は外から差す。** 相手の独自メッセージに
            // 束縛されるので、この crate には入れない（入れると、操縦だけ
            // したい機体でも向こうの ws が要るようになる）。
            misa_hal::config::HardwareConfig::Ros2(_) => {
                match backends.iter().find_map(|b| b.connect(&cfg)) {
                    Some(r) => r?,
                    None => {
                        return Err(
                            "この実行ファイルは kind = \"ros2\" の機体を繋げません。\
                             機体側のリポジトリの実行ファイルを使ってください"
                                .into(),
                        )
                    }
                }
            }
        };

    let stop = install_signal_handler();
    let period = Duration::from_secs_f64(1.0 / cfg.control.rate_hz);
    // **「こちらの指令で動くか」は Plant が名乗る。** 受信機直結の腕は
    // 動いてはいるがアプリの指令では動かないので、駆動しないなら目標角には
    // 観測値を置く（指令値を置くと実機と食い違った角度でログが埋まる）。
    let layout = crate::snapshot::axis_layout(&cfg)?;
    let arm_app_driven = layout
        .head
        .is_some_and(|id| plant.capabilities().driven.get(id.index()) == Some(&true));
    if !arm_app_driven {
        log::info!("補助軸はアプリから駆動しません。目標角には観測値を置きます");
    }
    // Controller へ move する前に控えておく。
    let model_limits = robot.limits.clone();
    let model_rates = robot.rate_limits.clone();
    let model_efforts = robot.effort_limits.clone();
    // **WBC はここで組み立てる。** モデルの不備（足リンクが無い、トルクの
    // 定格が無い）は制御ループへ入る前に落とす。無効なら `None` で、以降の
    // 経路は従来どおり歩容の IK 出力をそのまま位置制御で流す。
    let mut wbc = crate::wbc::WbcRunner::new(&robot, &cfg.wbc)?;
    // **脚オドメトリは WBC の有無に依らず回す。** MPC 歩容も同じ推定を
    // 使うので、出どころは 1 か所（[`crate::estimator`]）。
    let estimator = crate::estimator::BodyEstimator::new(&robot);
    // **Plant が名乗らないモードでは出さない。** 実機のトルク制御は、
    // トルク定数を書くまで単位が食い違う（`AppConfig::torque_unit_mismatch`）。
    // ここで止めないと、N·m が電流 (A) として 12 軸ぶん線に乗る。
    if let Some(w) = wbc.as_ref() {
        let want = w.control_mode();
        if !plant.capabilities().modes.contains(&want) {
            let _ = plant.disarm();
            return Err(format!(
                "この機体は {:?} 制御を扱えません（wbc.output = {:?}）。{}",
                want,
                cfg.wbc.output.label(),
                cfg.torque_unit_mismatch().unwrap_or_default()
            ));
        }
    }
    let mut controller = Controller::with_arm(robot, cfg.clone(), arm_app_driven);

    let mut warned_unread = false;
    let mut last_verdict_clean = true;
    // 傾きの報告は**立ち上がりだけ**。毎周期出すと、倒れたあと床で
    // 転がっている間ずっと埋まる。
    let mut tilt_reported = false;
    let mut publisher = open_viz(&opts.viz)?;
    let started = Instant::now();
    let mut motors_enabled = false;
    let mut measured_seen = false;
    let mut fault_hint_shown = false;
    let mut watch = Watch::new(&cfg, &model_limits);
    let mut wbc_status: Option<(crate::config::WbcOutput, crate::wbc::WbcStatus)>;
    // 記録と、その脇で回す SafetyGate。
    //
    // **ゲートは実機へ流れる指令そのものに掛かる**（2026-09-02 に配線した。
    // それまでは影で回して出力を捨てていた）。可動域・目標の変化率・観測の
    // 古さをここで丸め、何をしたかを `SafetyVerdict` として記録に残す。
    // 記録に載るのは**丸めたあとの指令**で、実機へ出したものと一致する。
    let layout = crate::snapshot::axis_layout(&cfg)?;
    let mut gate = misa_core::SafetyGate::new(crate::snapshot::safety_config(
        &cfg,
        &layout,
        &model_limits,
        &model_rates,
        &model_efforts,
        period.as_secs_f64(),
        STALE_TICKS,
    ));
    let recorder = match opts.record.as_deref() {
        Some(path) => {
            let header = misa_core::record::Header {
                format_version: misa_core::record::FORMAT_VERSION,
                robot: cfg.name.clone(),
                axes: layout
                    .table
                    .axes()
                    .iter()
                    .map(|a| a.name.clone())
                    .collect(),
                rate_hz: cfg.control.rate_hz,
            };
            let rec = crate::record::Recorder::create(path, &header)?;
            log::info!("毎周期を {path} に記録します");
            Some(rec)
        }
        None => None,
    };

    let mut next = Instant::now();
    let mut last_tick = Instant::now();
    let mut last_status = Instant::now();
    let mut worst_overrun = Duration::ZERO;
    let mut ticks: u64 = 0;

    log::info!(
        "制御ループ開始: {:.0} Hz（{}）。Ctrl-C で脱力して終了します",
        cfg.control.rate_hz,
        match cfg.hardware.max_control_rate_hz() {
            Some(hz) => format!("脚バス {hz:.0} Hz"),
            // ブリッジ越しでは向こうの周期に従うので、こちらから言えることが無い。
            None => "周期は相手側が決める".to_string(),
        }
    );

    // 最初の観測を 1 回取っておく。指令は全軸脱力。
    //
    // **`exchange` は「出して、受け取る」で 1 往復。** したがって `tick` が
    // 見る観測は前周期に持ち帰ったものになる（200 Hz で 5 ms）。指令のほうは
    // `tick` の直後に出るので遅れない。要求応答の配備先（namiashi2 の中間層、
    // Unitree の lowcmd/lowstate）では消せない性質なので、実機が只で読める
    // namiashi でも同じ形に揃えてある。
    let mut obs = misa_core::Observation::empty(layout.table.len(), 4);
    plant
        .exchange(&misa_core::Command::idle(layout.table.len()), &mut obs)
        .map_err(|e| format!("実機の初回読み出しに失敗: {e}"))?;

    while !stop.load(Ordering::Relaxed) {

        // **受信が無いときの扱いは 2 通りあり、混ぜてはいけない。**
        // その判断は `SbusPilot` が持つ（受信断は活動度を上げない、
        // `--allow-no-sbus` のベンチは起立させたい）。
        let mut cmd = pilot.poll(obs.time);
        // **まだ 1 軸でも読めていないうちは立ち上がらせない。**
        //
        // 読めていない軸の観測は 0 のままで、`measured` はそれをそのまま
        // 実測として渡す。**脱力からの遷移はこの実測を始点に張る**ので、
        // 実際には畳まれている脚を「伸び切っている」と思って軌道を作る。
        // namiashi2 なら関節角 0 は脚を伸ばし切った姿勢で、そこから立ち姿勢へ
        // 向かう軌道は実機の姿勢とまるで違う。
        //
        // 一度読めたら以降は見ない。途中で来なくなるのは別の話で、
        // そちらは安全ゲートが `max_observation_age` で見る（目標を進めず
        // その場で保持する。脱力へは落とさない — 荷重のかかった四足を
        // 脱力させると崩れる）。
        if !measured_seen {
            measured_seen = layout.commanded_axes_read(&obs);
            if !measured_seen && !warned_unread {
                warned_unread = true;
                log::warn!(
                    "まだ状態を受け取れていないので脱力のまま待ちます\
                     （読めていない軸の観測は 0 で、そこから立つと危ない）"
                );
            }
            if measured_seen {
                log::info!("状態を受け取りました。操縦を受け付けます");
            }
        }
        cmd.mode = mode_until_read(cmd.mode, !measured_seen);
        // 受信機直結の腕は、プロポのチャンネルから読んだ角度が唯一の手がかり。
        if let Some(observed) = cmd.aux(0) {
            if let Some(id) = layout.head {
                plant.observe_aux(id, observed);
            }
        }
        let measured = jointvec_from(&obs);
        let attitude = obs.imu.map(|i| i.rpy_rad).unwrap_or([0.0; 3]);

        let mut out = controller.tick(&cmd, &measured, attitude, period.as_secs_f64());
        // **計画した立脚を実測の接地で直す。** WBC の「立脚足が滑らない」は
        // 硬い制約なので、接地していない足を接地と信じると解が壊れる。
        if cfg.wbc.use_measured_contact {
            out.stance = crate::estimator::stance_with_measured_contact(out.stance, &obs);
        }

        // **胴体の状態は歩容の出力が出てから測る**（立脚フラグと計画した
        // 関節角が要る）。測った結果は歩容へ返して**次の周期**で使わせる。
        let measured_qd = crate::estimator::velocities_from(&obs);
        let gyro = obs.imu.map(|i| i.gyro_rad_s).unwrap_or([0.0; 3]);
        let body = estimator.estimate(
            &measured,
            &measured_qd,
            &out.targets,
            attitude,
            gyro,
            out.stance,
        );
        controller.observe_body(&body);

        // モータの投入・切断は状態が変わった瞬間だけ。毎周期投げると
        // バスの帯域を食うし、`motor_run` の連打はモータ側にも優しくない。
        let want_enabled = out.leg_mode != JointMode::Idle;
        if want_enabled != motors_enabled {
            let r = if want_enabled {
                plant.arm()
            } else {
                plant.disarm()
            };
            if let Err(e) = r {
                log::error!("モータの投入／切断を送れません: {e}");
                break;
            }
            motors_enabled = want_enabled;
        }

        // 脱力中は目標を送っていないので、追従誤差を見ても意味がない。
        if out.leg_mode != JointMode::Idle {
            watch.tick(&out.targets, &measured);
        }
        // **WBC は歩容の目標を「置き換える」のではなく「解き直す」。**
        // 入力は同じ観測と同じ歩容の目標で、出るのは τ（と、それを積分した
        // 位置・速度）。`None` なら従来どおり歩容の IK 出力がそのまま出る。
        let plan = wbc
            .as_mut()
            .and_then(|w| {
                w.tick(
                    &out,
                    &obs,
                    &measured,
                    &measured_qd,
                    &body,
                    last_tick.elapsed().as_secs_f64(),
                )
            });
        // 状態行に出す用。**解いていない周期は `None`** なので、
        // 「WBC 有効だが歩容が回っていない」と「解けている」が潰れない。
        wbc_status = plan
            .as_ref()
            .and_then(|p| wbc.as_ref().map(|w| (w.output(), p.status)));
        let mut outgoing = crate::snapshot::command(
            &layout,
            &out.targets,
            cfg.hardware.default_max_speed_rad_s(),
            out.leg_mode == JointMode::Idle,
            cfg.hardware.mit_gains(),
            plan.as_ref(),
        );

        // **ここが指令を書き換えてよい唯一の場所。** `dt` は実測を渡す
        // （目標周期を渡すと、ループが遅れている間に目標だけ規定どおり
        // 進んで変化率の制限が意味を失う）。
        let verdict = gate.apply(&mut outgoing, &obs, last_tick.elapsed());
        // **丸めたことを黙っていない。** ただし毎周期出すと埋もれるので、
        // 状態が変わったときだけ。可動域や変化率に当たり続けているのは、
        // 歩容か設定のどちらかが機体に合っていないという意味。
        if verdict.is_clean() != last_verdict_clean {
            last_verdict_clean = verdict.is_clean();
            if verdict.is_clean() {
                log::info!("安全ゲート: 丸めなくなりました");
            } else {
                log::warn!(
                    "安全ゲート: 可動域 {} 軸 / 変化率 {} 軸 / トルク {} 軸\
                     {}{}",
                    verdict.clamped.len(),
                    verdict.rate_limited.len(),
                    verdict.torque_limited.len(),
                    if verdict.held_for_stale_observation {
                        " / 観測が古いので目標を進めていません"
                    } else {
                        ""
                    },
                    if verdict.faulted.is_empty() {
                        String::new()
                    } else {
                        format!(" / 異常ビット {} 軸", verdict.faulted.len())
                    }
                );
            }
        }

        // **傾きは丸めとは別に報告する。** 接地センサが無いので、転倒に
        // 近づいたことを知る手がかりは姿勢角しかない。**自動では脱力しない**
        // （立っている四足を脱力させると崩れる）ので、止めるかどうかは
        // operator が決める。
        match (verdict.tilt_rad, tilt_reported) {
            (Some(tilt), false) => {
                tilt_reported = true;
                log::error!(
                    "**胴体が {:.0}° 傾いています**（上限 {:.0}°）。\
                     転倒しかけているかもしれません。自動では脱力しません — \
                     脱力させるか電源を切るかは operator が決めてください",
                    tilt.to_degrees(),
                    cfg.max_tilt_rad().to_degrees()
                );
            }
            (None, true) => {
                tilt_reported = false;
                log::info!("胴体の傾きが上限の内側へ戻りました");
            }
            _ => {}
        }

        // 記録は**実機へ出した指令**（丸めたあと）と、それを計算するのに
        // 使った観測の組。
        if let Some(rec) = recorder.as_ref() {
            rec.push(misa_core::record::Frame {
                seq: ticks,
                time: obs.time,
                intent: cmd.clone(),
                observation: obs.clone(),
                command: outgoing.clone(),
                verdict,
            });
        }
        last_tick = Instant::now();

        if let Err(e) = plant.exchange(&outgoing, &mut obs) {
            log::error!("実機との往復に失敗: {e}");
            break;
        }

        if let Some(p) = publisher.as_mut() {
            // 最初の読み戻しが済むまで measured を送らない。ゼロ姿勢のフレームは
            // 受け側で「崩れ落ちたロボット」として描かれる。
            // 一度立ったら見に行かない（`all_ok` は 12 軸ぶんロックを取る）。
            // 最初の読み戻しが済んだかは観測そのものが知っている。
            let body = controller.body_view();
            let t = started.elapsed().as_secs_f64();
            let att = attitude;
            p.maybe_publish(|seq| {
                let planned = viz::frame(seq, t, &out.targets, &body);
                if !measured_seen {
                    return viz::Frames::planned(planned);
                }
                // 胴体の姿勢 3 軸は IMU の実測。位置 x, y と高さはオドメトリが
                // 無いので歩容の値のまま。**実測なのは 12 関節と姿勢だけ**で、
                // 位置を入れたらここを差し替える。
                let measured_body = viz::BodyView {
                    rp: [att[0], att[1]],
                    yaw: att[2],
                    ..body
                };
                viz::Frames::both(planned, viz::frame(seq, t, &measured, &measured_body))
            });
        }

        if controller.state_changed() {
            log::info!("状態: {}", out.state.label());
        }

        ticks += 1;
        if opts.status_interval_s > 0.0
            && last_status.elapsed().as_secs_f64() >= opts.status_interval_s
        {
            log_status(
                plant.as_ref(),
                pilot.as_ref(),
                &obs,
                &controller,
                &cmd,
                ticks,
                worst_overrun,
                &mut fault_hint_shown,
                &mut watch,
                wbc_status,
            );
            last_status = Instant::now();
            worst_overrun = Duration::ZERO;
        }

        next += period;
        let now = Instant::now();
        if next > now {
            std::thread::sleep(next - now);
        } else {
            worst_overrun = worst_overrun.max(now - next);
            next = now;
        }
    }

    if let Some(rec) = recorder {
        let dropped = rec.dropped();
        match rec.finish() {
            Ok(n) if dropped == 0 => log::info!("{n} 周期を記録しました"),
            // **取りこぼしのある記録を、完全な記録と取り違えないこと。**
            Ok(n) => log::warn!("{n} 周期を記録しました（{dropped} 周期は取りこぼし）"),
            Err(e) => log::error!("記録を閉じられません: {e}"),
        }
    }

    log::info!("停止要求を受けました。脱力します");
    let _ = plant.disarm();
    // 実機なら、バススレッドが Disable を実際に送るまで待ってから drop する。
    std::thread::sleep(Duration::from_millis(100));
    Ok(())
}

/// ライブ可視化の配信器を開く。無効なら `None`。
///
/// `viz` フィーチャを外したビルドで `--viz` を渡されると
/// `Publisher::new` がエラーを返す（黙って無視しない）。
pub(crate) fn open_viz(cfg: &VizConfig) -> Result<Option<viz::Publisher>, String> {
    if !cfg.enabled {
        return Ok(None);
    }
    viz::Publisher::new(cfg).map(Some)
}

/// 目標角を 4 本のバスへ配る。
/// 観測を関節ベクトルへ。並びは [`crate::snapshot::axis_table`] と同じ。
///
/// 制御則がまだ [`JointVec`] を受け取るための橋渡し。Policy が
/// `Observation` を直接取るようになったら消える。
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

fn log_status(
    plant: &dyn misa_core::Plant,
    pilot: &dyn misa_core::Pilot,
    obs: &misa_core::Observation,
    controller: &Controller,
    cmd: &misa_core::Intent,
    ticks: u64,
    worst_overrun: Duration,
    fault_hint_shown: &mut bool,
    watch: &mut Watch,
    wbc: Option<(crate::config::WbcOutput, crate::wbc::WbcStatus)>,
) {
    log::info!(
        "[{}] {} v=({:+.3},{:+.3},{:+.3}) {} {} tick={} 遅延最大={:.1}ms 追従最大={}",
        controller.state().label(),
        controller.gait_select().label(),
        cmd.velocity.vx_m_s,
        cmd.velocity.vy_m_s,
        cmd.velocity.wz_rad_s,
        plant.status_line(),
        pilot.status_line(),
        ticks,
        worst_overrun.as_secs_f64() * 1e3,
        watch.take(),
    );
    // **WBC は回っていることを黙らせない。** 出力の種類が変わると
    // 実機の挙動が根本的に変わるので、状態行に出す。接地力の合計が体重と
    // 大きく違う、τ が上限に張り付いている、といった兆候はここに出る。
    if let Some((output, st)) = wbc {
        log::info!(
            "  WBC {} 出力 / 参照 {} / 立脚 {}/4 / τ最大 {:.2} N·m / Σfz {:.1} N",
            output.label(),
            if st.mpc_driven { "MPC" } else { "準静的" },
            st.stance_count,
            st.tau_max_nm,
            st.f_z_total_n,
        );
    }
    // 異常ビットは埋もれさせない。自動で脱力はしない（立っている四足を
    // 脱力させると倒れる）ので、operator がモードスイッチで判断できるよう
    // 毎回はっきり出す。
    let mut any = false;
    for (id, st) in obs.faulted() {
        any = true;
        log::error!(
            "  異常: {} **{}**{}",
            plant.axes().name(id).unwrap_or("?"),
            plant.describe_fault(st.health.fault_raw),
            match st.health.temperature_c {
                Some(t) => format!("{t:.0} °C"),
                None => String::new(),
            },
        );
    }
    // 異常が消えたら次に出たときまた出す。
    if !any {
        *fault_hint_shown = false;
    }
    if controller.state() == State::PlayingPose {
        if let Some(name) = controller.playing() {
            log::info!("  再生中: {name}");
        }
    }
    if any && !*fault_hint_shown {
        *fault_hint_shown = true;
        log::error!(
            "  **自動では脱力しません。** 立っている四足を脱力させると倒れるので、\
             止めるかどうかは operator が決めてください"
        );
    }
}

/// Ctrl-C (SIGINT) / SIGTERM で立つフラグ。
static STOP_FLAG: AtomicBool = AtomicBool::new(false);

/// シグナルハンドラを仕掛ける。
///
/// ハンドラの中では `AtomicBool` を立てるだけにして、脱力処理はメインスレッド
/// で行う。ハンドラからシリアル I/O をするのは非同期シグナル安全でないし、
/// 途中で握っているロックがあれば自己デッドロックになる。
///
/// **仕掛けたら、そのコマンドのループは必ず戻り値のフラグを見ること。**
/// SIGINT の既定動作（プロセス終了）を奪うので、見ないループで呼ぶと
/// Ctrl-C がまったく効かなくなる。`diag` の `--forever` もこれを使う。
pub(crate) fn install_signal_handler() -> &'static AtomicBool {
    unsafe {
        let handler = handle_signal as extern "C" fn(libc::c_int) as libc::sighandler_t;
        libc::signal(libc::SIGINT, handler);
        libc::signal(libc::SIGTERM, handler);
    }
    &STOP_FLAG
}

extern "C" fn handle_signal(_sig: libc::c_int) {
    STOP_FLAG.store(true, Ordering::Relaxed);
}

#[cfg(test)]
mod startup_tests {
    use super::mode_until_read;
    use misa_core::ModeRequest;

    /// **1 軸でも読めていないうちは立ち上がらせない。**
    ///
    /// 読めていない軸の観測は 0 で、脱力からの遷移はその実測を始点に張る。
    /// namiashi2 なら関節角 0 は脚を伸ばし切った姿勢なので、実際には畳まれている
    /// 脚を伸び切っていると思って軌道を作ることになる。
    #[test]
    fn nothing_stands_before_the_first_state_arrives() {
        for want in [ModeRequest::Relax, ModeRequest::Stand, ModeRequest::Walk] {
            assert_eq!(mode_until_read(want, true), ModeRequest::Relax, "{want:?}");
        }
    }

    /// 読めたら操縦をそのまま通す。**握り潰さない。**
    #[test]
    fn once_read_the_pilot_gets_through() {
        for want in [ModeRequest::Relax, ModeRequest::Stand, ModeRequest::Walk] {
            assert_eq!(mode_until_read(want, false), want);
        }
    }
}
