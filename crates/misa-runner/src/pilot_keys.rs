//! キーボードで操縦する [`Pilot`]。**MuJoCo を見ながら手で動かすためのもの。**
//!
//! プロポ（S.BUS）も ROS 2 も無いところで歩容を触りたいときに使う。実機の
//! 立ち上げ前に「この速度指令だと何が起きるか」を目で見て確かめるのが用途で、
//! **実機で使うことは想定していない**（`sim` だけに繋いである）。
//!
//! # 押しっぱなしは取れない
//!
//! 端末はキーを離したことを教えてくれない。**押すたびに 1 段ずつ足す**形に
//! してあり、止めるのはスペース。プロポのスティックのように「離したら中立」
//! にはならないので、**目を離すなら先に止めること。**
//!
//! # 実機でも使える（`run --pilot keys`）— ただしデッドマン付き
//!
//! 「離しても止まらない」は実機では危ないので、`run` では **一定時間キーが
//! 来なければ速度を 0 に落とす**（既定 0.5 s。`cmd_vel` の 300 ms タイムアウト
//! と同じ考え方）。歩き続けるには方向キーを**押し続ける**（端末のオート
//! リピートがキーを送り続ける）。押し続けているあいだのリピートは「1 段足す」
//! ではなく**生存確認**として扱う（`REPEAT_WINDOW` より短い間隔の同じキー）ので、
//! 押し続けても速度は上がらない。タップすれば 1 段足す。sim も同じ扱い
//! （デッドマンだけが無い）。
//!
//! # 端末の設定を必ず戻す
//!
//! raw モードにしたまま落ちると、そのシェルはエコーも改行も効かなくなる
//! （`reset` を打つまで）。[`RawMode`] が `Drop` で戻すのに加えて、
//! Ctrl-C でも戻るように `poll` が終了要求を見ている。

use std::io::Read;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use misa_core::{GaitControllerRequest, GaitSelect, GaitTune, Intent, ModeRequest, Pilot, Time, Velocity, WbcRequest};

use crate::config::AppConfig;

/// 1 回の押下で動く量。
///
/// **細かすぎると届くまでに何度も押すことになり、粗すぎると跨いでしまう。**
/// 上限（`gait.max_*`）の 1/8 を 1 段にしてある。
const STEPS: usize = 8;

/// 押下 → 意図の変化。**端末に依らないので試験できる。**
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Key {
    Forward,
    Back,
    Left,
    Right,
    TurnLeft,
    TurnRight,
    Stop,
    Mode(ModeRequest),
    Gait(GaitSelect),
    Higher,
    Lower,
    TiltUp,
    TiltDown,
    TiltLeft,
    TiltRight,
    Level,
    Tune(Knob, i8),
    TuneReset,
    /// 全身制御の出力を巡回（OFF → 位置 → トルク → OFF）。
    WbcCycle,
    /// 歩容コントローラを MPC ↔ CHAMP で切り替える（立っているときだけ効く）。
    ControllerToggle,
    Help,
    Quit,
}

/// 実行中に触れる歩容パラメータ。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Knob {
    /// 1 周期の時間。**揺れにいちばん効く。**
    Cycle,
    /// 遊脚の頂点の高さ。
    Swing,
    /// 歩幅の上限。
    Step,
    /// 接地比。
    Duty,
}

impl Knob {
    /// 1 回の押下で動く量。**速度と違って上限の 1/8 では粗すぎる**ので、
    /// パラメータごとに実用的な刻みを決め打ちしてある。
    fn step(self) -> f64 {
        match self {
            Knob::Cycle => 0.05,
            Knob::Swing => 0.005,
            Knob::Step => 0.01,
            Knob::Duty => 0.02,
        }
    }

    fn label(self) -> &'static str {
        match self {
            Knob::Cycle => "周期",
            Knob::Swing => "遊脚",
            Knob::Step => "歩幅",
            Knob::Duty => "接地比",
        }
    }
}

/// 1 文字を押下へ。知らない文字は `None`。
///
/// **大文字も受ける。** Shift が掛かったまま打っても効かないのは事故のもと。
pub fn decode(c: u8) -> Option<Key> {
    Some(match c.to_ascii_lowercase() {
        b'w' => Key::Forward,
        b's' => Key::Back,
        b'a' => Key::Left,
        b'd' => Key::Right,
        b'q' => Key::TurnLeft,
        b'e' => Key::TurnRight,
        b' ' => Key::Stop,
        b'0' => Key::Mode(ModeRequest::Relax),
        b'1' => Key::Mode(ModeRequest::Stand),
        b'2' => Key::Mode(ModeRequest::Walk),
        b'z' => Key::Gait(GaitSelect::Crawl),
        b'x' => Key::Gait(GaitSelect::Walk),
        b'c' => Key::Gait(GaitSelect::Trot),
        b'r' => Key::Higher,
        b'f' => Key::Lower,
        b'i' => Key::TiltUp,
        b'k' => Key::TiltDown,
        b'j' => Key::TiltLeft,
        b'l' => Key::TiltRight,
        b'v' => Key::Level,
        // **歩容パラメータ。上段が +、下段が −。** 走らせながら詰める用。
        b't' => Key::Tune(Knob::Cycle, 1),
        b'g' => Key::Tune(Knob::Cycle, -1),
        b'y' => Key::Tune(Knob::Swing, 1),
        b'b' => Key::Tune(Knob::Swing, -1),
        b'u' => Key::Tune(Knob::Step, 1),
        b'n' => Key::Tune(Knob::Step, -1),
        b'.' => Key::Tune(Knob::Duty, 1),
        b',' => Key::Tune(Knob::Duty, -1),
        b'm' => Key::TuneReset,
        b'o' => Key::WbcCycle,
        b'p' => Key::ControllerToggle,
        b'h' | b'?' => Key::Help,
        // **Ctrl-C も自分で拾う。** raw モードでは端末が SIGINT を出さない
        // ので、これを見落とすと止められなくなる。
        b'\x03' | b'\x1b' => Key::Quit,
        _ => return None,
    })
}

/// 操縦の上限。プロファイルから採る。
#[derive(Debug, Clone, Copy)]
pub struct Limits {
    pub max_vx: f64,
    pub max_vy: f64,
    pub max_wz: f64,
    pub height_range: f64,
    pub attitude_max: f64,
    /// 歩容ごとの基準値（Crawl / Walk / Trot の順）。
    ///
    /// **「1 段上げる」を書くには基準値が要る。** 周期は歩容ごとに違うので
    /// 3 つ持ち、歩容を替えたらその歩容の基準へ戻す。
    pub base_tune: [GaitTune; 3],
    /// 起動時の WBC / 歩容コントローラ。`o` / `p` の巡回はここから数える。
    pub wbc_initial: WbcRequest,
    pub controller_initial: GaitControllerRequest,
}

/// 歩容 → `base_tune` の添字。
fn gait_index(g: GaitSelect) -> usize {
    match g {
        GaitSelect::Crawl => 0,
        GaitSelect::Walk => 1,
        GaitSelect::Trot => 2,
    }
}

impl Limits {
    pub fn from(cfg: &AppConfig) -> Self {
        Self {
            max_vx: cfg.gait.max_vx_m_s,
            max_vy: cfg.gait.max_vy_m_s,
            max_wz: cfg.gait.max_wz_rad_s,
            height_range: cfg.gait.height_range_m,
            attitude_max: cfg.gait.body_attitude_max_rad,
            base_tune: [GaitSelect::Crawl, GaitSelect::Walk, GaitSelect::Trot]
                .map(|g| crate::robot::base_gait_tune(&cfg.gait, g)),
            wbc_initial: match (cfg.wbc.enabled, cfg.wbc.output) {
                (false, _) => WbcRequest::Off,
                (true, crate::config::WbcOutput::Torque) => WbcRequest::Torque,
                (true, _) => WbcRequest::Position,
            },
            controller_initial: match cfg.gait.controller {
                crate::config::GaitControllerKind::Mpc
                | crate::config::GaitControllerKind::Centroidal => GaitControllerRequest::Mpc,
                _ => GaitControllerRequest::Champ,
            },
        }
    }

    /// その歩容の基準値。
    pub fn base_of(&self, g: GaitSelect) -> GaitTune {
        self.base_tune[gait_index(g)]
    }
}

/// 押下を意図へ反映する。**`Quit` は呼び出し側が見る**（ここでは何もしない）。
pub fn apply(intent: &mut Intent, key: Key, lim: &Limits) {
    let clamp = |v: f64, max: f64| v.clamp(-max, max);
    match key {
        Key::Forward => {
            intent.velocity.vx_m_s = clamp(
                intent.velocity.vx_m_s + lim.max_vx / STEPS as f64,
                lim.max_vx,
            )
        }
        Key::Back => {
            intent.velocity.vx_m_s = clamp(
                intent.velocity.vx_m_s - lim.max_vx / STEPS as f64,
                lim.max_vx,
            )
        }
        Key::Left => {
            intent.velocity.vy_m_s = clamp(
                intent.velocity.vy_m_s + lim.max_vy / STEPS as f64,
                lim.max_vy,
            )
        }
        Key::Right => {
            intent.velocity.vy_m_s = clamp(
                intent.velocity.vy_m_s - lim.max_vy / STEPS as f64,
                lim.max_vy,
            )
        }
        Key::TurnLeft => {
            intent.velocity.wz_rad_s = clamp(
                intent.velocity.wz_rad_s + lim.max_wz / STEPS as f64,
                lim.max_wz,
            )
        }
        Key::TurnRight => {
            intent.velocity.wz_rad_s = clamp(
                intent.velocity.wz_rad_s - lim.max_wz / STEPS as f64,
                lim.max_wz,
            )
        }
        // **止めるのは速度だけ。** モードや姿勢まで戻すと、慌てて叩いたときに
        // 立っている機体が脱力しかねない。
        Key::Stop => intent.velocity = Velocity::ZERO,
        Key::Mode(m) => intent.mode = m,
        // **歩容を替えたら、その歩容の基準値へ戻す。** 周期の基準が歩容ごとに
        // 違うので、trot で詰めた 0.40 s を crawl へ持ち込むと訳が分からなく
        // なる。制御側も切り替えで上書きを落とすので、これで揃う。
        Key::Gait(g) => {
            intent.gait = g;
            intent.gait_tune = lim.base_of(g);
        }
        Key::Higher => {
            intent.height_offset_m = clamp(
                intent.height_offset_m + lim.height_range / STEPS as f64,
                lim.height_range,
            )
        }
        Key::Lower => {
            intent.height_offset_m = clamp(
                intent.height_offset_m - lim.height_range / STEPS as f64,
                lim.height_range,
            )
        }
        Key::TiltUp | Key::TiltDown | Key::TiltLeft | Key::TiltRight => {
            let step = lim.attitude_max / STEPS as f64;
            let (dr, dp) = match key {
                Key::TiltUp => (0.0, step),
                Key::TiltDown => (0.0, -step),
                Key::TiltLeft => (-step, 0.0),
                _ => (step, 0.0),
            };
            let mut r = intent.body_attitude_rad[0] + dr;
            let mut p = intent.body_attitude_rad[1] + dp;
            // **上限は合成量で見る。** 軸ごとに丸めると、斜めに振ったぶんが
            // 合わさって脚の可動域を食う（サービス側と同じ扱い）。
            let n = (r * r + p * p).sqrt();
            if n > lim.attitude_max && n > 0.0 {
                let k = lim.attitude_max / n;
                r *= k;
                p *= k;
            }
            intent.body_attitude_rad[0] = r;
            intent.body_attitude_rad[1] = p;
        }
        Key::Level => intent.body_attitude_rad = [0.0; 3],
        Key::Tune(knob, dir) => {
            let d = knob.step() * f64::from(dir);
            let t = &mut intent.gait_tune;
            // **上書きしていない項目は基準値から始める。** 0 から始めると
            // 1 回目の押下で歩容が跳ぶ。
            let base = lim.base_of(intent.gait);
            let f = match knob {
                Knob::Cycle => (&mut t.cycle_period_s, base.cycle_period_s),
                Knob::Swing => (&mut t.swing_height_m, base.swing_height_m),
                Knob::Step => (&mut t.step_length_m, base.step_length_m),
                Knob::Duty => (&mut t.duty_factor, base.duty_factor),
            };
            *f.0 = Some(f.0.or(f.1).unwrap_or(0.0) + d);
            *t = t.clamped();
        }
        Key::TuneReset => intent.gait_tune = lim.base_of(intent.gait),
        // **最初の 1 押しは起動時の設定から数える**（`None` は「そのまま」）。
        Key::WbcCycle => {
            intent.wbc = Some(match intent.wbc.unwrap_or(lim.wbc_initial) {
                WbcRequest::Off => WbcRequest::Position,
                WbcRequest::Position => WbcRequest::Torque,
                WbcRequest::Torque => WbcRequest::Off,
            })
        }
        Key::ControllerToggle => {
            intent.gait_controller = Some(match intent.gait_controller.unwrap_or(lim.controller_initial) {
                GaitControllerRequest::Champ => GaitControllerRequest::Mpc,
                GaitControllerRequest::Mpc => GaitControllerRequest::Champ,
            })
        }
        Key::Help | Key::Quit => {}
    }
}

/// 操作の一覧。**起動時に 1 回出す。**
///
/// 歩容パラメータの欄は `gait` の基準値を出す（歩容ごとに違う）。
pub fn help(lim: &Limits, gait: GaitSelect) -> String {
    let base = lim.base_of(gait);
    format!(
        "キーボードで操縦します（押すたびに 1 段。**離しても止まりません**）\n\
         \n\
         　  w / s    前後   ±{:.2} m/s まで（1 段 {:.3}）\n\
         　  a / d    横     ±{:.2} m/s まで\n\
         　  q / e    旋回   ±{:.2} rad/s まで\n\
         　  space    **速度を 0 に**（モードと姿勢はそのまま）\n\
         \n\
         　  0 / 1 / 2   脱力 / 初期姿勢 / 歩行\n\
         　  z / x / c   Crawl / Walk / Trot\n\
         　  r / f       立ち高さ ±{:.2} m\n\
         　  i / k / j / l   胴体を傾ける（合成 {:.2} rad まで）、v で水平へ\n\
         \n\
         　**歩容パラメータ（歩きながら替えられます。上段が + / 下段が −）**\n\
         　  t / g    周期   {:.2} s   1 段 0.05（**揺れにいちばん効く**）\n\
         　  y / b    遊脚   {:.3} m   1 段 0.005\n\
         　  u / n    歩幅   {:.3} m   1 段 0.01（速度から決まる着地点の上限）\n\
         　  . / ,    接地比 {:.2}     1 段 0.02（0.5 が trot）\n\
         　  m        いまの歩容の基準値へ戻す\n\
         　  ※ z / x / c で歩容を替えると、その歩容の基準値に戻ります\n\
         \n\
         　**制御の構成（走らせながら替えられます）**\n\
         　  o        全身制御の出力を巡回  OFF → 位置 → トルク → OFF\n\
         　  p        歩容コントローラ MPC ↔ CHAMP（**立って止まっているときだけ**効く）\n\
         \n\
         　  h / ?    この一覧    Esc / Ctrl-C    終了（脱力して抜けます）\n",
        lim.max_vx,
        lim.max_vx / STEPS as f64,
        lim.max_vy,
        lim.max_wz,
        lim.height_range,
        lim.attitude_max,
        base.cycle_period_s.unwrap_or(0.0),
        base.swing_height_m.unwrap_or(0.0),
        base.step_length_m.unwrap_or(0.0),
        base.duty_factor.unwrap_or(0.0),
    )
}

/// 端末を raw モードにして、`Drop` で必ず戻す。
///
/// **戻し忘れるとシェルが壊れたように見える**（エコーも改行も効かない）。
struct RawMode {
    saved: libc::termios,
    fd: i32,
}

impl RawMode {
    fn enter() -> Result<Self, String> {
        let fd = 0; // stdin
        unsafe {
            if libc::isatty(fd) == 0 {
                return Err("標準入力が端末ではありません（キーボード操縦はできません）".into());
            }
            let mut saved: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(fd, &mut saved) != 0 {
                return Err("端末の設定を読めません".into());
            }
            let mut raw = saved;
            // 行バッファとエコーを切る。**1 文字来たらすぐ返す。**
            raw.c_lflag &= !(libc::ICANON | libc::ECHO);
            raw.c_cc[libc::VMIN] = 1;
            raw.c_cc[libc::VTIME] = 0;
            if libc::tcsetattr(fd, libc::TCSANOW, &raw) != 0 {
                return Err("端末を raw モードにできません".into());
            }
            Ok(Self { saved, fd })
        }
    }
}

impl Drop for RawMode {
    fn drop(&mut self) {
        unsafe {
            libc::tcsetattr(self.fd, libc::TCSANOW, &self.saved);
        }
    }
}

/// 同じ方向キーがこれより短い間隔で来たらオートリピート（押し続け）と見なす。
/// 端末のリピートは 30 ms 前後、人が叩き直すのは 150 ms より遅い。
const REPEAT_WINDOW: Duration = Duration::from_millis(120);

/// 共有される操縦の意図。
struct Shared {
    intent: Intent,
    quit: bool,
    last_key: Option<Instant>,
    /// 直前の方向キーとその時刻。押し続け（リピート）の判定に使う。
    last_dir_key: Option<(Key, Instant)>,
    /// デッドマンで速度を落としたことを 1 度だけ言うためのフラグ。
    deadman_tripped: bool,
}

/// 1 キーを共有状態へ反映する。**読み取りスレッドからも試験からも呼ぶ**ので
/// 純粋に書いてある。
fn on_key(l: &mut Shared, key: Key, now: Instant, limits: &Limits) {
    l.last_key = Some(now);
    l.deadman_tripped = false;
    let is_dir = matches!(
        key,
        Key::Forward | Key::Back | Key::Left | Key::Right | Key::TurnLeft | Key::TurnRight
    );
    if is_dir {
        let repeat = matches!(l.last_dir_key, Some((k, t)) if k == key && now.duration_since(t) < REPEAT_WINDOW);
        l.last_dir_key = Some((key, now));
        if repeat {
            // 押し続け: 生存確認だけ。1 段足すのはタップのとき。
            return;
        }
    }
    match key {
        Key::Quit => l.quit = true,
        Key::Help => {
            let g = l.intent.gait;
            print!("\r{}", help(limits, g))
        }
        other => apply(&mut l.intent, other, limits),
    }
}

/// デッドマン: `deadman` のあいだキーが来なければ速度を 0 に落とす。
/// 落としたら true（呼び出し側が 1 度だけログを出す）。
fn deadman_check(l: &mut Shared, now: Instant, deadman: Duration) -> bool {
    let Some(last) = l.last_key else { return false };
    if now.duration_since(last) < deadman || l.intent.velocity == Velocity::ZERO {
        return false;
    }
    l.intent.velocity = Velocity::ZERO;
    if l.deadman_tripped {
        return false;
    }
    l.deadman_tripped = true;
    true
}

pub struct KeyPilot {
    shared: Arc<Mutex<Shared>>,
    stop: Arc<AtomicBool>,
    limits: Limits,
    /// キーがこれだけ来なければ速度を 0 に落とす。`None` で落とさない（sim）。
    deadman: Option<Duration>,
    /// **持っているあいだだけ raw モード。** 落ちても `Drop` で戻る。
    _raw: RawMode,
}

impl KeyPilot {
    /// sim 用。デッドマン無し。
    pub fn open(cfg: &AppConfig, gait: GaitSelect) -> Result<Self, String> {
        Self::open_with(cfg, gait, None)
    }

    /// `deadman` を付けて開く。**実機（`run --pilot keys`）はこちら。**
    pub fn open_with(cfg: &AppConfig, gait: GaitSelect, deadman: Option<Duration>) -> Result<Self, String> {
        let limits = Limits::from(cfg);
        let raw = RawMode::enter()?;
        print!("{}", help(&limits, gait));
        if let Some(d) = deadman {
            print!(
                "\r　**デッドマン {:.1} s**: キーが来なければ速度 0。歩き続けるには方向キーを押し続ける\n\r\n",
                d.as_secs_f64()
            );
        }

        let shared = Arc::new(Mutex::new(Shared {
            intent: Intent {
                // **脱力から始める。** 起動した瞬間に立ち上がらないこと。
                mode: ModeRequest::Relax,
                gait,
                aux_rad: vec![None],
                link_ok: true,
                // **最初からプロファイルの値を持つ。** 上書きが空のままだと
                // 1 回目の押下で「0 から 1 段」になって歩容が跳ぶ。
                gait_tune: limits.base_of(gait),
                ..Intent::default()
            },
            quit: false,
            last_key: None,
            last_dir_key: None,
            deadman_tripped: false,
        }));
        let stop = Arc::new(AtomicBool::new(false));

        let s = Arc::clone(&shared);
        let st = Arc::clone(&stop);
        // **読み取りは別スレッド。** `read` は 1 文字来るまで待つので、
        // 制御ループから直接呼ぶと 200 Hz が崩れる。
        std::thread::Builder::new()
            .name("keys".into())
            .spawn(move || {
                let mut buf = [0u8; 1];
                let mut stdin = std::io::stdin();
                while !st.load(Ordering::Relaxed) {
                    match stdin.read(&mut buf) {
                        Ok(1) => {
                            let Some(key) = decode(buf[0]) else { continue };
                            let mut l = s.lock().unwrap_or_else(|e| e.into_inner());
                            on_key(&mut l, key, Instant::now(), &limits);
                        }
                        Ok(_) => {}
                        Err(_) => break,
                    }
                }
            })
            .map_err(|e| format!("キー入力のスレッドを作れません: {e}"))?;

        Ok(Self {
            shared,
            stop,
            limits,
            deadman,
            _raw: raw,
        })
    }

}

impl Drop for KeyPilot {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

impl Pilot for KeyPilot {
    /// Esc / Ctrl-C が押されたか。**`Pilot` 越しに見えないと意味が無い**
    /// （呼び出し側は `Box<dyn Pilot>` しか持っていない）。
    fn quit_requested(&self) -> bool {
        self.shared.lock().unwrap_or_else(|e| e.into_inner()).quit
    }

    fn poll(&mut self, _now: Time) -> Intent {
        let mut l = self.shared.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(d) = self.deadman {
            if deadman_check(&mut l, Instant::now(), d) {
                log::warn!("キー入力が {:.1} s 無いので速度を 0 にしました（押し続ければ歩き続けます）", d.as_secs_f64());
            }
        }
        l.intent.clone()
    }

    fn status_line(&self) -> String {
        let l = self.shared.lock().unwrap_or_else(|e| e.into_inner());
        let since = l
            .last_key
            .map(|t| format!("{:.0}s前", t.elapsed().as_secs_f64()))
            .unwrap_or_else(|| "未入力".into());
        let t = l.intent.gait_tune;
        let base = self.limits.base_of(l.intent.gait);
        // **基準値と違う項目に * を付ける。** 触ったかどうかが一目で分かる。
        let mark = |v: Option<f64>, b: Option<f64>| if v == b { " " } else { "*" };
        format!(
            "キー {since} v=({:+.2},{:+.2},{:+.2}) 高さ{:+.2} 傾き({:+.2},{:+.2}) \
             周期{}{:.2} 遊脚{}{:.3} 歩幅{}{:.3} 接地比{}{:.2}",
            l.intent.velocity.vx_m_s,
            l.intent.velocity.vy_m_s,
            l.intent.velocity.wz_rad_s,
            l.intent.height_offset_m,
            l.intent.body_attitude_rad[0],
            l.intent.body_attitude_rad[1],
            mark(t.cycle_period_s, base.cycle_period_s),
            t.cycle_period_s.unwrap_or(0.0),
            mark(t.swing_height_m, base.swing_height_m),
            t.swing_height_m.unwrap_or(0.0),
            mark(t.step_length_m, base.step_length_m),
            t.step_length_m.unwrap_or(0.0),
            mark(t.duty_factor, base.duty_factor),
            t.duty_factor.unwrap_or(0.0),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lim() -> Limits {
        Limits {
            max_vx: 0.4,
            max_vy: 0.2,
            max_wz: 0.8,
            height_range: 0.08,
            attitude_max: 0.20,
            base_tune: [base_tune(0.85), base_tune(0.75), base_tune(0.5)],
            wbc_initial: WbcRequest::Off,
            controller_initial: GaitControllerRequest::Champ,
        }
    }

    fn shared() -> Shared {
        Shared {
            intent: Intent { gait_tune: lim().base_of(GaitSelect::Crawl), ..Intent::default() },
            quit: false,
            last_key: None,
            last_dir_key: None,
            deadman_tripped: false,
        }
    }

    /// **タップは 1 段、押し続け（リピート）は生存確認。**
    #[test]
    fn a_held_key_keeps_the_speed_instead_of_ramping_it() {
        let l = lim();
        let mut s = shared();
        let t0 = Instant::now();
        on_key(&mut s, Key::Forward, t0, &l);
        let one_step = s.intent.velocity.vx_m_s;
        assert!(one_step > 0.0);
        // 30 ms 間隔のリピートが 10 回来ても速度は変わらない。
        for i in 1..=10 {
            on_key(&mut s, Key::Forward, t0 + Duration::from_millis(30 * i), &l);
        }
        assert_eq!(s.intent.velocity.vx_m_s, one_step);
        // 300 ms 空けて叩き直せば 1 段足す。
        on_key(&mut s, Key::Forward, t0 + Duration::from_millis(700), &l);
        assert!(s.intent.velocity.vx_m_s > one_step);
    }

    /// **デッドマンはキーが途切れたら速度だけ 0 にし、モードは触らない。**
    #[test]
    fn the_deadman_zeroes_the_velocity_when_keys_stop() {
        let l = lim();
        let mut s = shared();
        let t0 = Instant::now();
        on_key(&mut s, Key::Mode(ModeRequest::Walk), t0, &l);
        on_key(&mut s, Key::Forward, t0, &l);
        let d = Duration::from_millis(500);
        assert!(!deadman_check(&mut s, t0 + Duration::from_millis(400), d));
        assert!(s.intent.velocity.vx_m_s > 0.0);
        assert!(deadman_check(&mut s, t0 + Duration::from_millis(600), d), "1 回目は true（ログ用）");
        assert_eq!(s.intent.velocity, Velocity::ZERO);
        assert_eq!(s.intent.mode, ModeRequest::Walk, "モードは変えない");
        assert!(!deadman_check(&mut s, t0 + Duration::from_millis(700), d), "2 回目は黙る");
        // 次のタップで復帰する（0 から 1 段）。
        on_key(&mut s, Key::Forward, t0 + Duration::from_secs(2), &l);
        assert!(s.intent.velocity.vx_m_s > 0.0);
    }

    /// `o` は OFF → 位置 → トルク → OFF と巡回し、`p` は MPC ↔ CHAMP。
    /// 最初の 1 押しは起動時の設定から数える。
    #[test]
    fn o_cycles_the_wbc_output_and_p_toggles_the_controller() {
        let l = lim();
        let mut i = Intent::default();
        assert_eq!(i.wbc, None);
        apply(&mut i, Key::WbcCycle, &l);
        assert_eq!(i.wbc, Some(WbcRequest::Position));
        apply(&mut i, Key::WbcCycle, &l);
        assert_eq!(i.wbc, Some(WbcRequest::Torque));
        apply(&mut i, Key::WbcCycle, &l);
        assert_eq!(i.wbc, Some(WbcRequest::Off));
        apply(&mut i, Key::ControllerToggle, &l);
        assert_eq!(i.gait_controller, Some(GaitControllerRequest::Mpc));
        apply(&mut i, Key::ControllerToggle, &l);
        assert_eq!(i.gait_controller, Some(GaitControllerRequest::Champ));
        assert_eq!(decode(b'o'), Some(Key::WbcCycle));
        assert_eq!(decode(b'p'), Some(Key::ControllerToggle));
    }

    /// 試験用の基準値。接地比だけ歩容らしく変えてある。
    fn base_tune(duty: f64) -> GaitTune {
        GaitTune {
            cycle_period_s: Some(0.60),
            swing_height_m: Some(0.05),
            step_length_m: Some(0.15),
            duty_factor: Some(duty),
        }
    }

    fn intent() -> Intent {
        Intent {
            aux_rad: vec![None],
            link_ok: true,
            ..Intent::default()
        }
    }

    /// **1 回目の押下は基準値から動く。** 0 から動くと歩容が跳ぶ。
    #[test]
    fn the_first_press_moves_from_the_profile_value() {
        let (l, mut i) = (lim(), intent());
        i.gait_tune = l.base_of(i.gait);
        apply(&mut i, Key::Tune(Knob::Cycle, -1), &l);
        assert!((i.gait_tune.cycle_period_s.unwrap() - 0.55).abs() < 1e-9);
        apply(&mut i, Key::Tune(Knob::Swing, 1), &l);
        assert!((i.gait_tune.swing_height_m.unwrap() - 0.055).abs() < 1e-9);
    }

    /// **上書きが空でも、基準値から動く。**
    #[test]
    fn an_empty_tune_still_starts_from_the_base() {
        let (l, mut i) = (lim(), intent());
        assert!(i.gait_tune.is_empty());
        apply(&mut i, Key::Tune(Knob::Cycle, 1), &l);
        assert!((i.gait_tune.cycle_period_s.unwrap() - 0.65).abs() < 1e-9);
    }

    /// 範囲で頭打ちになり、跨がない。
    #[test]
    fn tuning_stops_at_the_range_ends() {
        let (l, mut i) = (lim(), intent());
        for _ in 0..200 {
            apply(&mut i, Key::Tune(Knob::Duty, -1), &l);
            apply(&mut i, Key::Tune(Knob::Swing, -1), &l);
        }
        assert_eq!(i.gait_tune.duty_factor, Some(GaitTune::DUTY.0));
        assert_eq!(i.gait_tune.swing_height_m, Some(GaitTune::SWING_M.0));
        for _ in 0..200 {
            apply(&mut i, Key::Tune(Knob::Cycle, 1), &l);
        }
        assert_eq!(i.gait_tune.cycle_period_s, Some(GaitTune::CYCLE_S.1));
    }

    /// **歩容を替えたら、その歩容の基準値へ戻る。** 周期の基準が歩容ごとに
    /// 違うので、trot で詰めた値を crawl へ持ち込ませない。
    #[test]
    fn switching_gait_resets_the_tune_to_that_gaits_base() {
        let (l, mut i) = (lim(), intent());
        apply(&mut i, Key::Tune(Knob::Duty, -1), &l);
        assert_ne!(i.gait_tune.duty_factor, l.base_of(i.gait).duty_factor);
        apply(&mut i, Key::Gait(GaitSelect::Walk), &l);
        assert_eq!(i.gait_tune, l.base_of(GaitSelect::Walk));
        assert_eq!(i.gait_tune.duty_factor, Some(0.75));
    }

    /// m は基準値へ戻す。**速度や姿勢は触らない。**
    #[test]
    fn tune_reset_leaves_the_rest_alone() {
        let (l, mut i) = (lim(), intent());
        apply(&mut i, Key::Forward, &l);
        apply(&mut i, Key::Tune(Knob::Cycle, -1), &l);
        let v = i.velocity;
        apply(&mut i, Key::TuneReset, &l);
        assert_eq!(i.gait_tune, l.base_of(i.gait));
        assert_eq!(i.velocity, v);
    }

    /// **押すたびに 1 段。** 上限で頭打ちになり、跨がない。
    #[test]
    fn pressing_forward_steps_up_and_stops_at_the_limit() {
        let (l, mut i) = (lim(), intent());
        for _ in 0..STEPS {
            apply(&mut i, Key::Forward, &l);
        }
        assert!((i.velocity.vx_m_s - l.max_vx).abs() < 1e-12);
        apply(&mut i, Key::Forward, &l);
        assert!((i.velocity.vx_m_s - l.max_vx).abs() < 1e-12, "上限を跨いだ");
    }

    /// **スペースで止まるのは速度だけ。**
    ///
    /// 慌てて叩いたときにモードまで戻すと、立っている機体が脱力しかねない。
    #[test]
    fn stop_zeroes_the_velocity_but_leaves_the_mode_and_attitude() {
        let (l, mut i) = (lim(), intent());
        apply(&mut i, Key::Mode(ModeRequest::Walk), &l);
        apply(&mut i, Key::Forward, &l);
        apply(&mut i, Key::TiltUp, &l);
        apply(&mut i, Key::Stop, &l);
        assert!(i.velocity.is_zero());
        assert_eq!(i.mode, ModeRequest::Walk, "モードまで戻している");
        assert!(i.body_attitude_rad[1] > 0.0, "姿勢まで戻している");
    }

    /// **胴体姿勢の上限は合成量。** 軸ごとに丸めると斜めで可動域を食う
    /// （ROS 2 のサービスと同じ扱い）。
    #[test]
    fn the_attitude_is_clamped_by_its_magnitude() {
        let (l, mut i) = (lim(), intent());
        for _ in 0..STEPS * 2 {
            apply(&mut i, Key::TiltUp, &l);
            apply(&mut i, Key::TiltRight, &l);
        }
        let [r, p, _] = i.body_attitude_rad;
        let n = (r * r + p * p).sqrt();
        assert!(n <= l.attitude_max + 1e-9, "合成量 {n:.4} が上限を超えた");
        assert!(n > l.attitude_max * 0.9, "斜めで効かなくなっている");
    }

    /// **大文字も受ける。** Shift が掛かったまま打って効かないのは事故のもと。
    #[test]
    fn upper_case_works_too() {
        assert_eq!(decode(b'W'), Some(Key::Forward));
        assert_eq!(decode(b'w'), Some(Key::Forward));
        assert_eq!(decode(b'2'), Some(Key::Mode(ModeRequest::Walk)));
    }

    /// **Ctrl-C と Esc は自分で拾う。** raw モードでは端末が SIGINT を
    /// 出さないので、見落とすと止められなくなる。
    #[test]
    fn ctrl_c_and_escape_are_quit() {
        assert_eq!(decode(0x03), Some(Key::Quit));
        assert_eq!(decode(0x1b), Some(Key::Quit));
    }

    #[test]
    fn unknown_keys_do_nothing() {
        assert_eq!(decode(b'@'), None);
    }
}
