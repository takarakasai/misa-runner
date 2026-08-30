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
//! # 端末の設定を必ず戻す
//!
//! raw モードにしたまま落ちると、そのシェルはエコーも改行も効かなくなる
//! （`reset` を打つまで）。[`RawMode`] が `Drop` で戻すのに加えて、
//! Ctrl-C でも戻るように `poll` が終了要求を見ている。

use std::io::Read;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use misa_core::{GaitSelect, Intent, ModeRequest, Pilot, Time, Velocity};

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
    Help,
    Quit,
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
}

impl Limits {
    pub fn from(cfg: &AppConfig) -> Self {
        Self {
            max_vx: cfg.gait.max_vx_m_s,
            max_vy: cfg.gait.max_vy_m_s,
            max_wz: cfg.gait.max_wz_rad_s,
            height_range: cfg.gait.height_range_m,
            attitude_max: cfg.gait.body_attitude_max_rad,
        }
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
        Key::Gait(g) => intent.gait = g,
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
        Key::Help | Key::Quit => {}
    }
}

/// 操作の一覧。**起動時に 1 回出す。**
pub fn help(lim: &Limits) -> String {
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
         　  h / ?    この一覧    Esc / Ctrl-C    終了（脱力して抜けます）\n",
        lim.max_vx,
        lim.max_vx / STEPS as f64,
        lim.max_vy,
        lim.max_wz,
        lim.height_range,
        lim.attitude_max,
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

/// 共有される操縦の意図。
struct Shared {
    intent: Intent,
    quit: bool,
    last_key: Option<Instant>,
}

pub struct KeyPilot {
    shared: Arc<Mutex<Shared>>,
    stop: Arc<AtomicBool>,
    limits: Limits,
    /// **持っているあいだだけ raw モード。** 落ちても `Drop` で戻る。
    _raw: RawMode,
}

impl KeyPilot {
    pub fn open(cfg: &AppConfig, gait: GaitSelect) -> Result<Self, String> {
        let limits = Limits::from(cfg);
        let raw = RawMode::enter()?;
        print!("{}", help(&limits));

        let shared = Arc::new(Mutex::new(Shared {
            intent: Intent {
                // **脱力から始める。** 起動した瞬間に立ち上がらないこと。
                mode: ModeRequest::Relax,
                gait,
                aux_rad: vec![None],
                link_ok: true,
                ..Intent::default()
            },
            quit: false,
            last_key: None,
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
                            l.last_key = Some(Instant::now());
                            match key {
                                Key::Quit => l.quit = true,
                                Key::Help => print!("\r{}", help(&limits)),
                                other => apply(&mut l.intent, other, &limits),
                            }
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
            _raw: raw,
        })
    }

    /// Esc / Ctrl-C が押されたか。**呼び出し側がループを抜ける合図。**
    pub fn quit_requested(&self) -> bool {
        self.shared
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .quit
    }
}

impl Drop for KeyPilot {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

impl Pilot for KeyPilot {
    fn poll(&mut self, _now: Time) -> Intent {
        self.shared
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .intent
            .clone()
    }

    fn status_line(&self) -> String {
        let l = self.shared.lock().unwrap_or_else(|e| e.into_inner());
        let since = l
            .last_key
            .map(|t| format!("{:.0}s前", t.elapsed().as_secs_f64()))
            .unwrap_or_else(|| "未入力".into());
        format!(
            "キー {since} v=({:+.2},{:+.2},{:+.2}) 高さ{:+.2} 傾き({:+.2},{:+.2})",
            l.intent.velocity.vx_m_s,
            l.intent.velocity.vy_m_s,
            l.intent.velocity.wz_rad_s,
            l.intent.height_offset_m,
            l.intent.body_attitude_rad[0],
            l.intent.body_attitude_rad[1],
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
        }
    }

    fn intent() -> Intent {
        Intent {
            aux_rad: vec![None],
            link_ok: true,
            ..Intent::default()
        }
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
