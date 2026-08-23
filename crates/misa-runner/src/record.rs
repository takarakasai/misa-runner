//! 記録をファイルへ落とす。
//!
//! # 制御ループを絶対に待たせない
//!
//! 書き込みはディスクに触るので、制御ループの中でやると周期にジッタが乗る。
//! 脚バスと同じ方針で、**ループは有界のチャネルへ渡すだけ**、実際の書き込みは
//! 別スレッドが自分のペースでやる。
//!
//! チャネルが詰まったら**捨てて数える**。待たない。記録は診断のためのもので、
//! そのために制御周期を落とすのは本末転倒。捨てた数は終了時に出るので、
//! 「取りこぼしのある記録」と「完全な記録」を取り違えることはない。
//!
//! # 書式
//!
//! `[u32 LE 長さ][postcard]` の繰り返し。先頭の 1 個が [`Header`]、以降が
//! [`Frame`]。長さを前置するのは、途中で電源が落ちた記録でも読めるところまで
//! 読めるようにするため。
//!
//! JSON にしなかったのは大きさの問題で、200 Hz × 13 軸だと 1 分で数十 MB に
//! なる。SBC に置いて常用できる大きさではない。

use std::fs::File;
use std::io::{BufReader, BufWriter, Read, Write};
use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, SyncSender, TrySendError};
use std::sync::Arc;
use std::thread::JoinHandle;

use misa_core::record::{Frame, Header};

/// チャネルに溜められる周期の数。200 Hz なら約 20 秒ぶん。
///
/// ディスクが一時的に詰まっても取りこぼさない程度に深く、詰まり続けたときに
/// メモリを食い潰さない程度に浅く。
const QUEUE_DEPTH: usize = 4096;

/// 1 フレームの上限 [byte]。壊れた記録を読んで巨大な確保をしないための番人。
const MAX_FRAME_BYTES: u32 = 4 << 20;

/// 記録の書き手。
pub struct Recorder {
    tx: Option<SyncSender<Frame>>,
    dropped: Arc<AtomicU64>,
    writer: Option<JoinHandle<Result<u64, String>>>,
    path: String,
}

impl Recorder {
    /// ファイルを作り、書き込みスレッドを起こす。
    pub fn create(path: &str, header: &Header) -> Result<Self, String> {
        let file = File::create(path).map_err(|e| format!("{path} を作れません: {e}"))?;
        let mut out = BufWriter::new(file);
        write_chunk(&mut out, header).map_err(|e| format!("{path} の見出しを書けません: {e}"))?;

        let (tx, rx) = sync_channel::<Frame>(QUEUE_DEPTH);
        let writer = std::thread::Builder::new()
            .name("record".into())
            .spawn(move || -> Result<u64, String> {
                let mut n = 0u64;
                for frame in rx {
                    write_chunk(&mut out, &frame).map_err(|e| format!("記録を書けません: {e}"))?;
                    n += 1;
                }
                out.flush().map_err(|e| format!("記録を閉じられません: {e}"))?;
                Ok(n)
            })
            .map_err(|e| format!("記録スレッドを起こせません: {e}"))?;

        Ok(Self {
            tx: Some(tx),
            dropped: Arc::new(AtomicU64::new(0)),
            writer: Some(writer),
            path: path.to_string(),
        })
    }

    /// 1 周期ぶんを渡す。**待たない。**
    pub fn push(&self, frame: Frame) {
        let Some(tx) = self.tx.as_ref() else { return };
        match tx.try_send(frame) {
            Ok(()) => {}
            // 詰まっている / 書き手が死んだ。どちらも捨てて数えるだけ。
            Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) => {
                self.dropped.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    /// 取りこぼした周期の数。
    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    /// 書き手を畳んで、書けた周期の数を返す。
    pub fn finish(mut self) -> Result<u64, String> {
        self.close()
    }

    fn close(&mut self) -> Result<u64, String> {
        drop(self.tx.take());
        match self.writer.take() {
            Some(h) => h.join().map_err(|_| "記録スレッドが異常終了しました".to_string())?,
            None => Ok(0),
        }
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        // `finish` を呼ばずに落ちた場合でも、書き終わってからファイルを閉じる。
        if let Err(e) = self.close() {
            log::warn!("{}: {e}", self.path);
        }
    }
}

fn write_chunk<T: serde::Serialize>(out: &mut impl Write, value: &T) -> std::io::Result<()> {
    let bytes = postcard::to_allocvec(value)
        .map_err(|e| std::io::Error::other(format!("符号化に失敗: {e}")))?;
    out.write_all(&(bytes.len() as u32).to_le_bytes())?;
    out.write_all(&bytes)
}

fn read_chunk<T: serde::de::DeserializeOwned>(
    input: &mut impl Read,
) -> Result<Option<T>, String> {
    let mut len = [0u8; 4];
    match input.read_exact(&mut len) {
        Ok(()) => {}
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(format!("記録を読めません: {e}")),
    }
    let len = u32::from_le_bytes(len);
    if len > MAX_FRAME_BYTES {
        return Err(format!("フレームが大きすぎます ({len} byte)。記録が壊れています"));
    }
    let mut buf = vec![0u8; len as usize];
    match input.read_exact(&mut buf) {
        Ok(()) => {}
        // **途中で切れた記録は、切れる前まで読めれば十分。** 電源が落ちた
        // 回のログこそ見たいので、末尾が欠けているだけで全部を捨てない。
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(format!("記録を読めません: {e}")),
    }
    postcard::from_bytes(&buf)
        .map(Some)
        .map_err(|e| format!("記録を復号できません: {e}"))
}

/// 記録を丸ごと読む。
pub fn read(path: impl AsRef<Path>) -> Result<(Header, Vec<Frame>), String> {
    let path = path.as_ref();
    let file = File::open(path).map_err(|e| format!("{} を開けません: {e}", path.display()))?;
    let mut input = BufReader::new(file);

    let header: Header = read_chunk(&mut input)?
        .ok_or_else(|| format!("{} が空です", path.display()))?;
    if header.format_version != misa_core::record::FORMAT_VERSION {
        return Err(format!(
            "{} は書式 v{} です（このビルドは v{}）",
            path.display(),
            header.format_version,
            misa_core::record::FORMAT_VERSION
        ));
    }
    let mut frames = Vec::new();
    while let Some(f) = read_chunk::<Frame>(&mut input)? {
        frames.push(f);
    }
    Ok((header, frames))
}

#[cfg(test)]
mod tests {
    use super::*;
    use misa_core::{AxisCommand, AxisId, Command, Intent, Observation, SafetyVerdict, Time};

    fn header() -> Header {
        Header {
            format_version: misa_core::record::FORMAT_VERSION,
            robot: "namiashi".into(),
            axes: vec!["FL_hip_joint".into(), "FL_thigh_joint".into()],
            rate_hz: 200.0,
        }
    }

    fn frame(seq: u64, q: f64) -> Frame {
        let mut command = Command::idle(2);
        *command.get_mut(AxisId::new(0)).unwrap() = AxisCommand::position(q, 8.0);
        Frame {
            seq,
            time: Time::from_secs_f64(seq as f64 * 0.005),
            intent: Intent::default(),
            observation: Observation::empty(2, 1),
            command,
            verdict: SafetyVerdict::default(),
        }
    }

    fn tmp(name: &str) -> String {
        let mut p = std::env::temp_dir();
        p.push(format!("misa-record-test-{}-{name}.bin", std::process::id()));
        p.to_string_lossy().into_owned()
    }

    #[test]
    fn what_is_written_is_what_comes_back() {
        let path = tmp("roundtrip");
        let rec = Recorder::create(&path, &header()).unwrap();
        for i in 0..64 {
            rec.push(frame(i, i as f64 * 0.01));
        }
        assert_eq!(rec.finish().unwrap(), 64);

        let (h, frames) = read(&path).unwrap();
        assert_eq!(h, header());
        assert_eq!(frames.len(), 64);
        assert_eq!(frames[7], frame(7, 0.07));
        let _ = std::fs::remove_file(&path);
    }

    /// **末尾が欠けた記録も、欠ける前まで読めること。**
    ///
    /// 電源が落ちた回のログこそ見たいので、途中で切れているだけで全部を
    /// 捨ててはいけない。
    #[test]
    fn a_truncated_recording_still_reads_up_to_where_it_was_cut() {
        let path = tmp("truncated");
        let rec = Recorder::create(&path, &header()).unwrap();
        for i in 0..32 {
            rec.push(frame(i, 0.5));
        }
        rec.finish().unwrap();

        let full = std::fs::read(&path).unwrap();
        std::fs::write(&path, &full[..full.len() - 7]).unwrap();

        let (_, frames) = read(&path).unwrap();
        assert_eq!(frames.len(), 31);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn a_recording_from_another_format_version_is_refused() {
        let path = tmp("version");
        let mut bad = header();
        bad.format_version = misa_core::record::FORMAT_VERSION + 1;
        Recorder::create(&path, &bad).unwrap().finish().unwrap();

        let e = read(&path).unwrap_err();
        assert!(e.contains("書式"), "{e}");
        let _ = std::fs::remove_file(&path);
    }
}
