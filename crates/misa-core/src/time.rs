//! 単調時刻。
//!
//! **時計は Plant が供給する。** 実機はハードウェアの時計、シミュレータは
//! シム時刻、ログ再生はログのタイムスタンプ。制御則の側で
//! `Instant::now()` を呼ばないことが、記録した観測を流し直したときに
//! 同じ指令が出ること（＝再生差分でリファクタを検証できること）の前提になる。
//!
//! [`std::time::Instant`] を使わないのは 2 つ理由がある。シリアライズできない
//! こと、そして「実時間から取るしかない」ことがシムと再生を弾いてしまうこと。

use core::ops::{Add, Sub};
use core::time::Duration;

use serde::{Deserialize, Serialize};

/// ある基準点からの単調経過時間。
///
/// 基準点が何かは Plant が決める（実機なら起動時、シムなら 0、再生なら
/// ログの先頭）。**絶対時刻ではない**ので、2 つの Plant をまたいで比べては
/// いけない。比べてよいのは同じ実行のなかの 2 点だけ。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default, Serialize, Deserialize)]
pub struct Time {
    nanos: u64,
}

impl Time {
    pub const ZERO: Time = Time { nanos: 0 };

    pub const fn from_nanos(nanos: u64) -> Self {
        Self { nanos }
    }

    pub const fn as_nanos(self) -> u64 {
        self.nanos
    }

    pub fn from_secs_f64(secs: f64) -> Self {
        Self {
            nanos: (secs.max(0.0) * 1e9) as u64,
        }
    }

    pub fn as_secs_f64(self) -> f64 {
        self.nanos as f64 * 1e-9
    }

    /// `self` から `earlier` までの経過。**逆順なら 0** を返す。
    ///
    /// 引き算で下に回り込むと、古さの判定が「桁外れに新しい」に化けて
    /// フェイルセーフを黙って素通りさせる。飽和させるのはそのため。
    pub fn since(self, earlier: Time) -> Duration {
        Duration::from_nanos(self.nanos.saturating_sub(earlier.nanos))
    }
}

impl Add<Duration> for Time {
    type Output = Time;
    fn add(self, d: Duration) -> Time {
        Time {
            nanos: self.nanos.saturating_add(d.as_nanos() as u64),
        }
    }
}

impl Sub<Time> for Time {
    type Output = Duration;
    fn sub(self, earlier: Time) -> Duration {
        self.since(earlier)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn elapsed_never_wraps_backwards() {
        let t0 = Time::from_secs_f64(1.0);
        let t1 = Time::from_secs_f64(2.0);
        assert_eq!(t1.since(t0), Duration::from_secs(1));
        // **逆順でも 0。** ここが飽和しないと、古い観測が「新しい」と読まれる。
        assert_eq!(t0.since(t1), Duration::ZERO);
    }

    #[test]
    fn seconds_round_trip() {
        let t = Time::from_secs_f64(1.25);
        assert!((t.as_secs_f64() - 1.25).abs() < 1e-9);
    }
}
