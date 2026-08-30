//! 脚ロボット制御の共通語彙。
//!
//! この crate は **1 周期を関数として書くための型だけ**を持つ。
//!
//! ```text
//! Policy: (Intent, Observation, State) -> (Command, State)
//! ```
//!
//! I/O なし、時計なし、スレッドなし。同じ入力列に対して必ず同じ出力列を返す
//! 形にしておくと、副産物が 3 つ付いてくる。
//!
//! - **記録した観測を流し直して指令を差分できる。** 1 bit も変わらなければ、
//!   その改修は挙動を変えていない。危ない書き換えを検証する手段になる。
//! - **シミュレータと実機の等価性が「検査できる性質」になる。** 願望ではなく。
//! - **CI に載る。** 実機も MuJoCo も無しに状態機械を回せる。
//!
//! # 依存を薄く保つこと
//!
//! 依存は serde だけ。ロボットのモデルも、歩容も、バスのドライバも入れない。
//! ここが薄いことが上の 3 つの根拠なので、依存を足すときは
//! 「実機なしでテストできるか」「別のロボットが同じ型を使えるか」を
//! 壊していないか確かめること。
//!
//! # いまどこまで入っているか
//!
//! 語彙（この crate）→ SafetyGate → 記録の背骨 → `Plant` の抽出、という順で
//! 進める。**記録を `Plant` より先に入れる**のは、いちばん危ない書き換えを
//! 再生差分で検証できるようにするため。
//!
//! | 段 | 状態 |
//! |---|---|
//! | 語彙: [`Time`] / [`Observation`] / [`Command`] / [`Intent`] | ここ |
//! | [`SafetyGate`]（指令を書き換えてよい唯一の場所） | 部品はここ。配線は記録の後 |
//! | 記録と再生: [`Frame`] / [`diff_commands`] | ここ |
//! | [`Plant`]（全軸・1 tick の `exchange`） | ここ |
//!
//! いまはまだ制御ループがこの型を使っていない。実装との橋渡しは
//! `misa-runner` 側の変換にあり、そこがこの語彙で実機の状態を表せることの
//! 証拠になっている。

pub mod axis;
pub mod command;
pub mod intent;
pub mod observation;
pub mod plant;
pub mod record;
pub mod safety;
pub mod time;

pub use axis::{Axis, AxisId, AxisRole, AxisTable};
pub use command::{AxisCommand, Command, ControlMode};
pub use intent::{GaitSelect, GaitTune, Intent, ModeRequest, Pilot, PoseSlot, Velocity};
pub use observation::{AxisHealth, AxisState, Contact, Imu, Observation};
pub use plant::{Plant, PlantCaps};
pub use record::{diff_commands, Divergence, Frame, Header, FORMAT_VERSION};
pub use safety::{AxisLimits, SafetyConfig, SafetyGate, SafetyVerdict};
pub use time::Time;
