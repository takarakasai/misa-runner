//! ROS 2 から操縦する。
//!
//! **速度は `cmd_vel`、それ以外はサービス。** `geometry_msgs/Twist` は速度
//! しか運べないので、モード・歩容・ポーズ再生・胴体姿勢・高さは項目ごとの
//! サービスで受ける（`ros/misa_msgs`）。
//!
//! ```text
//!  /cmd_vel              geometry_msgs/Twist       速度（best-effort, depth 1）
//!  ~/set_mode            misa_msgs/SetMode         脱力 / 初期姿勢 / 歩行
//!  ~/set_gait            misa_msgs/SetGait         Crawl / Walk / Trot
//!  ~/play_pose           misa_msgs/PlayPose        1 回だけ起こす
//!  ~/set_body_attitude   misa_msgs/SetBodyAttitude roll / pitch / yaw
//!  ~/set_height          misa_msgs/SetHeight       立ち高さの差分
//! ```
//!
//! # ROS を制御ループの中で待たない
//!
//! spin は別スレッドで回し、受信も応答もそこで完結する。制御ループは
//! [`Pilot::poll`] で共有スロットを読むだけ。脚バスと同じ形で、**ROS が
//! 詰まっても制御周期は落ちない**。
//!
//! # 「来ない」は正常
//!
//! S.BUS はフレームが途切れれば切断と分かるが、`cmd_vel` は publisher が
//! 黙っているだけの状態が正常にありうる。だから**時間切れで速度だけ 0 に
//! 落とす**（モードは変えない）。四足では、荷重がかかったまま脱力させると
//! 崩れるので、止めたいなら明示的に `set_mode` を呼ぶこと。
//!
//! # QoS
//!
//! `cmd_vel` は **best-effort・depth 1**。制御ループで信頼性を求めると、
//! 再送で遅延が跳ねて周期が乱れる。古い指令が遅れて届くほうが有害。
//! reliable な publisher から best-effort な subscriber は繋がるので、
//! `teleop_twist_keyboard` のような既定 QoS の相手とも噛み合う。

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use futures::StreamExt;
use misa_core::{GaitSelect, Intent, ModeRequest, Pilot, PoseSlot, Time, Velocity};

use crate::config::AppConfig;

/// spin スレッドと制御ループが共有する、いまの意図。
#[derive(Debug)]
struct Shared {
    velocity: Velocity,
    /// 最後に `cmd_vel` を受けた時刻。
    velocity_at: Option<Instant>,
    mode: ModeRequest,
    gait: GaitSelect,
    /// **立ち上がりを表す。** `poll` が 1 回だけ取り出して消す。
    pose_pending: Option<PoseSlot>,
    attitude_rad: [f64; 3],
    height_offset_m: f64,
}

impl Default for Shared {
    fn default() -> Self {
        Self {
            velocity: Velocity::ZERO,
            velocity_at: None,
            // **脱力から始める。** ROS が繋がった瞬間に立ち上がらないこと。
            mode: ModeRequest::Relax,
            gait: GaitSelect::Crawl,
            pose_pending: None,
            attitude_rad: [0.0; 3],
            height_offset_m: 0.0,
        }
    }
}

pub struct Ros2Pilot {
    shared: Arc<Mutex<Shared>>,
    /// `cmd_vel` がこれより古ければ速度を 0 にする。
    timeout: Duration,
    /// spin スレッドを止める合図。
    ///
    /// **止めずに落とすと落ちる。** ROS のコンテキストが片付いたあとに
    /// spin が走ると、解放済みの領域を触って malloc が壊れる（実測）。
    stop: Arc<AtomicBool>,
    spin: Option<std::thread::JoinHandle<()>>,
}

impl Ros2Pilot {
    /// ノードを立て、購読とサービスを開いて spin スレッドを起こす。
    ///
    /// `namespace` と `node_name` はプロファイルの `[hardware] kind = "ros2"`
    /// から。速度のタイムアウトも同じところ。
    pub fn connect(cfg: &AppConfig) -> Result<Self, String> {
        let (node_name, namespace, timeout_ms) = match &cfg.hardware {
            misa_hal::config::HardwareConfig::Ros2(r) => (
                r.node_name.clone(),
                r.namespace.clone(),
                r.cmd_vel_timeout_ms,
            ),
            // 実機がシリアルでも、操縦だけ ROS から入れたいことはある。
            _ => ("misa_run".to_string(), String::new(), 300),
        };

        let shared = Arc::new(Mutex::new(Shared::default()));
        let spin_shared = Arc::clone(&shared);

        // **ノードを立てる失敗は呼び出し側へ返す。** スレッドの中で落とすと、
        // 「起動したのに何も来ない」になって原因が分からない。
        let ctx = r2r::Context::create().map_err(|e| format!("ROS 2 の初期化に失敗: {e}"))?;
        let mut node = r2r::Node::create(ctx, &node_name, &namespace)
            .map_err(|e| format!("ノード {namespace}/{node_name} を作れません: {e}"))?;

        let cmd_vel = node
            .subscribe::<r2r::geometry_msgs::msg::Twist>(
                "cmd_vel",
                r2r::QosProfile::default().best_effort().keep_last(1),
            )
            .map_err(|e| format!("cmd_vel を購読できません: {e}"))?;
        let set_mode = node
            .create_service::<r2r::misa_msgs::srv::SetMode::Service>(
                "~/set_mode",
                r2r::QosProfile::default(),
            )
            .map_err(|e| format!("set_mode を開けません: {e}"))?;
        let set_gait = node
            .create_service::<r2r::misa_msgs::srv::SetGait::Service>(
                "~/set_gait",
                r2r::QosProfile::default(),
            )
            .map_err(|e| format!("set_gait を開けません: {e}"))?;
        let play_pose = node
            .create_service::<r2r::misa_msgs::srv::PlayPose::Service>(
                "~/play_pose",
                r2r::QosProfile::default(),
            )
            .map_err(|e| format!("play_pose を開けません: {e}"))?;
        let set_attitude = node
            .create_service::<r2r::misa_msgs::srv::SetBodyAttitude::Service>(
                "~/set_body_attitude",
                r2r::QosProfile::default(),
            )
            .map_err(|e| format!("set_body_attitude を開けません: {e}"))?;
        let set_height = node
            .create_service::<r2r::misa_msgs::srv::SetHeight::Service>(
                "~/set_height",
                r2r::QosProfile::default(),
            )
            .map_err(|e| format!("set_height を開けません: {e}"))?;

        log::info!(
            "ROS 2 から操縦します: {namespace}/{node_name}  cmd_vel + サービス 5 本"
        );

        let stop = Arc::new(AtomicBool::new(false));
        let spin_stop = Arc::clone(&stop);
        let spin = std::thread::Builder::new()
            .name("ros2-pilot".into())
            .spawn(move || {
                let mut pool = futures::executor::LocalPool::new();
                let sp = pool.spawner();
                spawn_cmd_vel(&sp, cmd_vel, Arc::clone(&spin_shared));
                spawn_set_mode(&sp, set_mode, Arc::clone(&spin_shared));
                spawn_set_gait(&sp, set_gait, Arc::clone(&spin_shared));
                spawn_play_pose(&sp, play_pose, Arc::clone(&spin_shared));
                spawn_set_attitude(&sp, set_attitude, Arc::clone(&spin_shared));
                spawn_set_height(&sp, set_height, Arc::clone(&spin_shared));
                while !spin_stop.load(Ordering::Relaxed) {
                    node.spin_once(Duration::from_millis(5));
                    pool.run_until_stalled();
                }
            })
            .map_err(|e| format!("ROS 2 の spin スレッドを起こせません: {e}"))?;

        Ok(Self {
            shared,
            timeout: Duration::from_millis(timeout_ms.max(20)),
            stop,
            spin: Some(spin),
        })
    }
}

impl Pilot for Ros2Pilot {
    fn status_line(&self) -> String {
        let s = self.shared.lock().unwrap_or_else(|e| e.into_inner());
        match s.velocity_at {
            Some(t) => format!("cmd_vel {:.0}ms前", t.elapsed().as_secs_f64() * 1e3),
            None => "cmd_vel 未受信".to_string(),
        }
    }

    fn poll(&mut self, now: Time) -> Intent {
        let mut s = self.shared.lock().unwrap_or_else(|e| e.into_inner());

        // **来ていない間は速度 0。モードは変えない。**
        let fresh = s
            .velocity_at
            .is_some_and(|t| t.elapsed() <= self.timeout);
        let velocity = if fresh { s.velocity } else { Velocity::ZERO };
        let slot = s.pose_pending.take();

        Intent {
            time: now,
            velocity,
            body_attitude_rad: s.attitude_rad,
            height_offset_m: s.height_offset_m,
            mode: s.mode,
            gait: s.gait,
            play_pose: slot.is_some(),
            pose_slot: slot.unwrap_or(PoseSlot(0)),
            // 胴体姿勢は set_body_attitude が直接指すので、ヘッドの
            // 追従（チキンヘッド）はここでは立てない。
            stabilize_head: false,
            // 補助軸は ROS から触らない。駆動する主体ができたらそこが持つ。
            aux_rad: Vec::new(),
            link_ok: fresh,
        }
    }
}

type Sp = futures::executor::LocalSpawner;

fn spawn_cmd_vel(
    sp: &Sp,
    mut sub: impl futures::Stream<Item = r2r::geometry_msgs::msg::Twist> + Unpin + 'static,
    shared: Arc<Mutex<Shared>>,
) {
    use futures::task::LocalSpawnExt;
    sp.spawn_local(async move {
        while let Some(t) = sub.next().await {
            let mut s = shared.lock().unwrap_or_else(|e| e.into_inner());
            s.velocity = Velocity {
                vx_m_s: t.linear.x,
                vy_m_s: t.linear.y,
                wz_rad_s: t.angular.z,
            };
            s.velocity_at = Some(Instant::now());
        }
    })
    .expect("spawn cmd_vel");
}

/// サービス 1 本ぶんの受け口を起こす。応答は `(ok, message)`。
macro_rules! spawn_service {
    ($name:ident, $srv:path, $resp:ty, |$req:ident, $s:ident| $body:block) => {
        fn $name(
            sp: &Sp,
            mut srv: impl futures::Stream<Item = r2r::ServiceRequest<$srv>> + Unpin + 'static,
            shared: Arc<Mutex<Shared>>,
        ) {
            use futures::task::LocalSpawnExt;
            sp.spawn_local(async move {
                while let Some(req) = srv.next().await {
                    // **クロージャで包む。** 値域外を `return` で断れるように
                    // するため（async ブロックから抜けてしまわない）。
                    let (ok, message) = {
                        let $req = &req.message;
                        #[allow(unused_mut)]
                        let mut $s = shared.lock().unwrap_or_else(|e| e.into_inner());
                        let mut handle = move || $body;
                        handle()
                    };
                    let mut resp = <$resp as Default>::default();
                    resp.ok = ok;
                    resp.message = message;
                    if let Err(e) = req.respond(resp) {
                        log::warn!("サービスの応答に失敗: {e}");
                    }
                }
            })
            .expect("spawn service");
        }
    };
}

spawn_service!(
    spawn_set_mode,
    r2r::misa_msgs::srv::SetMode::Service,
    r2r::misa_msgs::srv::SetMode::Response,
    |req, s| {
        match req.mode {
            0 => {
                s.mode = ModeRequest::Relax;
                (true, "Relax".into())
            }
            1 => {
                s.mode = ModeRequest::Stand;
                (true, "Stand".into())
            }
            2 => {
                s.mode = ModeRequest::Walk;
                (true, "Walk".into())
            }
            other => (false, format!("mode {other} は 0..=2 の外です")),
        }
    }
);

spawn_service!(
    spawn_set_gait,
    r2r::misa_msgs::srv::SetGait::Service,
    r2r::misa_msgs::srv::SetGait::Response,
    |req, s| {
        // **実際に切り替わるかは制御則が決める。** 遊脚があるあいだは
        // 受け付けられないので、ここで「受け取った」以上は言わない。
        let g = match req.gait {
            0 => GaitSelect::Crawl,
            1 => GaitSelect::Walk,
            2 => GaitSelect::Trot,
            other => return (false, format!("gait {other} は 0..=2 の外です")),
        };
        s.gait = g;
        (true, format!("{} を要求しました", g.label()))
    }
);

spawn_service!(
    spawn_play_pose,
    r2r::misa_msgs::srv::PlayPose::Service,
    r2r::misa_msgs::srv::PlayPose::Response,
    |req, s| {
        s.pose_pending = Some(PoseSlot(req.slot));
        (true, format!("slot {} の再生を要求しました", req.slot))
    }
);

spawn_service!(
    spawn_set_attitude,
    r2r::misa_msgs::srv::SetBodyAttitude::Service,
    r2r::misa_msgs::srv::SetBodyAttitude::Response,
    |req, s| {
        s.attitude_rad = [req.roll, req.pitch, req.yaw];
        (true, String::new())
    }
);

spawn_service!(
    spawn_set_height,
    r2r::misa_msgs::srv::SetHeight::Service,
    r2r::misa_msgs::srv::SetHeight::Response,
    |req, s| {
        s.height_offset_m = req.offset_m;
        (true, String::new())
    }
);

#[cfg(test)]
mod tests {
    use super::*;

    /// **cmd_vel が来ていない間は速度 0、モードはそのまま。**
    ///
    /// pub/sub では「来ない」が正常にありうるので、時間切れを切断と
    /// 同じには扱えない。速度だけ落とす。
    #[test]
    fn a_stale_cmd_vel_zeroes_the_velocity_but_keeps_the_mode() {
        let shared = Arc::new(Mutex::new(Shared {
            velocity: Velocity {
                vx_m_s: 0.3,
                ..Velocity::ZERO
            },
            velocity_at: Some(Instant::now() - Duration::from_secs(1)),
            mode: ModeRequest::Walk,
            ..Shared::default()
        }));
        let mut p = Ros2Pilot {
            shared,
            timeout: Duration::from_millis(200),
            stop: Arc::new(AtomicBool::new(false)),
            spin: None,
        };
        let i = p.poll(Time::ZERO);
        assert!(i.velocity.is_zero());
        assert_eq!(i.mode, ModeRequest::Walk);
        assert!(!i.link_ok);
    }

    /// **ポーズ再生は 1 回だけ。** サービスは 1 度呼ばれたら 1 度立つ。
    #[test]
    fn a_pose_request_fires_once() {
        let shared = Arc::new(Mutex::new(Shared {
            pose_pending: Some(PoseSlot(1)),
            ..Shared::default()
        }));
        let mut p = Ros2Pilot {
            shared,
            timeout: Duration::from_millis(200),
            stop: Arc::new(AtomicBool::new(false)),
            spin: None,
        };
        let first = p.poll(Time::ZERO);
        assert!(first.play_pose);
        assert_eq!(first.pose_slot, PoseSlot(1));
        assert!(!p.poll(Time::ZERO).play_pose, "2 周期続けて立っている");
    }

    /// **繋がった瞬間に立ち上がらないこと。** 既定は脱力。
    #[test]
    fn the_default_mode_is_relaxed() {
        assert_eq!(Shared::default().mode, ModeRequest::Relax);
    }
}

impl Drop for Ros2Pilot {
    /// **spin を止めてから落とす。** ROS のコンテキストが片付いたあとに
    /// spin が走ると、解放済みの領域を触って malloc が壊れる。
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.spin.take() {
            let _ = h.join();
        }
    }
}
