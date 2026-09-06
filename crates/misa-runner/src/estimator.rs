//! 接地足から胴体の状態を測る（脚オドメトリ）。
//!
//! # 何を仮定しているか
//!
//! **「接地していると歩容が言っている足は、世界に対して止まっている」**
//! — これだけ。足裏センサも外部計測も無いので、ほかに手がかりが無い。
//! 滑れば外れるが、滑っているならそもそも歩容が成立していない
//! （`sim` の「接地中の足の滑り」がその指標）。
//!
//! # 何が出て、何が出ないか
//!
//! ```text
//!   出る   胴体の並進速度（世界座標）
//!          接地足からの高さ
//!          歩容の計画に対する胴体位置の誤差
//!   出ない 世界座標の絶対位置（積分するしかなく、滑りのぶんが溜まる）
//! ```
//!
//! **転ぶかどうかに効くのは支持足に対する相対位置だけ**なので、絶対位置が
//! 出せなくても制御は書ける。
//!
//! # 歩容の運動学を使う理由
//!
//! [`quadruped_gait::forward_leg_kinematics`] と
//! [`quadruped_gait::foot_jacobian_body`] で足りる（3 リンクの脚を解くだけ）。
//! misarta の全身モデルでも同じことはできて、[`crate::wbc`] は接触ヤコビアン
//! を既に持っているのでそちらでも書けるが、**WBC を無効にしたまま MPC 歩容
//! だけ使う構成があり得る**ので、こちらを唯一の出どころにしている。
//! 推定が 2 か所にあると、食い違ったときにどちらが正しいか誰も言えない。

use nalgebra as na;

use legged_estimation::{LinearKalmanEstimator, LinearKalmanInputs};
use quadruped_gait::{foot_jacobian_body, forward_leg_kinematics, KinematicsConfig};

use crate::config::EstimatorKind;
use crate::jointvec::JointVec;
use crate::robot::Robot;

/// 重力 [m/s²]。
const G: f64 = 9.806_65;

/// 接地足から測った胴体の状態。**接地足が 1 本も無ければ全部 `None`。**
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct BodyState {
    /// 接地足からの胴体高さ [m]。
    pub height_m: Option<f64>,
    /// **歩容の計画に対する胴体位置の誤差** [m]（世界座標系）。
    ///
    /// 足が世界に対して止まっているなら
    /// `胴体_目標 − 胴体_実際 = 足_実測 − 足_計画`（どちらも胴体基準）。
    /// z 成分は高さの誤差そのもの。
    pub position_error_world: Option<na::Vector3<f64>>,
    /// 胴体の並進速度 [m/s]（世界座標系）。
    pub velocity_world: Option<na::Vector3<f64>>,
    /// 胴体の角速度 [rad/s]（世界座標系）。**IMU のジャイロを回しただけ**
    /// なので接地足が無くても出る。
    pub angular_velocity_world: na::Vector3<f64>,
    /// 立脚と見なした脚の数。
    pub stance_count: usize,
}

/// 脚オドメトリ（＋任意で LKF）。
///
/// 脚オドメトリは**状態を持たない**（毎周期の観測だけで決まる）。
/// [`EstimatorKind::Kalman`] のときだけ `kalman` に legged_control の
/// 18 状態 LKF を持ち、高さと速度をそちらから出す。**計画に対する位置誤差
/// は常に脚オドメトリ**（LKF は計画を知らない）。
#[derive(Debug, Clone)]
pub struct BodyEstimator {
    kin: KinematicsConfig,
    /// IK 出力 → モデル符号の変換表。`q_model = q_ik * sign` なので、
    /// 逆向きも同じ式（`sign` は ±1）。
    signs: [[f64; 3]; 4],
    kalman: Option<LinearKalmanEstimator>,
    /// LKF を脚オドメトリの高さで一度張ったか。**張らないと胴体が原点
    /// （z = 0 = 地面）から始まり、収束するまで高さが嘘になる。**
    kalman_seeded: bool,
}

impl BodyEstimator {
    pub fn new(robot: &Robot, kind: EstimatorKind) -> Self {
        Self {
            kin: robot.kin.clone(),
            signs: robot.signs,
            kalman: match kind {
                EstimatorKind::LegOdometry => None,
                EstimatorKind::Kalman => Some(LinearKalmanEstimator::new()),
            },
            kalman_seeded: false,
        }
    }

    /// 歩容が止まったら LKF を捨てる。**立ち上がり・伏せの間は足が世界に
    /// 対して止まっていない**ので、そこで積んだ状態は次の歩容の邪魔になる。
    pub fn reset(&mut self) {
        self.kalman_seeded = false;
    }

    /// 1 周期ぶん。`gyro_body` は IMU の角速度、`accel_body` は加速度計
    /// （**重力込み・胴体座標系**、無ければ `None`）。どちらも胴体座標系。
    pub fn estimate(
        &mut self,
        measured_q: &JointVec,
        measured_qd: &JointVec,
        target_q: &JointVec,
        attitude_rad: [f64; 3],
        gyro_body: [f64; 3],
        accel_body: Option<[f64; 3]>,
        stance: [bool; 4],
        dt: f64,
    ) -> BodyState {
        let [roll, pitch, yaw] = attitude_rad;
        let r_wb = na::Rotation3::from_euler_angles(roll, pitch, yaw);
        let omega_body = na::Vector3::new(gyro_body[0], gyro_body[1], gyro_body[2]);

        let mut out = BodyState {
            angular_velocity_world: r_wb * omega_body,
            stance_count: stance.iter().filter(|s| **s).count(),
            ..BodyState::default()
        };
        let odom = self.leg_odometry(measured_q, measured_qd, target_q, &r_wb, &omega_body, stance, &mut out);

        if let Some(kf) = self.kalman.as_mut() {
            // 足 4 本ぶんの運動学（遊脚も入れる。LKF が共分散で重みを落とす）。
            let mut p_world = [na::Vector3::zeros(); 4];
            let mut v_world = [na::Vector3::zeros(); 4];
            for slot in 0..4 {
                let kin = self.kin.legs()[slot];
                let s = self.signs[slot];
                let q: Vec<f64> = (0..3).map(|k| measured_q.legs[slot][k] * s[k]).collect();
                let qd = na::Vector3::new(
                    measured_qd.legs[slot][0] * s[0],
                    measured_qd.legs[slot][1] * s[1],
                    measured_qd.legs[slot][2] * s[2],
                );
                let p = forward_leg_kinematics(kin, q[0], q[1], q[2]);
                let j = foot_jacobian_body(kin, q[0], q[1], q[2]);
                p_world[slot] = r_wb * p;
                // 足の胴体原点に対する速度（世界向き）。
                v_world[slot] = r_wb * (omega_body.cross(&p) + j * qd);
            }
            if !self.kalman_seeded {
                // 高さは脚オドメトリ、水平位置は原点。接地足が無ければ
                // 公称立ち高さで張る。
                let h = odom.unwrap_or(-self.kin.legs()[0].nominal_foot_body.z);
                let body = na::Vector3::new(0.0, 0.0, h);
                let feet: [na::Vector3<f64>; 4] = std::array::from_fn(|i| body + p_world[i]);
                kf.reset(body, &feet);
                self.kalman_seeded = true;
            }
            // 重力を抜いて世界座標へ。加速度計が無い Plant では 0（＝等速予測）。
            let accel_world = accel_body
                .map(|a| r_wb * na::Vector3::new(a[0], a[1], a[2]) - na::Vector3::new(0.0, 0.0, G))
                .unwrap_or_else(na::Vector3::zeros);
            let est = kf.update(&LinearKalmanInputs {
                dt,
                accel_world,
                foot_pos_world_offset: &p_world,
                foot_vel_world: &v_world,
                contact_flag: stance,
            });
            out.velocity_world = Some(est.body_vel_world);
            out.height_m = Some(est.body_pos_world.z);
        }
        out
    }

    /// 脚オドメトリ。`out` の高さ・速度・位置誤差を埋め、接地足からの高さを
    /// 返す（接地足が無ければ何も埋めず `None`）。
    fn leg_odometry(
        &self,
        measured_q: &JointVec,
        measured_qd: &JointVec,
        target_q: &JointVec,
        r_wb: &na::Rotation3<f64>,
        omega_body: &na::Vector3<f64>,
        stance: [bool; 4],
        out: &mut BodyState,
    ) -> Option<f64> {
        if out.stance_count == 0 {
            return None;
        }

        let mut foot_meas = na::Vector3::zeros();
        let mut foot_plan = na::Vector3::zeros();
        let mut v_body = na::Vector3::zeros();
        for slot in 0..4 {
            if !stance[slot] {
                continue;
            }
            let kin = self.kin.legs()[slot];
            let s = self.signs[slot];
            let q: Vec<f64> = (0..3).map(|k| measured_q.legs[slot][k] * s[k]).collect();
            let qd = na::Vector3::new(
                measured_qd.legs[slot][0] * s[0],
                measured_qd.legs[slot][1] * s[1],
                measured_qd.legs[slot][2] * s[2],
            );
            let p_meas = forward_leg_kinematics(kin, q[0], q[1], q[2]);
            let p_plan = forward_leg_kinematics(
                kin,
                target_q.legs[slot][0] * s[0],
                target_q.legs[slot][1] * s[1],
                target_q.legs[slot][2] * s[2],
            );
            // 足の世界速度が 0：`v_胴体 + ω × p + J·q̇ = 0`（すべて胴体座標）。
            let j = foot_jacobian_body(kin, q[0], q[1], q[2]);
            v_body -= omega_body.cross(&p_meas) + j * qd;
            foot_meas += p_meas;
            foot_plan += p_plan;
        }
        let n = out.stance_count as f64;
        // **姿勢で回してから平均する必要はない**（回転は線形）。
        let foot_meas_world = r_wb * (foot_meas / n);
        // 計画は水平姿勢のまま比べる。**歩容は水平計画**なので、傾いたぶんは
        // 姿勢のタスク（`a_base_des` の角成分）が別に見る。
        let foot_plan_world = foot_plan / n;

        out.velocity_world = Some(r_wb * (v_body / n));
        out.height_m = Some(-foot_meas_world.z);
        out.position_error_world = Some(foot_meas_world - foot_plan_world);
        out.height_m
    }
}

/// 歩容が計画した立脚フラグを、**実測の接地で上書きする**。
///
/// # 早い着地だけを見る
///
/// 「遊脚の計画なのに接地している」を立脚へ倒す。逆（立脚の計画なのに
/// 離れている）は見ない — 胴体が一瞬落ちて全脚が抜けた周期に全部を遊脚へ
/// 倒すと、支えが 1 つも無い解を WBC に解かせることになり、そこから戻れない
/// （`quadruped_gait::ContactDrivenPhase` と articara の WBC 検証が同じ判断を
/// している）。
///
/// # なぜ要るか
///
/// WBC の「立脚足が滑らない」は優先度 0 の硬い制約なので、**接地していない
/// 足を接地と信じるとその瞬間に解が壊れる**。trot は支持が 2 本しかなく、
/// 着地が計画より早いことが常にあるので、そこで効く。
///
/// **接地を出せる Plant でしか効かない。** namiashi の実機は足裏センサを
/// 持たないので `None` が並び、計画がそのまま通る。
///
/// # 既定で使わない
///
/// [`misa_core::Observation::contacts`] は真偽値しか持たず、**力の閾値を
/// 置けない**。articara は 5 N を超えたときだけ倒しているが、こちらは
/// かすっただけでも倒れるので、MuJoCo の trot では悪化した（進む量
/// 9.50 → 8.43 m、ヨー 29° → 61°）。詳しくは
/// [`crate::config::WbcConfig::use_measured_contact`]。
pub fn stance_with_measured_contact(planned: [bool; 4], obs: &misa_core::Observation) -> [bool; 4] {
    let mut out = planned;
    for (slot, flag) in out.iter_mut().enumerate() {
        if obs.contacts.get(slot).copied().flatten() == Some(true) {
            *flag = true;
        }
    }
    out
}

/// **推定器のための**立脚フラグ。接地を測れる足はその値、測れない足
/// （`None`、実機）は計画。
///
/// WBC の接地拘束（[`stance_with_measured_contact`]、早い着地だけ倒す）とは
/// 別物。推定器にとって大事なのは**「本当に荷重が乗って止まっている足」だけ
/// を使うこと**で、計画が立脚なのに浮いている足を入れると、その足の速度が
/// そのまま胴体速度の誤差になる（trot 0.80 の着地・離地の周期で −0.2 m/s
/// が出ていた）。
pub fn stance_for_estimator(planned: [bool; 4], obs: &misa_core::Observation) -> [bool; 4] {
    let mut out = planned;
    for (slot, flag) in out.iter_mut().enumerate() {
        if let Some(c) = obs.contacts.get(slot).copied().flatten() {
            *flag = c;
        }
    }
    out
}

/// 観測の関節速度を [`JointVec`] へ。読めていない軸は 0。
///
/// 位置側（`runner::jointvec_from`）と対。**速度を 0 と読むのは安全側**で、
/// 実際に動いている軸を 0 と見ると WBC は減衰項を出さないだけになる
/// （逆に大きな値を捏造すると、それを打ち消す加速度を要求してしまう）。
pub fn velocities_from(obs: &misa_core::Observation) -> JointVec {
    let mut v = JointVec::zeros();
    for leg in 0..4 {
        for k in 0..3 {
            let id = misa_core::AxisId::new((leg * 3 + k) as u16);
            if let Some(a) = obs.get(id) {
                if a.health.valid {
                    v.legs[leg][k] = a.velocity_rad_s;
                }
            }
        }
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    fn robot() -> Robot {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../models/namiashi/namiashi.misa"
        );
        Robot::load(path, "extend").expect("同梱モデルが読めること")
    }

    fn stance_pose(r: &Robot) -> JointVec {
        use quadruped_gait::GaitGenerator;
        let cfg = crate::config::AppConfig::default();
        let mut g = r.build_gait(&cfg.gait, &cfg.wbc, crate::teleop::GaitSelect::Crawl);
        let out = g.tick(0.005);
        r.output_to_joints(&out, 0.0)
    }

    /// **計画どおりに立っていれば誤差は 0、高さは立ち高さ。**
    #[test]
    fn standing_on_the_plan_reports_no_error_and_the_stance_height() {
        let r = robot();
        let mut e = BodyEstimator::new(&r, EstimatorKind::LegOdometry);
        let q = stance_pose(&r);
        let s = e.estimate(&q, &JointVec::zeros(), &q, [0.0; 3], [0.0; 3], None, [true; 4], 0.005);
        assert_eq!(s.stance_count, 4);
        let err = s.position_error_world.unwrap();
        assert!(err.norm() < 1e-9, "{err:?}");
        let h = s.height_m.unwrap();
        let want = crate::config::AppConfig::default().gait.stance_height_m;
        assert!((h - want).abs() < 1e-6, "高さ {h} が立ち高さ {want} と違う");
        assert!(s.velocity_world.unwrap().norm() < 1e-9);
    }

    /// **脚を畳めば胴体は下がり、誤差の z にそのぶんが出る。**
    #[test]
    fn a_folded_leg_shows_up_as_a_height_error() {
        let r = robot();
        let mut e = BodyEstimator::new(&r, EstimatorKind::LegOdometry);
        let plan = stance_pose(&r);
        let mut q = plan;
        for leg in 0..4 {
            q.legs[leg][1] += 0.05;
            q.legs[leg][2] -= 0.10;
        }
        let s = e.estimate(&q, &JointVec::zeros(), &plan, [0.0; 3], [0.0; 3], None, [true; 4], 0.005);
        let h = s.height_m.unwrap();
        let err = s.position_error_world.unwrap();
        assert!(h < 0.2, "畳んだのに高さが {h}");
        // 誤差 z = 実測の足 z − 計画の足 z。胴体が下がった＝足が近い＝正。
        assert!(err.z > 0.0, "{err:?}");
        assert!((err.z - (0.2 - h)).abs() < 1e-6, "高さの差と z 誤差が一致しない");
    }

    /// **関節が動いていれば胴体が動いていると読む。** 4 脚とも同じ速さで
    /// 伸ばせば、胴体は真上へ動く。
    #[test]
    fn extending_every_leg_reads_as_the_body_rising() {
        let r = robot();
        let mut e = BodyEstimator::new(&r, EstimatorKind::LegOdometry);
        let q = stance_pose(&r);
        let mut qd = JointVec::zeros();
        for leg in 0..4 {
            qd.legs[leg][1] = -0.2;
            qd.legs[leg][2] = 0.4;
        }
        let s = e.estimate(&q, &qd, &q, [0.0; 3], [0.0; 3], None, [true; 4], 0.005);
        let v = s.velocity_world.unwrap();
        assert!(v.z > 0.01, "伸ばしているのに上がっていない: {v:?}");
        assert!(v.x.abs() < 1e-6 && v.y.abs() < 1e-6, "横に動いている: {v:?}");
    }

    /// **接地を出せない Plant では計画がそのまま通る。** 実機（足裏センサ
    /// 無し）で挙動が変わらないこと。
    #[test]
    fn a_plant_without_contact_sensors_leaves_the_plan_alone() {
        let obs = misa_core::Observation::empty(13, 4);
        assert!(obs.contacts.iter().all(|c| c.is_none()));
        let planned = [true, false, true, false];
        assert_eq!(stance_with_measured_contact(planned, &obs), planned);
    }

    /// **早い着地は立脚へ倒す。遅い離地は倒さない。**
    #[test]
    fn only_an_early_touchdown_overrides_the_plan() {
        let mut obs = misa_core::Observation::empty(13, 4);
        // FL: 遊脚の計画だが接地している → 立脚へ。
        obs.contacts[0] = Some(true);
        // FR: 立脚の計画だが離れている → **計画のまま**（支えを失わない）。
        obs.contacts[1] = Some(false);
        let got = stance_with_measured_contact([false, true, false, true], &obs);
        assert_eq!(got, [true, true, false, true]);
    }

    /// **接地足が無ければ何も言わない。** 「0」と「分からない」を潰さない。
    #[test]
    fn a_flight_phase_reports_nothing() {
        let r = robot();
        let mut e = BodyEstimator::new(&r, EstimatorKind::LegOdometry);
        let q = stance_pose(&r);
        let s = e.estimate(&q, &JointVec::zeros(), &q, [0.0; 3], [0.1, 0.0, 0.0], None, [false; 4], 0.005);
        assert_eq!(s.stance_count, 0);
        assert!(s.height_m.is_none());
        assert!(s.velocity_world.is_none());
        // 角速度は接地に依らない。
        assert!((s.angular_velocity_world.x - 0.1).abs() < 1e-9);
    }

    /// **角速度は世界座標へ回す。** MPC は世界座標で受け取る。
    #[test]
    fn the_gyro_is_rotated_into_the_world_frame() {
        let r = robot();
        let mut e = BodyEstimator::new(&r, EstimatorKind::LegOdometry);
        let q = stance_pose(&r);
        // ヨー 90°。胴体 x 軸まわりの回転は世界 y 軸まわりになる。
        let s = e.estimate(
            &q,
            &JointVec::zeros(),
            &q,
            [0.0, 0.0, std::f64::consts::FRAC_PI_2],
            [1.0, 0.0, 0.0],
            None,
            [true; 4],
            0.005,
        );
        let w = s.angular_velocity_world;
        assert!(w.x.abs() < 1e-9 && (w.y - 1.0).abs() < 1e-9, "{w:?}");
    }

    /// **LKF は脚オドメトリの高さで張られ、止まって立っていれば同じ答えを出す。**
    /// 加速度計は重力だけ（胴体座標で +G の z）。
    #[test]
    fn the_kalman_filter_agrees_with_leg_odometry_when_standing_still() {
        let r = robot();
        let mut e = BodyEstimator::new(&r, EstimatorKind::Kalman);
        let q = stance_pose(&r);
        let want = crate::config::AppConfig::default().gait.stance_height_m;
        let mut s = BodyState::default();
        for _ in 0..200 {
            s = e.estimate(&q, &JointVec::zeros(), &q, [0.0; 3], [0.0; 3], Some([0.0, 0.0, G]), [true; 4], 0.005);
        }
        let h = s.height_m.unwrap();
        assert!((h - want).abs() < 1e-3, "LKF の高さ {h} が立ち高さ {want} と違う");
        assert!(s.velocity_world.unwrap().norm() < 1e-3, "{:?}", s.velocity_world);
        // 計画に対する誤差は脚オドメトリのまま。
        assert!(s.position_error_world.unwrap().norm() < 1e-9);
    }
}
