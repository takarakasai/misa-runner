#!/usr/bin/env python3
"""膝の反転（trot 型）の 2 脚支持の釣り合いゲイン `gait.knee_flip_balance_gains` を LQR で設計する。

モデル（支持線まわり。点接触 2 つは支持線まわりのモーメントを作れず、角運動量は
重力でしか変わらない）:

    I_line·θ̈ = m·g·(s + h·θ) − m·h·s̈          I_line = I_cm + m·h²
    s̈ = ω_a²·(s_c − s) − 2·ζ·ω_a·ṡ              脚の横剛性（MIT kp）を 2 次遅れで

状態 x = [θ, θ̇, s, ṡ]（θ は重心が +n へ動く向きを正、s は実測の重心の支持線からの
距離）、入力 s_c は指令する重心位置。胴体の寄せ u = −s0 − K·x。

    ./scripts/knee_flip_balance_lqr.py            # 既定の [2.8, 0.57, 7.3, 1.4]（keel、h 0.30 で設計。MuJoCo で 0.34 m でも通る）
    ./scripts/knee_flip_balance_lqr.py --mass 53.3 --height 0.356 --icm 1.73 --wa 22

`misa-run sim` の起動ログ「2 脚支持 …: 質量 / 重心の高さ / I_cm」がそのまま引数。
"""
import argparse
import numpy as np
import scipy.linalg as la

ap = argparse.ArgumentParser()
ap.add_argument("--mass", type=float, default=53.3)
ap.add_argument("--height", type=float, default=0.30, help="足先から重心までの高さ [m]")
ap.add_argument("--icm", type=float, default=1.7, help="支持線の向きに射影した重心まわりの慣性 [kg m^2]")
ap.add_argument("--wa", type=float, default=22.0, help="脚の横剛性の固有角周波数 [rad/s]（kp 1200 で 22、500 で 14）")
ap.add_argument("--zeta", type=float, default=0.29)
ap.add_argument("--qx", type=float, default=10.0, help="重心の横ずれ x = s + h θ の重み")
ap.add_argument("--qxd", type=float, default=10.0)
ap.add_argument("--qth", type=float, default=0.1)
ap.add_argument("--r", type=float, default=1000.0)
a = ap.parse_args()

m, g, h, wa, z = a.mass, 9.81, a.height, a.wa, a.zeta
I = a.icm + m * h * h
A = np.array([[0, 1, 0, 0],
              [m * g * h / I, 0, m * g / I + m * h * wa * wa / I, 2 * z * wa * m * h / I],
              [0, 0, 0, 1],
              [0, 0, -wa * wa, -2 * z * wa]])
B = np.array([[0], [-m * h * wa * wa / I], [0], [wa * wa]])
Cx = np.array([[h, 0, 1, 0]])
Cxd = np.array([[0, h, 0, 1]])
Q = a.qx * Cx.T @ Cx + a.qxd * Cxd.T @ Cxd + np.diag([a.qth, 0.01, 0, 0])
P = la.solve_continuous_are(A, B, Q, np.array([[a.r]]))
K = (B.T @ P / a.r).flatten()
print(f"不安定極 {np.sqrt(m * g * h / I):.2f} rad/s（I_line {I:.2f}）")
print(f"knee_flip_balance_gains = [{K[0]:.2f}, {K[1]:.2f}, {K[2]:.2f}, {K[3]:.2f}]")
print("閉ループ極", np.round(np.linalg.eigvals(A - B @ K[None, :]), 2))
