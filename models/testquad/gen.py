#!/usr/bin/env python3
"""汎用の試験用四脚モデル `testquad.misa` を書き出す。

**特定の機体ではない。** misa-runner のテストと既定のモデルパスが namiashi の
submodule（機体側のリポジトリ）に依らないようにするための、丸い数字だけで
組んだ四脚。トポロジは misa-runner が前提にしているもの:

    trunk ─ {FL,FR,RL,RR}_hip_joint (roll, x) ─ *_thigh_joint (pitch, y)
          ─ *_calf_joint (pitch, y) ─ *_foot_fixed ─ *_foot
    trunk ─ arm_pitch_joint (pitch, y) ─ arm

脚長 0.15 + 0.15 m、hip の横オフセット 0.075 m、質量 2.9 kg（胴体 1.6、脚 4 ×
0.31、腕 0.05）。慣性は形状（箱・球）から出す。ポーズ・シーケンスの名前と
角度は misa-runner のコードが名指しするもの（`constrain` / `extend` /
`start` / `standby` / `wave_*` / `greeting` / `jump`）。

    python3 models/testquad/gen.py   # models/testquad/testquad.misa を上書き

生成物も追跡する（cargo test は Python を要求しない）。
"""
import math, os

OUT = os.path.join(os.path.dirname(__file__), "testquad.misa")

def f(x):
    return repr(float(x))

def vec(v):
    return "[" + ", ".join(f(x) for x in v) + "]"

def box_inertia(m, sx, sy, sz):
    return (m/12*(sy*sy+sz*sz), m/12*(sx*sx+sz*sz), m/12*(sx*sx+sy*sy))

def sphere_inertia(m, r):
    i = 0.4*m*r*r
    return (i, i, i)

out = []
w = out.append
w('schema = "misarta/1"\n')
w('[robot]\nname = "testquad"\nroot = "trunk"\n')

def link(name, mass, inertia, com, geoms):
    w(f'\n[[link]]\nname = "{name}"\n')
    w('[link.inertial]\n')
    w(f'mass = {f(mass)}\nixx = {f(inertia[0])}\niyy = {f(inertia[1])}\nizz = {f(inertia[2])}\nixy = 0.0\nixz = 0.0\niyz = 0.0\n')
    if any(abs(c) > 0 for c in com):
        w(f'[link.inertial.origin]\nxyz = {vec(com)}\nrpy = [0.0, 0.0, 0.0]\n')
    for kind, arg, origin in geoms:
        for sec in ("visual", "collision"):
            w(f'[[link.{sec}]]\n')
            if origin is not None:
                w(f'[link.{sec}.origin]\nxyz = {vec(origin)}\nrpy = [0.0, 0.0, 0.0]\n')
            if kind == "box":
                w(f'[link.{sec}.geom.box]\nsize = {vec(arg)}\n')
            else:
                w(f'[link.{sec}.geom.sphere]\nradius = {f(arg)}\n')

def joint(name, kind, parent, child, axis, xyz, limit=None):
    w(f'\n[[joint]]\nname = "{name}"\ntype = "{kind}"\nparent = "{parent}"\nchild = "{child}"\naxis = {vec(axis)}\n')
    w(f'[joint.origin]\nxyz = {vec(xyz)}\nrpy = [0.0, 0.0, 0.0]\n')
    if limit:
        lo, hi, eff, vel = limit
        w(f'[joint.limit]\nlower = {f(lo)}\nupper = {f(hi)}\neffort = {f(eff)}\nvelocity = {f(vel)}\n')
        w('[joint.dynamics]\narmature = 0.0014\ndamping = 0.1\nfriction = 0.0\n')

# ── 胴体と腕 ─────────────────────────────────────────────
TRUNK = (0.20, 0.12, 0.09)
link("trunk", 1.6, box_inertia(1.6, *TRUNK), (0, 0, 0), [("box", TRUNK, None)])
ARM = (0.10, 0.016, 0.02)
link("arm", 0.05, box_inertia(0.05, *ARM), (0.05, 0, 0), [("box", ARM, (0.05, 0, 0))])
joint("arm_pitch_joint", "revolute", "trunk", "arm", (0, 1, 0), (0.10, 0, 0.06), (-2.3, 0.85, 6.865, 8.727))

# ── 脚 ───────────────────────────────────────────────────
L1 = 0.15   # thigh
L2 = 0.15   # calf
HIP_X, HIP_Y, THIGH_Y = 0.15, 0.035, 0.075
HIP_LIM, PITCH_LIM = 1.05, 2.62
EFF_HIP, EFF_CALF, VEL = 1.5, 2.2, 33.5
LEGS = {"FL": (1, 1), "FR": (1, -1), "RL": (-1, 1), "RR": (-1, -1)}
for leg, (sx, sy) in LEGS.items():
    hip, thigh, calf, foot = f"{leg}_hip", f"{leg}_thigh", f"{leg}_calf", f"{leg}_foot"
    link(hip, 0.15, box_inertia(0.15, 0.04, 0.04, 0.04), (0, sy*THIGH_Y/2, 0), [("box", (0.04, 0.04, 0.04), (0, sy*THIGH_Y/2, 0))])
    link(thigh, 0.10, box_inertia(0.10, 0.02, 0.03, L1), (0, 0, -L1/2), [("box", (0.02, 0.03, L1), (0, 0, -L1/2))])
    link(calf, 0.05, box_inertia(0.05, 0.016, 0.016, L2), (0, 0, -L2/2), [("box", (0.016, 0.016, L2), (0, 0, -L2/2))])
    link(foot, 0.01, sphere_inertia(0.01, 0.016), (0, 0, 0), [("sphere", 0.016, None)])
    joint(f"{leg}_hip_joint", "revolute", "trunk", hip, (1, 0, 0), (sx*HIP_X, sy*HIP_Y, 0), (-HIP_LIM, HIP_LIM, EFF_HIP, VEL))
    joint(f"{leg}_thigh_joint", "revolute", hip, thigh, (0, 1, 0), (0, sy*THIGH_Y, 0), (-PITCH_LIM, PITCH_LIM, EFF_HIP, VEL))
    joint(f"{leg}_calf_joint", "revolute", thigh, calf, (0, 1, 0), (0, 0, -L1), (-PITCH_LIM, PITCH_LIM, EFF_CALF, VEL))
    joint(f"{leg}_foot_fixed", "fixed", calf, foot, (1, 0, 0), (0, 0, -L2))

# hip 同士の当たりは見ない（隣り合っていて常に近い）。
pairs = [("FL_hip", "FR_hip"), ("RL_hip", "RR_hip"), ("FL_hip", "RL_hip"), ("FR_hip", "RR_hip"), ("FL_hip", "RR_hip"), ("FR_hip", "RL_hip")]
for a, b in pairs:
    w(f'\n[[collision_pair]]\nlink_a = "{a}"\nlink_b = "{b}"\nenabled = false\n')

# ── アクチュエータ ───────────────────────────────────────
JOINTS = [f"{leg}_{k}_joint" for leg in ("FL", "FR", "RL", "RR") for k in ("hip", "thigh", "calf")]
for j in ["arm_pitch_joint"] + JOINTS:
    kp, kv = (5.0, 0.5) if j == "arm_pitch_joint" else (100.0, 1.2)
    w(f'\n[[actuator]]\nname = "{j}_motor"\nmode = "Position"\nkp = {f(kp)}\nkv = {f(kv)}\n[[actuator.joints]]\nname = "{j}"\ngear = 1.0\n')

# ── ポーズ ───────────────────────────────────────────────
def legs(thigh, calf, hip=0.0, **per):
    """4 脚同じ角度。`per` で脚ごとに (hip, thigh, calf) を上書き。"""
    a = {}
    for leg in ("FL", "FR", "RL", "RR"):
        h, t, c = per.get(leg, (hip, thigh, calf))
        a[f"{leg}_hip_joint"], a[f"{leg}_thigh_joint"], a[f"{leg}_calf_joint"] = h, t, c
    a["arm_pitch_joint"] = 0.0
    return a

def pose(name, dur, angles):
    w(f'\n[[pose]]\nname = "{name}"\nduration = {f(dur)}\nkind = "QuinticSmooth"\n[pose.angles]\n')
    for k in sorted(angles):
        w(f'{k} = {f(angles[k])}\n')

pose("extend_full", 0.2, legs(0.0, 0.0))
pose("extend", 1.0, legs(0.3, -0.6))                    # 立ち姿勢（運動学の基準）
pose("constrain", 0.5, legs(1.0, -2.0))                 # 伏せ（既定の初期姿勢）
pose("constrain_2", 0.1, legs(1.3, -2.6))
pose("start", 0.5, legs(0.0, 0.0, FL=(0, 0.785, -2.52), FR=(0, 0.785, -2.52), RL=(0, 0.873, -1.309), RR=(0, 0.873, -1.309)))
pose("standby", 0.5, legs(0.785, -1.571))
pose("arm_raise", 0.4, {"arm_pitch_joint": -1.2})
pose("arm_lower", 0.4, {"arm_pitch_joint": -0.3})
pose("arm_home", 0.5, {"arm_pitch_joint": 0.0})
# 前足を振る。振る側の前足だけ変え、他は体重を後ろへ寄せた姿勢で固定。
def wave(side):
    other = "FL" if side == "FR" else "FR"
    m = 1.0 if side == "FR" else -1.0          # hip の符号は左右で反転
    base = {other: (-0.166*m, 0.515, -1.096), "RL": (-0.235*m, 0.831, -1.867), "RR": (-0.214*m, 0.704, -1.581)}
    def p(h, t, c):
        d = dict(base); d[side] = (h*m, t, c); return legs(0, 0, **d)
    pose(f"wave_{side.lower()}_ready", 0.4, p(-0.158, 0.340, -0.741))
    pose(f"wave_{side.lower()}_up", 0.4, p(-0.187, 0.459, -2.146))
    pose(f"wave_{side.lower()}_a", 0.4, p(-0.522, 0.452, -1.860))
    pose(f"wave_{side.lower()}_b", 0.4, p(0.041, 0.430, -2.256))
wave("FR"); wave("FL")

def seq(name, steps):
    w(f'\n[[sequence]]\nname = "{name}"\n')
    for p_, d in steps:
        w(f'[[sequence.steps]]\npose_name = "{p_}"\nduration = {f(d)}\nkind = "QuinticSmooth"\n')
seq("jump", [("constrain_2", 0.5), ("extend", 0.4), ("constrain_2", 0.1)])
seq("greeting", [("arm_raise", 0.5), ("arm_lower", 0.35), ("arm_raise", 0.35), ("arm_home", 0.6)])
for s in ("fr", "fl"):
    seq(f"wave_{s}", [(f"wave_{s}_ready", 0.8), (f"wave_{s}_up", 0.6), (f"wave_{s}_a", 0.35), (f"wave_{s}_b", 0.35), (f"wave_{s}_a", 0.35), (f"wave_{s}_b", 0.35), (f"wave_{s}_up", 0.4), (f"wave_{s}_ready", 0.6)])

w('\n[home]\nbase_position = [0.0, 0.0, 0.0]\nbase_orientation = [0.0, 0.0, 0.0, 1.0]\n[home.joint_positions]\n')
for j in ["arm_pitch_joint"] + JOINTS:
    w(f'{j} = 0.0\n')

open(OUT, "w").write("".join(out))
print(f"wrote {OUT} ({len(''.join(out))} bytes)")
