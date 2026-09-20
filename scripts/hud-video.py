#!/usr/bin/env python3
"""`sim --csv` と `sim --video` から、計器板つきの動画を作る。

**指令と実測を数字で並べないと、絵だけでは何が起きたか分からない。** 押された
瞬間に推定速度が真値から外れる、といったことは計器板にしか出ない。

    misa-run sim --robot <profile> --gait trot --vx 0.12 --secs 14 \\
        --timestep 0.0005 --kp 500 --kv 5 --safety-gate \\
        --push "6.0,0.2,0,-600,0" --csv /tmp/run.csv \\
        --video /tmp/frames --cam-fixed --cam-az 180 --cam-el -11 \\
        --cam-dist 1.55 --cam-x 0.45 --cam-y -0.22 --cam-z 0.26

    ./scripts/hud-video.py /tmp/run.csv /tmp/frames out.mp4 \\
        --title "keel 600N 0.2s | MPC+WBC" --push 6.0,0.2,0,-600,0

**カメラの向き。** `--cam-az 180` で機体の前がカメラ側、`0` で後ろ姿、
`90` で真横。**0 と 180 を取り違えないこと** — この機体は前後がよく似て
いて絵では判別できない。確かめ方は `--vx` を正で走らせて、機体が小さく
なれば背面（az 0）、大きくなれば正面（az 180）。

**世界座標の重畳（`--cam`）。** sim へ渡したのと同じ
`az,el,dist,x,y,z` を渡すと、目標位置の点と外力の矢印を絵の中の正しい
場所に描く。`--cam-fixed` の自由カメラだけが対象（追従カメラは注視点が
毎周期動くので再現できない）。

    ./scripts/hud-video.py /tmp/run.csv /tmp/frames out.mp4 \\
        --push 6.0,0.2,0,-500,0 --cam 180,-11,2.2,0,0,0.26

**幅は 640 から広げられない。** MuJoCo のオフスクリーンバッファの既定が
640x480 で、articara の MJCF エクスポータは `<visual><global offwidth>` を
出さない。超えると描画領域が左上に寄って黒帯になる。
"""

import argparse
import csv
import math
import os
import subprocess
import sys
from PIL import Image, ImageDraw, ImageFont

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
W, H = 640, 214
DEG = 57.29577951308232


def cam_basis(az, el):
    """MuJoCo の自由カメラの前・右・上（世界座標の単位ベクトル）。"""
    a, e = math.radians(az), math.radians(el)
    f = (math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e))
    r = (f[1], -f[0], 0.0)  # cross(f, +z)
    n = math.hypot(r[0], r[1]) or 1.0
    r = (r[0] / n, r[1] / n, 0.0)
    u = (
        r[1] * f[2] - r[2] * f[1],
        r[2] * f[0] - r[0] * f[2],
        r[0] * f[1] - r[1] * f[0],
    )
    return f, r, u


class Cam:
    """`sim --cam-fixed` の自由カメラを再現して、世界座標を画素へ落とす。

    **fovy は MuJoCo の既定 45°。** articara の MJCF エクスポータは
    `<visual><global fovy>` を出さないので、モデル側に指定はない。
    """

    def __init__(self, spec, w, h, fovy=45.0):
        az, el, dist, x, y, z = (float(v) for v in spec.split(","))
        self.f, self.r, self.u = cam_basis(az, el)
        self.pos = tuple((x, y, z)[i] - dist * self.f[i] for i in range(3))
        self.w, self.h = w, h
        self.fp = (h / 2) / math.tan(math.radians(fovy) / 2)

    def __call__(self, p):
        d = tuple(p[i] - self.pos[i] for i in range(3))
        zc = sum(d[i] * self.f[i] for i in range(3))
        if zc <= 1e-6:  # カメラの後ろ
            return None
        xc = sum(d[i] * self.r[i] for i in range(3))
        yc = sum(d[i] * self.u[i] for i in range(3))
        return (self.w / 2 + self.fp * xc / zc, self.h / 2 - self.fp * yc / zc)


def arrow(d, tail, head, col, width):
    """2 点（画素）を結ぶ矢印。頭は画面上で描くので奥行きで歪まない。"""
    dx, dy = head[0] - tail[0], head[1] - tail[1]
    L = math.hypot(dx, dy)
    if L < 1e-3:
        return
    ux, uy = dx / L, dy / L
    hl = min(max(width * 3.0, 10.0), L * 0.5)  # 頭の長さ
    hw = hl * 0.6
    base = (head[0] - ux * hl, head[1] - uy * hl)
    d.line([tail, base], fill=col, width=int(width))
    d.polygon(
        [head, (base[0] - uy * hw, base[1] + ux * hw), (base[0] + uy * hw, base[1] - ux * hw)],
        fill=col,
    )


def overlay_world(im, r, cam, push, t, font, n_scale):
    """目標位置の点と、外力の矢印を絵の上に描く。"""
    d = ImageDraw.Draw(im, "RGBA")
    g = lambda k: float(r[k])
    here = (g("x_world"), g("y_world"))
    goal = (g("hold_x"), g("hold_y"))

    # 目標位置 — 地面の点。同心円にして、機体に隠れても輪郭が残るようにする。
    q = cam((goal[0], goal[1], 0.0))
    p_here = cam((here[0], here[1], 0.0))
    if q:
        if p_here and math.dist(q, p_here) > 4:
            d.line([q, p_here], fill=(120, 230, 255, 150), width=2)
        for rad, wdt in ((13, 3), (5, 3)):
            d.ellipse([q[0] - rad, q[1] - rad, q[0] + rad, q[1] + rad],
                      outline=(120, 230, 255, 255), width=wdt)
        err = math.dist(here, goal)
        d.text((q[0] + 17, q[1] - 8), f"TARGET  err {err:.3f} m",
               font=font, fill=(120, 230, 255, 255))

    if not push:
        return
    t0, dur, fx, fy, fz = push
    mag = math.sqrt(fx * fx + fy * fy + fz * fz)
    if mag < 1e-6:
        return
    # **押していない間は何も描かない。** 薄い矢印を残すと、絵を横切る線が
    # 出っぱなしになって外力が続いているように見える。予告は計器板の
    # FORCE 行（UPCOMING / ACTIVE / DONE）で足りる。
    active = t0 <= t <= t0 + dur
    if not active:
        return
    # **矢印は長さと太さの両方で大きさを表す。** 長さだけだと、奥行きで
    # 縮んだのか力が小さいのか区別できない。
    L = n_scale * mag
    u = (fx / mag, fy / mag, fz / mag)
    head = (here[0], here[1], g("z"))
    tail = tuple(head[i] - L * u[i] for i in range(3))
    a, b = cam(tail), cam(head)
    if not (a and b):
        return
    col = (255, 70, 70, 255)
    arrow(d, a, b, col, 3 + 7 * mag / 600.0)
    d.text((a[0] - 10, a[1] - 26), f"{mag:.0f} N", font=font, fill=col)
    # 発生点（力を掛けている所）を丸で囲う。
    d.ellipse([b[0] - 7, b[1] - 7, b[0] + 7, b[1] + 7], outline=col, width=2)


def eq_lut(eq):
    """ffmpeg の `eq=brightness=B:contrast=C` と同じ変換を PIL の LUT で作る。

    **重畳より先に明るさを直す。** 後から ffmpeg でかけると、矢印や点の色
    まで持ち上がって白く飛ぶ。
    """
    b = c = None
    for part in eq.replace("eq=", "").split(":"):
        k, _, v = part.partition("=")
        if k == "brightness":
            b = float(v)
        elif k == "contrast":
            c = float(v)
    b, c = (b or 0.0), (1.0 if c is None else c)
    return [max(0, min(255, round(((i / 255 - 0.5) * c + 0.5 + b) * 255))) for i in range(256)] * 3


def load(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        sys.exit(f"{path} が空です")
    return rows, [float(r["t"]) for r in rows]


def panel(out_dir, rows, ts, n_frames, title, push, fps):
    """1 フレーム 1 枚、計器板の PNG を書く。"""
    font = ImageFont.truetype(FONT, 14) if os.path.exists(FONT) else ImageFont.load_default()
    os.makedirs(out_dir, exist_ok=True)
    t0, dur, fx, fy, fz = push if push else (None, 0, 0, 0, 0)
    mag = (fx * fx + fy * fy + fz * fz) ** 0.5
    z0 = float(rows[0]["z"])
    for k in range(n_frames):
        t = k / fps
        # CSV は制御周期（200 Hz）、動画は fps。いちばん近い行を採る。
        i = min(range(len(ts)), key=lambda j: abs(ts[j] - max(t, ts[0])))
        r = rows[i]
        g = lambda key: float(r[key])
        im = Image.new("RGB", (W, H), (11, 17, 26))
        d = ImageDraw.Draw(im)
        d.rectangle([0, 0, W - 1, H - 1], outline=(120, 140, 165))
        y, lh = 7, 21
        d.text((9, y), f"{title} | MuJoCo | t={t:5.1f}s", font=font, fill=(235, 240, 248))
        y += lh + 3
        for label, keys, col in (
            ("REQUEST ", ("vx_req", "vy_req", "wz_req"), (225, 230, 240)),
            ("COMMAND ", ("vx_cmd", "vy_cmd", "wz_cmd"), (240, 200, 80)),
            ("ACTUAL  ", ("vx_true", "vy_true", "wz_true"), (90, 225, 120)),
        ):
            d.text(
                (9, y),
                f"{label}  vx {g(keys[0]):+.3f}   vy {g(keys[1]):+.3f} m/s   wz {g(keys[2]):+.3f} rad/s",
                font=font,
                fill=col,
            )
            y += lh
        d.text(
            (9, y),
            f"ESTIMATE  vx {g('vx_est'):+.3f}   vy {g('vy_est'):+.3f} m/s",
            font=font,
            fill=(110, 175, 245),
        )
        y += lh
        d.text(
            (9, y),
            f"BODY   height {g('z'):.3f} m (initial {z0:.3f})  "
            f"roll {g('roll') * DEG:+.2f}  pitch {g('pitch') * DEG:+.2f} deg",
            font=font,
            fill=(225, 230, 240),
        )
        y += lh
        d.text(
            (9, y),
            f"stance widen {g('widen'):.3f} m | ACTUAL = sim truth",
            font=font,
            fill=(165, 180, 200),
        )
        y += lh
        if push:
            state = "ACTIVE" if t0 <= t <= t0 + dur else ("UPCOMING" if t < t0 else "DONE")
            hot = state == "ACTIVE"
            d.text(
                (9, y),
                f"FORCE {state}  {mag:.1f} N   body Fx {fx:+.1f}  Fy {fy:+.1f}  Fz {fz:+.1f} N",
                font=font,
                fill=(255, 90, 90) if hot else (200, 120, 120),
            )
            y += lh
            d.text(
                (9, y),
                f"POINT trunk origin   t={t0:.2f}-{t0 + dur:.2f}s",
                font=font,
                fill=(200, 120, 120),
            )
        im.save(f"{out_dir}/p_{k:05d}.png")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("frames", help="sim --video が書いた PNG のディレクトリ")
    ap.add_argument("out")
    ap.add_argument("--title", default="misa-run sim")
    ap.add_argument("--push", help="sim へ渡したのと同じ t,dur,fx,fy,fz")
    ap.add_argument("--fps", type=float, default=30.0)
    # **明るさ補正は要る。** articara の MJCF エクスポータは光源を出さないので
    # MuJoCo の既定ヘッドライトだけになって暗い。
    ap.add_argument("--eq", default="eq=brightness=0.30:contrast=1.5")
    ap.add_argument("--cam", help="sim へ渡したのと同じ az,el,dist,x,y,z（--cam-fixed 用）")
    ap.add_argument("--force-scale", type=float, default=0.0012,
                    help="矢印の長さ [m/N]。既定は 500 N で 0.6 m")
    a = ap.parse_args()

    rows, ts = load(a.csv)
    n = len([f for f in os.listdir(a.frames) if f.startswith("frame_")])
    if n == 0:
        sys.exit(f"{a.frames} に frame_*.png がありません")
    push = tuple(float(x) for x in a.push.split(",")) if a.push else None
    tmp = os.path.join(os.path.dirname(os.path.abspath(a.out)) or ".", ".hud_panels")
    panel(tmp, rows, ts, n, a.title, push, a.fps)

    src = a.frames
    if a.cam:
        src = os.path.join(tmp, "over")
        os.makedirs(src, exist_ok=True)
        font = (ImageFont.truetype(FONT, 13) if os.path.exists(FONT)
                else ImageFont.load_default())
        lut = eq_lut(a.eq)
        for k in range(n):
            im = Image.open(f"{a.frames}/frame_{k:05d}.png").convert("RGB").point(lut)
            t = k / a.fps
            i = min(range(len(ts)), key=lambda j: abs(ts[j] - max(t, ts[0])))
            cam = Cam(a.cam, *im.size)
            overlay_world(im, rows[i], cam, push, t, font, a.force_scale)
            im.save(f"{src}/frame_{k:05d}.png")

    def ff(*args):
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", *args], check=True)

    robot = f"{tmp}/robot.mp4"
    pan = f"{tmp}/panel.mp4"
    # --cam のときは明るさ補正を PIL で済ませてある。
    ff("-framerate", str(a.fps), "-i", f"{src}/frame_%05d.png",
       "-vf", "null" if a.cam else a.eq,
       "-c:v", "libx264", "-pix_fmt", "yuv420p", "-r", str(a.fps), robot)
    ff("-framerate", str(a.fps), "-i", f"{tmp}/p_%05d.png",
       "-c:v", "libx264", "-pix_fmt", "yuv420p", "-r", str(a.fps), pan)
    # 計器板が上、絵が下。添付の参照画像と同じ並び。
    ff("-i", pan, "-i", robot, "-filter_complex", "[0:v][1:v]vstack=inputs=2",
       "-c:v", "libx264", "-pix_fmt", "yuv420p", a.out)
    print(f"{a.out} を書きました（{n} フレーム、{n / a.fps:.1f} s）")


if __name__ == "__main__":
    main()
