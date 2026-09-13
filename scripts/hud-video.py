#!/usr/bin/env python3
"""`sim --csv` と `sim --video` から、計器板つきの動画を作る。

**指令と実測を数字で並べないと、絵だけでは何が起きたか分からない。** 押された
瞬間に推定速度が真値から外れる、といったことは計器板にしか出ない。

    misa-run sim --robot <profile> --gait trot --vx 0.12 --secs 14 \\
        --timestep 0.0005 --kp 500 --kv 5 --safety-gate \\
        --push "6.0,0.2,0,-600,0" --csv /tmp/run.csv \\
        --video /tmp/frames --cam-fixed --cam-az 0 --cam-el -11 \\
        --cam-dist 1.55 --cam-x 0.45 --cam-y -0.22 --cam-z 0.26

    ./scripts/hud-video.py /tmp/run.csv /tmp/frames out.mp4 \\
        --title "keel 600N 0.2s | MPC+WBC" --push 6.0,0.2,0,-600,0

**カメラの向き。** `--cam-az 0` で機体の前がカメラ側、`180` で後ろ姿、
`90` で真横（右が前）。横に押すところを見せるなら正面か後ろ、進む量を
見せるなら真横。

**幅は 640 から広げられない。** MuJoCo のオフスクリーンバッファの既定が
640x480 で、articara の MJCF エクスポータは `<visual><global offwidth>` を
出さない。超えると描画領域が左上に寄って黒帯になる。
"""

import argparse
import csv
import os
import subprocess
import sys
from PIL import Image, ImageDraw, ImageFont

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
W, H = 640, 214
DEG = 57.29577951308232


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
    a = ap.parse_args()

    rows, ts = load(a.csv)
    n = len([f for f in os.listdir(a.frames) if f.startswith("frame_")])
    if n == 0:
        sys.exit(f"{a.frames} に frame_*.png がありません")
    push = tuple(float(x) for x in a.push.split(",")) if a.push else None
    tmp = os.path.join(os.path.dirname(os.path.abspath(a.out)) or ".", ".hud_panels")
    panel(tmp, rows, ts, n, a.title, push, a.fps)

    def ff(*args):
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", *args], check=True)

    robot = f"{tmp}/robot.mp4"
    pan = f"{tmp}/panel.mp4"
    ff("-framerate", str(a.fps), "-i", f"{a.frames}/frame_%05d.png", "-vf", a.eq,
       "-c:v", "libx264", "-pix_fmt", "yuv420p", "-r", str(a.fps), robot)
    ff("-framerate", str(a.fps), "-i", f"{tmp}/p_%05d.png",
       "-c:v", "libx264", "-pix_fmt", "yuv420p", "-r", str(a.fps), pan)
    # 計器板が上、絵が下。添付の参照画像と同じ並び。
    ff("-i", pan, "-i", robot, "-filter_complex", "[0:v][1:v]vstack=inputs=2",
       "-c:v", "libx264", "-pix_fmt", "yuv420p", a.out)
    print(f"{a.out} を書きました（{n} フレーム、{n / a.fps:.1f} s）")


if __name__ == "__main__":
    main()
