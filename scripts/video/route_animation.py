#!/usr/bin/env python3
"""固定巡回と AI 巡回を左右に並べ、実際の走行軌跡が伸びていくアニメーション(MP4)を作る.

mission_planner が保存した結果(patrol_summary.json と trajectory.csv)を 2 つ読み、
地図(world_layout.yaml)の上に走行軌跡・候補地点・熊・熊の発見位置を描く。

  python3 scripts/video/route_animation.py \\
      --left  ~/kumawatch_patrol_output/<日時>_fixed \\
      --right ~/kumawatch_patrol_output/<日時>_utility \\
      --out video_work/compare_routes.mp4 --seconds 14
"""

import argparse
import csv
import json
import math
import os
import subprocess
import sys

from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'src', 'kumawatch_gazebo'))
import captions as C  # noqa: E402
from kumawatch_gazebo import world_builder as wb  # noqa: E402

W, H = C.W, C.H
PANEL = 760
TOP = 175
BG = (14, 19, 24)
LAYOUT = os.path.join(ROOT, 'src', 'kumawatch_gazebo', 'config', 'world_layout.yaml')
POINTS = os.path.join(ROOT, 'src', 'kumawatch_ai', 'data', 'patrol_points.yaml')


def load_run(d):
    with open(os.path.join(d, 'patrol_summary.json'), encoding='utf-8') as f:
        s = json.load(f)
    with open(os.path.join(d, 'trajectory.csv'), encoding='utf-8') as f:
        traj = [(float(r['x']), float(r['y'])) for r in csv.DictReader(f)]
    return s, traj


class Panel:
    def __init__(self, x0, layout, points, summary):
        self.x0, self.layout, self.points, self.s = x0, layout, points, summary
        b = layout['bounds']
        self.b = b
        self.scale = PANEL / (b['x_max'] - b['x_min'])
        self.bg = self._background()

    def px(self, x, y):
        return (self.x0 + (x - self.b['x_min']) * self.scale,
                TOP + (self.b['y_max'] - y) * self.scale)

    def _background(self):
        img = Image.new('RGBA', (W, H), (0, 0, 0, 0))
        d = ImageDraw.Draw(img)
        d.rectangle((*self.px(self.b['x_min'], self.b['y_max']),
                     *self.px(self.b['x_max'], self.b['y_min'])), fill=(77, 115, 56))
        for s in self.layout.get('surfaces', []):
            col = tuple(int(255 * c) for c in s['color'][:3])
            d.rectangle((*self.px(s['x_min'], s['y_max']), *self.px(s['x_max'], s['y_min'])),
                        fill=col)
        for shape in wb.static_obstacles(self.layout):
            if shape[0] == 'circle':
                _, cx, cy, r = shape
                (x, y), rr = self.px(cx, cy), r * 2 * self.scale
                d.ellipse((x - rr, y - rr, x + rr, y + rr), fill=(26, 97, 31))
            else:
                _, cx, cy, sx, sy, yaw = shape
                if sx > 1.6 and sy > 15:
                    continue
                c, s_ = math.cos(yaw), math.sin(yaw)
                pts = [self.px(cx + dx * c - dy * s_, cy + dx * s_ + dy * c)
                       for dx, dy in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2),
                                      (-sx / 2, sy / 2))]
                d.polygon(pts, fill=(217, 209, 191), outline=(40, 40, 40))
        sel = {r['point_id']: r['order'] for r in self.s['results']}
        for p in self.points['points']:
            risk = next((r['risk'] for r in self.s['results'] if r['point_id'] == p['id']), None)
            x, y = self.px(p['x'], p['y'])
            on = p['id'] in sel
            d.ellipse((x - 13, y - 13, x + 13, y + 13), fill=(255, 150, 60) if on else (140, 150, 160),
                      outline=(255, 255, 255) if on else (30, 30, 30), width=3 if on else 1)
            d.text((x + 16, y - 30), p['id'] + (' #%d' % sel[p['id']] if on else ''),
                   font=C.font(26, 'bold'), fill=(255, 255, 255) if on else (200, 205, 210))
            del risk
        for a in self.layout.get('animals', []):
            x, y = self.px(a['x'], a['y'])
            if a['species'] == 'bear':
                d.rectangle((x - 11, y - 8, x + 11, y + 8), fill=(20, 16, 14), outline=(255, 255, 255))
                d.text((x - 20, y - 36), 'bear', font=C.font(20), fill=(255, 255, 255))
            else:
                d.text((x - 8, y - 16), '×', font=C.font(30, 'bold'), fill=(210, 160, 90))
        base = self.points['base']
        x, y = self.px(base['x'], base['y'])
        d.rectangle((x - 14, y - 14, x + 14, y + 14), fill=(50, 100, 230), outline=(255, 255, 255))
        return img


def cumulative(traj):
    out = [0.0]
    for a, b in zip(traj, traj[1:]):
        out.append(out[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--left', required=True, help='固定巡回の結果フォルダ')
    ap.add_argument('--right', required=True, help='AI 巡回の結果フォルダ')
    ap.add_argument('--out', default='video_work/compare_routes.mp4')
    ap.add_argument('--seconds', type=float, default=14.0)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--note', default='Navigation2 による実際の走行軌跡（同じ 3 地点・同じ熊の配置）')
    args = ap.parse_args()

    layout = wb.load_yaml(LAYOUT)
    points = wb.load_yaml(POINTS)
    runs = [load_run(args.left), load_run(args.right)]
    panels = [Panel(110, layout, points, runs[0][0]), Panel(1010, layout, points, runs[1][0])]
    cums = [cumulative(t) for _, t in runs]
    longest = max(c[-1] for c in cums)
    frames = int(args.seconds * args.fps)
    hold = int(2.0 * args.fps)       # 最後に 2 秒止める

    base = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(base)
    d.text((110, 36), '比較：固定ルート と AI ルート', font=C.font(56, 'bold'), fill=(240, 244, 248))
    d.text((112, 118), args.note, font=C.font(30), fill=(160, 175, 190))
    for p in panels:
        base.paste(p.bg, (0, 0), p.bg)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    ff = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                           '-s', '%dx%d' % (W, H), '-r', str(args.fps), '-i', '-',
                           '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '18', args.out],
                          stdin=subprocess.PIPE)
    labels = ('固定ルート', 'AI ルート')
    for i in range(frames + hold):
        dist = longest * min(1.0, i / frames)
        img = base.copy()
        d = ImageDraw.Draw(img)
        for k, ((s, traj), cum, p) in enumerate(zip(runs, cums, panels)):
            n = sum(1 for c in cum if c <= dist)
            pts = [p.px(x, y) for x, y in traj[:max(n, 1)]]
            if len(pts) >= 2:
                d.line(pts, fill=(60, 190, 255), width=6, joint='curve')
            x, y = pts[-1]
            d.ellipse((x - 11, y - 11, x + 11, y + 11), fill=(255, 220, 0), outline=(0, 0, 0), width=2)
            found = 0
            for e in s['events']:   # 熊の発見位置に軌跡が届いたら赤い輪
                idx = min(range(len(traj)), key=lambda j: math.hypot(
                    traj[j][0] - e['robot_x'], traj[j][1] - e['robot_y']))
                if n > idx:
                    found += 1
                    bx, by = p.px(e['bear_x'], e['bear_y'])
                    d.ellipse((bx - 26, by - 26, bx + 26, by + 26), outline=(255, 40, 40), width=6)
            # 表示する距離は実行ログ(patrol_summary.json)の値に合わせる(軌跡の積算は概算なので比率だけ使う)
            walked = s['distance_m'] * cum[max(n, 1) - 1] / cum[-1] if cum[-1] > 0 else 0.0
            col = (255, 150, 60) if k == 1 else (200, 210, 220)
            d.text((p.x0, TOP + PANEL + 14), '%s  %s' % (labels[k], ' → '.join(s['route'])),
                   font=C.font(34, 'bold'), fill=col)
            d.text((p.x0, TOP + PANEL + 60 - 4), '熊の発見 %d 頭   移動 %.1f m' % (found, walked),
                   font=C.font(32), fill=(120, 220, 140) if found else (240, 244, 248))
        ff.stdin.write(img.tobytes())
    ff.stdin.close()
    ff.wait()
    print('保存しました: %s' % args.out)


if __name__ == '__main__':
    main()
