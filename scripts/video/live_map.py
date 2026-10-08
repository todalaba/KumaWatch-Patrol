#!/usr/bin/env python3
"""巡回 1 回分の記録から、ロボットの位置が地図の上を動く映像(RViz の代わり)を作る.

画面録画に RViz が映っていないときや、RViz より見やすい地図が欲しいときに使う。
mission_planner が保存した trajectory.csv(位置と ROS 時刻)と mission_events.csv(出来事)を読み、
走行軌跡・巡回地点(AI が選んだ順番)・熊の発見位置・退避先・状態を ROS 時刻どおりに描く。
image_recorder と同じ形の <動画名>.json(first_stamp_s = 最初のフレームの ROS 時刻)も書くので、
edl_from_run.py の --map にそのまま渡せる。

  python3 scripts/video/live_map.py --run ~/kumawatch_patrol_output/<日時>_utility \\
      --out video_work/live_map.mp4
"""

import argparse
import bisect
import csv
import json
import math
import os
import subprocess
import sys

from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import captions as C  # noqa: E402
import route_animation as RA  # noqa: E402
from kumawatch_gazebo import world_builder as wb  # noqa: E402  (route_animation がパスを通す)

MW, MH = 760, 940            # 出力の大きさ(地図 760×760 + 下の状態表示)
BG = (14, 19, 24)
SAFE = (80, 230, 120)
ALERT = (255, 50, 50)
PATH = (60, 190, 255)
STATE_TEXT = {'navigate': 'Navigating', 'arrived': 'Arrived - observing', 'clear': 'Clear',
              'bear_detected': 'BEAR DETECTED - stopped', 'retreat': 'Retreating to safe point',
              'retreat_done': 'Safe point reached', 'retreat_failed': 'Safe point unreachable',
              'unreachable': 'Point unreachable', 'timeout': 'Timeout',
              'return_to_base': 'Returning to base', 'returned': 'Back at base',
              'mission_completed': 'Mission completed', 'mission_started': 'Mission started'}


def read_run(run_dir):
    with open(os.path.join(run_dir, 'trajectory.csv'), encoding='utf-8') as f:
        traj = [(float(r['x']), float(r['y']), float(r['ros_time_s'])) for r in csv.DictReader(f)]
    with open(os.path.join(run_dir, 'mission_events.csv'), encoding='utf-8') as f:
        events = list(csv.DictReader(f))
    for e in events:
        e['ros_time_s'] = float(e['ros_time_s'])
        e['detail'] = json.loads(e['detail'] or '{}')
    with open(os.path.join(run_dir, 'patrol_summary.json'), encoding='utf-8') as f:
        summary = json.load(f)
    return traj, events, summary


def pose_at(traj, times, t):
    """時刻 t のロボットの位置と向き(軌跡を線形補間)."""
    i = bisect.bisect_right(times, t)
    if i <= 0:
        a = b = traj[0]
    elif i >= len(traj):
        a = b = traj[-1]
    else:
        a, b = traj[i - 1], traj[i]
    if b[2] > a[2]:
        s = (t - a[2]) / (b[2] - a[2])
        x, y = a[0] + (b[0] - a[0]) * s, a[1] + (b[1] - a[1]) * s
    else:
        x, y = a[0], a[1]
    j = max(1, min(i, len(traj) - 1))
    k = j
    while k > 1 and math.hypot(traj[k][0] - traj[k - 1][0], traj[k][1] - traj[k - 1][1]) < 0.02:
        k -= 1
    yaw = math.atan2(traj[k][1] - traj[k - 1][1], traj[k][0] - traj[k - 1][0])
    return x, y, yaw, i


def state_at(events, t):
    """時刻 t までの出来事から、状態表示に使う情報を集める."""
    st = {'text': 'Waiting', 'point': '', 'index': '', 'total': '', 'risk': None,
          'bears': [], 'retreat': None, 'retreat_state': None, 'goal': None, 'done': False}
    for e in events:
        if e['ros_time_s'] > t:
            break
        k, d = e['event'], e['detail']
        st['text'] = STATE_TEXT.get(k, k)
        if k == 'mission_started':
            st['total'] = len(d.get('route', []))
        elif k == 'navigate':
            st.update(point=e['point_id'], index=e['index'], risk=d.get('risk'),
                      goal=e['point_id'], retreat=None, retreat_state=None)
        elif k == 'bear_detected':
            st['bears'].append(d)
        elif k == 'retreat':
            st['retreat'] = (d.get('x'), d.get('y'))
            st['retreat_state'] = 'going'
        elif k in ('retreat_done', 'retreat_failed'):
            st['retreat_state'] = 'done' if k == 'retreat_done' else 'failed'
        elif k == 'return_to_base':
            st.update(goal='base', point='', index='')
        elif k == 'mission_completed':
            st['done'] = True
    return st


def render_frames(run_dir, fps, hold_s, title):
    traj, events, summary = read_run(run_dir)
    times = [p[2] for p in traj]
    layout = wb.load_yaml(RA.LAYOUT)
    points = wb.load_yaml(RA.POINTS)
    panel = RA.Panel(0, layout, points, summary)
    bg = Image.new('RGB', (MW, MH), BG)
    bg.paste(panel.bg.crop((0, RA.TOP, MW, RA.TOP + MW)), (0, 60),
             panel.bg.crop((0, RA.TOP, MW, RA.TOP + MW)))
    d = ImageDraw.Draw(bg)
    d.text((16, 10), title, font=C.font(34, 'bold'), fill=(240, 244, 248))
    pts = {p['id']: (p['x'], p['y']) for p in points['points']}
    base = (points['base']['x'], points['base']['y'])

    def px(x, y):
        a, b = panel.px(x, y)
        return a, b - RA.TOP + 60

    t0 = traj[0][2]
    t_end = max(traj[-1][2], events[-1]['ros_time_s'])
    n = int((t_end - t0) * fps) + int(hold_s * fps)
    for f in range(n):
        t = min(t0 + f / fps, t_end)
        x, y, yaw, i = pose_at(traj, times, t)
        st = state_at(events, t)
        img = bg.copy()
        d = ImageDraw.Draw(img)
        goal = base if st['goal'] == 'base' else pts.get(st['goal'])
        if goal and not st['done']:
            d.line((px(x, y), px(*goal)), fill=(255, 200, 0), width=2)
        line = [px(p[0], p[1]) for p in traj[:i]] + [px(x, y)]
        if len(line) >= 2:
            d.line(line, fill=PATH, width=6, joint='curve')
        for b in st['bears']:
            bx, by = px(b['bear_x'], b['bear_y'])
            d.ellipse((bx - 24, by - 24, bx + 24, by + 24), outline=ALERT, width=6)
            d.text((bx + 26, by - 14), '%.2f' % b.get('confidence', 0), font=C.font(24, 'bold'),
                   fill=ALERT)
        if st['retreat'] and st['retreat_state']:
            rx, ry = px(*st['retreat'])
            col = SAFE if st['retreat_state'] != 'failed' else (150, 150, 150)
            d.rectangle((rx - 14, ry - 14, rx + 14, ry + 14), outline=col, width=5)
            d.text((rx + 18, ry - 16), 'SAFE' if st['retreat_state'] != 'failed' else 'SAFE (blocked)',
                   font=C.font(24, 'bold'), fill=col)
        cx, cy = px(x, y)
        tip = (cx + 22 * math.cos(yaw), cy - 22 * math.sin(yaw))
        d.ellipse((cx - 13, cy - 13, cx + 13, cy + 13), fill=(255, 220, 0), outline=(0, 0, 0), width=2)
        d.line((cx, cy, *tip), fill=(0, 0, 0), width=4)
        # 下の状態表示
        y0 = 60 + MW + 8
        d.rectangle((0, y0 - 4, MW, MH), fill=BG)
        head = 'Goal %s/%s: %s' % (st['index'], st['total'], st['point']) if st['point'] else (
            'KumaWatch Patrol')
        if st['risk'] is not None and st['point']:
            head += '   AI risk %.2f' % st['risk']
        alert = st['text'].startswith('BEAR')
        d.text((16, y0), head, font=C.font(30, 'bold'), fill=(240, 244, 248))
        d.text((16, y0 + 40), '%s   |   bears %d   |   t=%.0f s' % (st['text'], len(st['bears']), t - t0),
               font=C.font(26, 'bold' if alert else 'regular'),
               fill=ALERT if alert else (160, 255, 170))
        yield img
    return


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run', required=True, help='mission_planner の結果フォルダ')
    ap.add_argument('--out', default='video_work/live_map.mp4')
    ap.add_argument('--fps', type=int, default=15)
    ap.add_argument('--hold', type=float, default=8.0, help='最後の状態を止めて見せる秒数')
    ap.add_argument('--title', default='Map (logged robot pose, Nav2 run)')
    args = ap.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    ff = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                           '-s', '%dx%d' % (MW, MH), '-r', str(args.fps), '-i', '-',
                           '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '18', args.out],
                          stdin=subprocess.PIPE)
    n = 0
    for img in render_frames(args.run, args.fps, args.hold, args.title):
        ff.stdin.write(img.tobytes())
        n += 1
    ff.stdin.close()
    ff.wait()
    traj, _, _ = read_run(args.run)
    with open(os.path.splitext(args.out)[0] + '.json', 'w', encoding='utf-8') as f:
        json.dump({'first_stamp_s': traj[0][2], 'fps': args.fps, 'size': [MW, MH],
                   'source': 'trajectory.csv'}, f)
    print('保存しました: %s(%d フレーム、%.1f 秒)' % (args.out, n, n / args.fps))


if __name__ == '__main__':
    main()
