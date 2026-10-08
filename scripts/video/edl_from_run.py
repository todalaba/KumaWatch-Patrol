#!/usr/bin/env python3
"""巡回 1 回分の記録から、提出動画の編集リスト(YAML)を自動で作る.

mission_planner が書く mission_events.csv(出来事の時刻表)と、録画ファイルの時刻(.json)を
突き合わせて、「出発 → 移動(倍速)→ 熊の検知(等速・警告・効果音)→ 退避 → 到着 → 帰還」の
場面を切り出す。できた YAML は compose.py でそのまま動画になる(手で直してもよい)。

  python3 scripts/video/edl_from_run.py \\
      --run ~/kumawatch_patrol_output/<日時>_utility \\
      --screen ~/kumawatch_patrol_output/videos/screen_<日時>.mp4 \\
      --camera ~/kumawatch_patrol_output/videos/<日時>_detection.mp4 \\
      --chase  ~/kumawatch_patrol_output/videos/<日時>_chase.mp4 \\
      --gazebo-crop 0,60,1150,700 --rviz-crop 1160,60,760,900 \\
      --cards video_work/cards --out video_work/final.yaml
  python3 scripts/video/compose.py video_work/final.yaml

--screen の中の Gazebo と RViz の位置(x,y,幅,高さ)は、--frame-at 30 で 30 秒目の画像を
書き出して確かめる。--camera / --chase は省略できる(省略すると画面録画だけで作る)。
画面録画に RViz が映っていないときは、live_map.py で作った地図の映像を --map で渡すと
右側がその地図になる。
"""

import argparse
import csv
import json
import os
import subprocess
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
TARGET_MOVE_S = 12.0      # 移動の場面は、倍速にしてこの秒数くらいに縮める
LEFT, RIGHT = [20, 120, 1140, 870], [1170, 120, 730, 870]


def read_events(run_dir):
    with open(os.path.join(run_dir, 'mission_events.csv'), encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        r['wall_time'] = float(r['wall_time'])
        r['ros_time_s'] = float(r['ros_time_s'])
        r['detail'] = json.loads(r['detail'] or '{}')
    return rows


def sidecar(path):
    js = os.path.splitext(path)[0] + '.json'
    if not os.path.exists(js):
        raise SystemExit('%s がありません(録画と同じ名前の .json が必要です)' % js)
    with open(js, encoding='utf-8') as f:
        return json.load(f)


def crop_arg(text):
    v = [int(x) for x in text.split(',')]
    if len(v) != 4:
        raise argparse.ArgumentTypeError('x,y,幅,高さ で指定してください')
    return v


def speed_for(duration):
    if duration <= TARGET_MOVE_S * 1.2:
        return 1.0
    return max(1.5, round(duration / TARGET_MOVE_S * 2) / 2)


class Builder:

    def __init__(self, args, events, names):
        self.a, self.ev, self.names = args, events, names
        self.screen0 = sidecar(args.screen)['start_wall']
        self.cam0 = sidecar(args.camera)['first_stamp_s'] if args.camera else None
        self.chase0 = sidecar(args.chase)['first_stamp_s'] if args.chase else None
        self.map0 = sidecar(args.map)['first_stamp_s'] if args.map else None
        self.segments = []

    # 元動画の秒
    def st(self, e_or_wall):
        w = e_or_wall['wall_time'] if isinstance(e_or_wall, dict) else e_or_wall
        return max(0.0, w - self.screen0)

    def layers(self, t0, ros0, left='move'):
        """t0: 画面録画の開始秒, ros0: その瞬間の ROS 時刻."""
        a = self.a
        lay = []
        if left == 'camera' and a.camera:
            lay.append({'video': a.camera, 'start': round(ros0 - self.cam0, 2), 'box': LEFT,
                        'size': sidecar(a.camera)['size']})
        elif a.chase:
            lay.append({'video': a.chase, 'start': round(ros0 - self.chase0, 2), 'box': LEFT,
                        'size': sidecar(a.chase)['size']})
        else:
            lay.append({'crop': a.gazebo_crop, 'box': LEFT})
        lay.append(self.right_layer(ros0, RIGHT))
        return lay

    def right_layer(self, ros0, box):
        if self.a.map:
            return {'video': self.a.map, 'start': round(ros0 - self.map0, 2), 'box': box,
                    'size': sidecar(self.a.map)['size']}
        return {'crop': self.a.rviz_crop, 'box': box}

    def left_start(self):
        """移動の場面の左に置く映像(追従カメラ)が始まる ROS 時刻."""
        return self.chase0 if self.a.chase else float('-inf')

    def mission(self, idx, total, pid, risk, state):
        jp, en = self.names.get(pid, ('', ''))
        return {'style': 'mission', 'text': ['Mission %s/%s: %s %s (%s)' % (idx, total, pid, jp, en),
                                             'AI risk %.2f  |  %s' % (risk, state)]}

    def add_move(self, e0, e1, cap, lower=None):
        t0, t1 = self.st(e0), self.st(e1) - 0.5
        ros0 = e0['ros_time_s']
        late = self.left_start() - ros0     # 追従カメラの録画がまだ始まっていない部分は使わない
        if late > 0:
            t0, ros0 = t0 + late, ros0 + late
        if t1 - t0 < 1.0:
            return
        sp = speed_for(t1 - t0)
        seg = {'name': cap['text'][0], 'video': self.a.screen, 'start': round(t0, 2),
               'end': round(t1, 2), 'speed': sp,
               'layers': self.layers(t0, ros0), 'captions': [cap]}
        if sp > 1:
            seg['captions'].append({'style': 'badge', 'text': '%g× Speed' % sp})
        if lower:
            seg['captions'].append({'style': 'lower', 'at': 0, 'dur': 5, 'text': lower})
        self.segments.append(seg)

    def build(self):
        a, ev = self.a, self.events_by_kind()
        cards = a.cards
        for name, dur in (('01_title', 7), ('02_background', 7), ('03_ai_risk', 12),
                          ('04_system', 7)):
            self.segments.append({'name': name, 'image': os.path.join(cards, name + '.png'),
                                  'duration': dur})
        start = ev['mission_started'][0]
        t = self.st(start)
        if t >= 4:     # ROS 2 で動いている証拠として、全画面を数秒
            self.segments.append({
                'name': '起動(全画面)', 'video': a.screen, 'start': round(t - 4, 2),
                'end': round(t + 1, 2), 'layers': [{'box': [0, 120, 1920, 960],
                                                    'size': a.screen_size}],
                'captions': [{'style': 'mission', 'text': [
                    'ROS 2 で起動', 'Gazebo（シミュレーション）・Navigation2・AI ノード']}]})
        seq = self.ev
        total = len(start['detail'].get('route', [])) or '?'
        idx, pid, risk = 0, '', 0.0
        for i, e in enumerate(seq[:-1]):
            nxt = seq[i + 1]
            k = e['event']
            if k == 'navigate':
                idx, pid = e['index'], e['point_id']
                risk = e['detail'].get('risk', 0.0)
                jp, _ = self.names.get(pid, ('', ''))
                self.add_move(e, nxt, self.mission(idx, total, pid, risk, 'Navigating'),
                              ['AI が選んだ %s 番目の地点 %s（%s）へ自律走行' % (idx, pid, jp)])
            elif k == 'bear_detected':
                d = e['detail']
                t0 = self.st(e) - 3.0
                seg = {'name': '熊を検知 %s' % pid, 'video': a.screen, 'start': round(t0, 2),
                       'end': round(self.st(e) + 6.0, 2),
                       'layers': self.layers(t0, e['ros_time_s'] - 3.0, left='camera'),
                       'captions': [
                           dict(self.mission(idx, total, pid, risk, 'Navigating'), at=0, dur=3.0),
                           dict(self.mission(idx, total, pid, risk, 'BEAR - stopped'), at=3.0),
                           {'style': 'label', 'text': 'ロボットのカメラ（YOLOv8）' if a.camera
                            else 'Gazebo', 'pos': [40, 134]}],
                       'beeps': [3.0]}
                if not a.camera:   # カメラ映像には HUD の警告が入っているので重ねない
                    seg['captions'].append({'style': 'alert', 'at': 3.0, 'dur': 6.0, 'center_x': 590,
                                            'text': ['BEAR DETECTED',
                                                     'Confidence: %.2f   Distance: %.1f m' % (
                                                         d.get('confidence', 0), d.get('distance_m', 0)),
                                                     'Location: Patrol point %s' % pid,
                                                     'Action: %s' % d.get('action', 'Stop and report')]})
                self.segments.append(seg)
            elif k == 'retreat':
                if nxt['event'] == 'retreat_failed':
                    self.add_move(e, nxt, self.mission(idx, total, pid, risk,
                                                       'Retreating - no path to safe point'),
                                  ['退避先への経路が見つからず（建物のそば）→ 本試作は巡回を続行',
                                   '実運用では安全停止・代替退避地点の再探索・管理者への通知を優先すべき（今後の課題）'])
                    for c in self.segments[-1]['captions']:   # 安全上の課題なので場面の最後まで出す
                        if c['style'] == 'lower':
                            c.pop('dur', None)
                else:
                    self.add_move(e, nxt, self.mission(idx, total, pid, risk,
                                                       'Retreating to safe point'),
                                  ['熊から離れた安全地点へ退避（その後、巡回を再開）'])
            elif k == 'arrived':
                t0 = self.st(e)
                self.segments.append({
                    'name': '到着 %s' % pid, 'video': a.screen, 'start': round(t0 - 1, 2),
                    'end': round(self.st(nxt) + 1, 2),
                    'layers': self.layers(t0 - 1, e['ros_time_s'] - 1),
                    'captions': [self.mission(idx, total, pid, risk, 'Arrived - observing'),
                                 {'style': 'lower', 'text': ['%s に到着 → 周囲を確認' % pid]}]})
            elif k == 'return_to_base':
                self.add_move(e, nxt, {'style': 'mission', 'text': ['KumaWatch Patrol',
                                                                    'Returning to base']},
                              ['全地点を確認し、詰所へ帰還'])
        done = ev.get('mission_completed', [None])[0]
        if done:
            t0 = self.st(done)
            self.segments.append({
                'name': 'RViz の結果', 'video': a.screen, 'start': round(t0, 2),
                'end': round(t0 + 5, 2),
                'layers': [self.right_layer(done['ros_time_s'], [360, 120, 1200, 900])],
                'captions': [{'style': 'mission', 'text': [
                    'Mission completed', '地図：走行軌跡と熊の発見位置' if a.map else 'RViz：巡回結果']}]})
        for name, dur in (('09_result', 7),):
            self.segments.append({'name': name, 'image': os.path.join(cards, name + '.png'),
                                  'duration': dur})
        if a.compare_anim:
            self.segments.append({'name': '比較アニメーション', 'video': a.compare_anim, 'start': 0,
                                  'end': 16, 'size': [1920, 1080]})
        if os.path.exists(os.path.join(cards, '08_compare.png')):
            self.segments.append({'name': '比較', 'image': os.path.join(cards, '08_compare.png'),
                                  'duration': 8})
        self.segments.append({'name': 'まとめ', 'image': os.path.join(cards, '10_closing.png'),
                              'duration': 6})
        return self.segments

    def events_by_kind(self):
        out = {}
        for e in self.ev:
            out.setdefault(e['event'], []).append(e)
        if 'mission_started' not in out:
            raise SystemExit('mission_events.csv に mission_started がありません')
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run', required=True, help='mission_planner の結果フォルダ')
    ap.add_argument('--screen', required=True, help='record_screen.sh の画面録画(.json 付き)')
    ap.add_argument('--camera', help='image_recorder の検出枠入り映像(.json 付き)')
    ap.add_argument('--chase', help='chase_camera の追従映像(.json 付き)')
    ap.add_argument('--map', help='live_map.py の地図の映像(.json 付き)。RViz の代わりに右側に置く')
    ap.add_argument('--gazebo-crop', type=crop_arg, help='画面録画の中の Gazebo の範囲 x,y,幅,高さ')
    ap.add_argument('--rviz-crop', type=crop_arg, help='画面録画の中の RViz の範囲 x,y,幅,高さ')
    ap.add_argument('--cards', default='video_work/cards')
    ap.add_argument('--compare-anim', help='route_animation.py で作った比較アニメーション')
    ap.add_argument('--out', default='video_work/final.yaml')
    ap.add_argument('--frame-at', type=float, help='この秒の画面を PNG に書き出して終わる(範囲の確認用)')
    args = ap.parse_args()

    probe = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
                            'stream=width,height', '-of', 'csv=p=0', args.screen],
                           capture_output=True, text=True, check=True).stdout.strip()
    args.screen_size = [int(v) for v in probe.split(',')[:2]]
    if args.frame_at is not None:
        png = os.path.splitext(args.out)[0] + '_frame.png'
        os.makedirs(os.path.dirname(os.path.abspath(png)), exist_ok=True)
        subprocess.run(['ffmpeg', '-v', 'error', '-y', '-ss', str(args.frame_at), '-i', args.screen,
                        '-frames:v', '1', png], check=True)
        print('書き出しました: %s(%dx%d)。画像を見て Gazebo と RViz の範囲を決めてください'
              % (png, *args.screen_size))
        return
    w, h = args.screen_size
    args.gazebo_crop = args.gazebo_crop or [0, 0, int(w * 0.6), h]
    args.rviz_crop = args.rviz_crop or [int(w * 0.6), 0, w - int(w * 0.6), h]

    with open(os.path.join(ROOT, 'src', 'kumawatch_ai', 'data', 'patrol_points.yaml'),
              encoding='utf-8') as f:
        pts = yaml.safe_load(f)['points']
    names = {p['id']: (p['name'], p.get('name_en', '')) for p in pts}
    events = read_events(args.run)
    b = Builder(args, events, names)
    segs = b.build()
    out_dir = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(out_dir, exist_ok=True)
    for s in segs:   # compose.py は編集リストの場所からの相対パスも読めるが、ここでは絶対パスにする
        for k in ('image', 'video'):
            if k in s:
                s[k] = os.path.abspath(s[k])
        for lay in s.get('layers', []):
            if 'video' in lay:
                lay['video'] = os.path.abspath(lay['video'])
    edl = {'output': os.path.abspath(os.path.splitext(args.out)[0] + '.mp4'), 'fps': 30,
           'segments': segs}
    with open(args.out, 'w', encoding='utf-8') as f:
        yaml.safe_dump(edl, f, allow_unicode=True, sort_keys=False, width=120)
    total = sum(s.get('duration', (s.get('end', 0) - s.get('start', 0)) / s.get('speed', 1))
                for s in segs)
    print('編集リストを作りました: %s(%d 場面、約 %.0f 秒)' % (args.out, len(segs), total))
    print('動画にする: python3 scripts/video/compose.py %s' % args.out)


if __name__ == '__main__':
    sys.exit(main())
