#!/usr/bin/env python3
"""編集リスト(YAML)から提出用の動画を組み立てる(ffmpeg).

画面録画の一部を切り出して拡大・並べ替え・倍速にし、タイトルや表の画像、字幕、
熊検知の警告パネル、効果音を入れて 1 本の MP4(1920×1080, H.264)にする。

  python3 scripts/video/compose.py edit.yaml
  python3 scripts/video/compose.py edit.yaml --only 3     # 3 番目の場面だけ作る(確認用)

編集リストの書き方(例):

  output: KumaWatch_Patrol_demo.mp4
  segments:
    - image: cards/01_title.png        # 静止画(タイトル・表など)
      duration: 8
    - video: screen.mp4                # 画面録画から切り出す
      start: 5.3                       # 元動画の秒
      end: 59.0
      speed: 2.0                       # 2 倍速(字幕の時刻は仕上がりの秒)
      layers:                          # 元動画の範囲 crop=[x, y, 幅, 高さ] を box に拡大して置く
        - {crop: [1033, 240, 1075, 685], box: [0, 120, 1150, 733]}
        - {crop: [2620, 235, 792, 820], box: [1160, 120, 750, 777]}
      captions:                        # at/dur を省くと場面全体
        - {style: mission, text: ["Mission 1/3: P2 Forest edge", "AI risk 0.85 | Navigating"]}
        - {style: badge, text: "2× Speed"}
        - {style: lower, at: 0, dur: 6, text: ["林縁 P2 へ自律走行", "Navigation2 が経路を計画"]}
      beeps: [3.0]                     # 効果音(仕上がりの秒)

字幕のスタイルは captions.py を参照。
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import captions  # noqa: E402

W, H = captions.W, captions.H


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        sys.stderr.write(' '.join(cmd) + '\n' + r.stderr[-3000:])
        raise SystemExit('ffmpeg に失敗しました')


def fit(src_w, src_h, box):
    """box=[x, y, w, h] に縦横比を保って収めたときの (x, y, w, h)(偶数に丸める)."""
    bx, by, bw, bh = box
    s = min(bw / src_w, bh / src_h)
    w, h = int(src_w * s) // 2 * 2, int(src_h * s) // 2 * 2
    return bx + (bw - w) // 2, by + (bh - h) // 2, w, h


def caption_inputs(seg, dur, tmp, idx):
    """字幕 PNG を書き出し、(ファイル, 開始秒, 終了秒) のリストを返す."""
    out = []
    for j, c in enumerate(seg.get('captions', [])):
        path = os.path.join(tmp, 'cap_%03d_%02d.png' % (idx, j))
        captions.render(c).save(path)
        a = float(c.get('at', 0.0))
        b = min(dur, a + float(c['dur'])) if 'dur' in c else dur
        out.append((path, a, b))
    return out


def build_segment(seg, idx, cfg, tmp):
    fps = cfg['fps']
    bg = cfg['background']
    out = os.path.join(tmp, 'seg_%03d.mp4' % idx)
    inputs, filters = [], []
    if 'image' in seg:
        dur = float(seg['duration'])
        inputs += ['-loop', '1', '-framerate', str(fps), '-t', '%.3f' % dur, '-i', seg['image']]
        filters.append('color=c=%s:s=%dx%d:r=%d:d=%.3f[bg]' % (bg, W, H, fps, dur))
        filters.append('[0:v]scale=%d:%d:force_original_aspect_ratio=decrease[img]' % (W, H))
        filters.append('[bg][img]overlay=(W-w)/2:(H-h)/2:shortest=1[c0]')
        last = 'c0'
    else:
        start, end = float(seg['start']), float(seg['end'])
        speed = float(seg.get('speed', 1.0))
        dur = (end - start) / speed
        layers = seg.get('layers') or [{'crop': None, 'box': [0, 0, W, H]}]
        filters.append('color=c=%s:s=%dx%d:r=%d:d=%.3f[bg]' % (bg, W, H, fps, dur))
        last = 'bg'
        for i, lay in enumerate(layers):
            # 層ごとに別の動画も使える(video と、その動画での開始秒 start)
            src = lay.get('video', seg.get('video'))
            lay_start = float(lay.get('start', start))
            inputs += ['-ss', '%.3f' % max(0.0, lay_start), '-t', '%.3f' % (end - start),
                       '-i', src]
            crop = lay.get('crop')
            if crop:
                src_w, src_h = crop[2], crop[3]
            else:
                src_w, src_h = lay.get('size', seg.get('size', (W, H)))  # 元動画の大きさ
            x, y, w, h = fit(src_w, src_h, lay['box'])
            chain = '[%d:v]setpts=(PTS-STARTPTS)/%.4f,fps=%d,' % (i, speed, fps)
            chain += 'crop=%d:%d:%d:%d,' % (crop[2], crop[3], crop[0], crop[1]) if crop else ''
            chain += 'scale=%d:%d[l%d]' % (w, h, i)
            filters.append(chain)
            filters.append('[%s][l%d]overlay=%d:%d:eof_action=repeat[c%d]' % (last, i, x, y, i))
            last = 'c%d' % i
    n_in = inputs.count('-i')        # 字幕 PNG の入力番号はこの後から
    for path, a, b in caption_inputs(seg, dur, tmp, idx):
        inputs += ['-loop', '1', '-framerate', str(fps), '-t', '%.3f' % dur, '-i', path]
        filters.append("[%s][%d:v]overlay=0:0:enable='between(t,%.3f,%.3f)'[k%d]" % (
            last, n_in, a, b, n_in))
        last = 'k%d' % n_in
        n_in += 1
    fade = float(seg.get('fade', cfg['fade']))
    tail = '[%s]format=yuv420p' % last
    if fade > 0:
        tail += ',fade=t=in:st=0:d=%.2f,fade=t=out:st=%.3f:d=%.2f' % (
            fade, max(0.0, dur - fade), fade)
    filters.append(tail + '[out]')
    run(['ffmpeg', '-v', 'error', '-y'] + inputs + [
        '-filter_complex', ';'.join(filters), '-map', '[out]', '-t', '%.3f' % dur,
        '-r', str(fps), '-c:v', 'libx264', '-preset', cfg['preset'], '-crf', str(cfg['crf']),
        '-pix_fmt', 'yuv420p', '-an', out])
    return out, dur


def add_audio(video, total, beeps, out):
    """無音の音声トラックに、指定時刻の効果音(短いビープ 2 回)を重ねて多重化する."""
    inputs = ['-i', video, '-f', 'lavfi', '-t', '%.3f' % total, '-i',
              'anullsrc=channel_layout=stereo:sample_rate=48000']
    filters, labels = [], ['[1:a]']
    for i, t in enumerate(beeps):
        inputs += ['-f', 'lavfi', '-t', '0.55', '-i',
                   "aevalsrc='0.35*sin(2*PI*1320*t)*(lt(mod(t,0.28),0.16))':s=48000:c=stereo"]
        ms = int(t * 1000)
        filters.append('[%d:a]adelay=%d|%d[b%d]' % (i + 2, ms, ms, i))
        labels.append('[b%d]' % i)
    filters.append('%samix=inputs=%d:duration=first:normalize=0[a]' % (''.join(labels),
                                                                        len(labels)))
    run(['ffmpeg', '-v', 'error', '-y'] + inputs + [
        '-filter_complex', ';'.join(filters), '-map', '0:v', '-map', '[a]', '-c:v', 'copy',
        '-c:a', 'aac', '-b:a', '128k', '-shortest', out])


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('edl')
    ap.add_argument('--only', type=int, help='この番号(1 始まり)の場面だけ作る')
    ap.add_argument('-o', '--output')
    args = ap.parse_args()
    with open(args.edl, encoding='utf-8') as f:
        edl = yaml.safe_load(f)
    base = os.path.dirname(os.path.abspath(args.edl))
    cfg = {'fps': int(edl.get('fps', 30)), 'background': edl.get('background', '#0e1318'),
           'fade': float(edl.get('fade', 0.25)), 'crf': int(edl.get('crf', 20)),
           'preset': edl.get('preset', 'medium')}
    segs = edl['segments']
    for s in segs:   # 相対パスは編集リストの場所から
        for k in ('image', 'video'):
            if k in s and not os.path.isabs(s[k]):
                s[k] = os.path.join(base, s[k])
    out = args.output or os.path.join(base, edl.get('output', 'demo.mp4'))
    tmp = tempfile.mkdtemp(prefix='kw_video_')
    try:
        parts, beeps, t = [], [], 0.0
        for i, s in enumerate(segs, 1):
            if args.only and i != args.only:
                continue
            path, dur = build_segment(s, i, cfg, tmp)
            print('場面 %2d: %5.1f 秒  %s' % (i, dur, s.get('name', s.get('image', s.get('video')))))
            beeps += [t + float(b) for b in s.get('beeps', [])]
            parts.append(path)
            t += dur
        lst = os.path.join(tmp, 'list.txt')
        with open(lst, 'w') as f:
            f.writelines("file '%s'\n" % p for p in parts)
        joined = os.path.join(tmp, 'joined.mp4')
        run(['ffmpeg', '-v', 'error', '-y', '-f', 'concat', '-safe', '0', '-i', lst,
             '-c', 'copy', joined])
        add_audio(joined, t, beeps, out)
        print('保存しました: %s(%.1f 秒)' % (out, t))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    main()
