#!/usr/bin/env python3
"""提出動画用の説明画面(1920×1080 の PNG)を作る.

  python3 scripts/video/make_cards.py --out video_work/cards
  python3 scripts/video/make_cards.py --out video_work/cards \\
      --risk-csv src/kumawatch_ai/data/simulated_risk_scores.csv   # 撮影時と同じリスクで作る
  python3 scripts/video/make_cards.py --out video_work/cards \\
      --summary ~/kumawatch_patrol_output/<日時>_utility/patrol_summary.json \\
      --compare ~/kumawatch_patrol_output/<日時>_fixed/patrol_summary.json

作る画面:
  01_title.png       タイトルと目的
  02_background.png  社会的背景と課題
  03_ai_risk.png     AI のリスク予測と、巡回する 3 地点の選択(表)
  04_system.png      ROS 2 のシステム構成
  08_compare.png     固定巡回と AI 巡回の比較(--compare を指定したとき。数値は実際の走行結果)
  09_result.png      Mission Completed(--summary を指定したとき)
  10_closing.png     まとめ
リスクの表・比較・結果の数値は、すべてファイルから読む(手で書かない)。
"""

import argparse
import csv
import json
import os
import sys

from PIL import Image, ImageDraw
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'src', 'kumawatch_mission'))
import captions as C  # noqa: E402
from kumawatch_mission.route_planning import distance, plan_patrol, utility  # noqa: E402

W, H = C.W, C.H
BG = (14, 19, 24)
FG = (240, 244, 248)
SUB = (160, 175, 190)
ACCENT = (255, 150, 60)       # AI(橙)
GOOD = (120, 220, 140)
BAD = (240, 90, 80)
LINE = (60, 72, 86)


def new():
    img = Image.new('RGB', (W, H), BG)
    return img, ImageDraw.Draw(img)


def header(d, title, sub=None):
    d.rectangle((0, 0, 14, H), fill=ACCENT)
    d.text((90, 70), title, font=C.font(64, 'bold'), fill=FG)
    if sub:
        d.text((92, 160), sub, font=C.font(36), fill=SUB)


def footer(d, text):
    d.text((90, H - 70), text, font=C.font(26), fill=SUB)


def read_scores(path):
    with open(path, encoding='utf-8') as f:
        rows = list(csv.DictReader(ln for ln in f if not ln.startswith('#')))
    return {r['point_id']: r for r in rows}


def card_title(out):
    img, d = new()
    d.rectangle((0, 0, W, 10), fill=ACCENT)
    lines = [('KumaWatch Patrol', C.font(120, 'bold'), FG),
             ('AI による熊出没リスク巡回ロボット', C.font(60, 'bold'), FG),
             ('ROS 2 × Gazebo × Navigation2 × AI（TTM・YOLOv8）', C.font(40), ACCENT),
             ('目的：限られた巡回資源で、リスクの高い地点を優先的に確認する', C.font(40), SUB)]
    y = 300
    for t, f, col in lines:
        w, h = C.text_size(d, t, f)
        d.text(((W - w) // 2, y), t, font=f, fill=col)
        y += h + (70 if f.size >= 100 else 50)
    img.save(os.path.join(out, '01_title.png'))


def card_background(out):
    img, d = new()
    header(d, '背景：熊の人里への出没', '限られた人員で、どこを見回るかを毎日決めなければならない')
    items = [
        ('課題', '通学路・ごみ集積所・林縁など、見回るべき場所は多いが人員は限られる'),
        ('予測', '研究 KumaWatch：公開出没データから、翌日の出没リスクを AI（TTM）で予測'),
        ('不足', '予測は地図上の数字。現場を確認し、熊がいたら安全に行動する部分は人任せ'),
        ('本作品', '予測 → 巡回地点の選択 → ロボットの自律巡回 → 熊の発見・停止・退避 までをつなぐ'),
    ]
    y = 300
    for k, v in items:
        d.rounded_rectangle((90, y, 330, y + 90), 14, fill=ACCENT if k == '本作品' else LINE)
        w, h = C.text_size(d, k, C.font(40, 'bold'))
        d.text((90 + (240 - w) // 2, y + 18), k, font=C.font(40, 'bold'),
               fill=BG if k == '本作品' else FG)
        d.text((370, y + 20), v, font=C.font(38), fill=FG)
        y += 140
    img.save(os.path.join(out, '02_background.png'))


def card_ai_risk(out, points_file, risk_csv, k=3, lam=0.02):
    with open(points_file, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    base = (float(cfg['base']['x']), float(cfg['base']['y']))
    scores = read_scores(risk_csv)
    model = next(iter(scores.values())).get('model', '')
    cands = []
    for p in cfg['points']:
        cands.append({'id': p['id'], 'name': p['name'], 'category': p['category'],
                      'x': p['x'], 'y': p['y'], 'risk': float(scores[p['id']]['risk']),
                      'importance_weight': p.get('importance_weight', 1.0)})
    plan = plan_patrol(cands, k, 'utility', base, lam, return_to_start=True)
    order = {c['id']: i + 1 for i, c in enumerate(plan['route'])}

    img, d = new()
    is_ttm = 'TTM' in model
    header(d, 'AI の判断：%d 地点から巡回する %d 地点を選ぶ' % (len(cands), k),
           ('AI（KumaWatch の TTM）が各地点の熊出没リスクを予測' if is_ttm
            else 'AI が各地点の熊出没リスクを推定（%s）' % model))
    cols = [('地点', 130), ('環境', 260), ('AI リスク p', 260), ('重要度 w', 200),
            ('距離 d', 190), ('効用 U', 200), ('判断', 380)]
    x0, y0, rh = 110, 270, 92
    x = x0
    for name, w in cols:
        d.text((x, y0), name, font=C.font(34, 'bold'), fill=SUB)
        x += w
    d.line((x0, y0 + 60, x0 + sum(w for _, w in cols) - 40, y0 + 60), fill=LINE, width=3)
    for i, c in enumerate(sorted(cands, key=lambda c: c['id'])):
        y = y0 + 80 + i * rh
        sel = c['id'] in order
        if sel:
            d.rounded_rectangle((x0 - 20, y - 12, x0 + sum(w for _, w in cols) - 30, y + rh - 22),
                                12, fill=(52, 38, 26))
        dist = distance(base, (c['x'], c['y']))
        u = utility(c, base, lam)
        cells = [c['id'], c['name'], '%.2f' % c['risk'], '%.1f' % c['importance_weight'],
                 '%.1f m' % dist, '%+.2f' % u,
                 '巡回 #%d' % order[c['id']] if sel else '対象外']
        x = x0
        for j, ((_, w), t) in enumerate(zip(cols, cells)):
            col = FG
            if j == 6:
                col = ACCENT if sel else SUB
            f = C.font(40, 'bold' if j in (0, 6) else 'regular')
            if j == 2:   # リスクの棒
                d.rectangle((x, y + 44, x + int(200 * c['risk']), y + 56),
                            fill=ACCENT if c['risk'] >= 0.7 else (90, 110, 130))
            d.text((x, y - 4), t, font=f, fill=col)
            x += w
    route = ' → '.join(['詰所'] + [c['id'] for c in plan['route']] + ['詰所'])
    d.text((110, y0 + 80 + len(cands) * rh + 20),
           '効用 U = w・p − λd（λ=%.2f）の上位 %d 地点を選び、移動距離が最短の順に回る：%s'
           % (lam, k, route), font=C.font(34, 'bold'), fill=FG)
    note = ('リスク = 山形県 10 km セル（実在）の TTM 予測値の県内順位（2025-06-07）。町と地点は架空'
            if is_ttm else 'リスクは模擬データで学習したモデルの推定値（町と地点は架空）')
    footer(d, note)
    img.save(os.path.join(out, '03_ai_risk.png'))
    return plan


def card_system(out):
    img, d = new()
    header(d, 'システム構成（ROS 2）', '橙の枠が AI。ノードはトピックと Nav2 のアクションでつながる')
    fig = Image.open(os.path.join(ROOT, 'docs', 'report', 'figures', 'system_diagram.png'))
    s = min((W - 160) / fig.width, 700 / fig.height)
    fig = fig.convert('RGB').resize((int(fig.width * s), int(fig.height * s)), Image.LANCZOS)
    x = (W - fig.width) // 2
    d.rounded_rectangle((x - 20, 250, x + fig.width + 20, 250 + fig.height + 40), 16,
                        fill=(255, 255, 255))
    img.paste(fig, (x, 270))
    footer(d, 'Gazebo（シミュレータ）＋ Navigation2（AMCL・経路計画）＋ AI 2 つ'
              '（TTM：リスク予測 / YOLOv8：カメラ画像の熊判定）')
    img.save(os.path.join(out, '04_system.png'))


def _checked(s):
    return s.get('num_checked', sum(1 for r in s['results'] if r['status'] in ('clear', 'bear_found')))


def incidental(s, points):
    """巡回対象でない地点の近くの熊を、別の地点から見つけた数と、その説明."""
    out = []
    for e in s.get('events', []):
        near = min(points, key=lambda p: (p['x'] - e['bear_x']) ** 2 + (p['y'] - e['bear_y']) ** 2)
        if near['id'] != e['point_id'] and near['id'] not in s['route']:
            out.append('%s から %s 付近の熊を偶然視認' % (e['point_id'], near['id']))
    return out


def card_compare(out, fixed_runs, ai_runs, note, points=None):
    """固定ルートと AI ルートの比較. 各側に 1 回以上の結果(複数なら平均)."""
    img, d = new()
    n = min(len(fixed_runs), len(ai_runs))
    header(d, '比較：固定ルート と AI ルート',
           '同じ %d 地点・同じ熊の配置で巡回%s' % (
               fixed_runs[0]['num_points'], '（各 %d 回の平均）' % n if n > 1 else ''))

    def mean(runs, f):
        return sum(f(r) for r in runs) / len(runs)
    rows = [('巡回地点の AI リスク合計', lambda r: sum(x['risk'] for x in r['results']), '%.2f', ''),
            ('確認できた地点', lambda r: _checked(r), '%.1f' if n > 1 else '%d', ' / %d' % fixed_runs[0]['num_points']),
            ('熊の発見数', lambda r: r['bears_found'], '%.1f' if n > 1 else '%d', ' 頭'),
            ('移動距離', lambda r: r['distance_m'], '%.1f', ' m'),
            ('所要時間', lambda r: r['duration_s'], '%.0f', ' 秒')]
    x0, xs = 160, (760, 1300)
    y = 300
    d.text((xs[0], y), '固定ルート', font=C.font(44, 'bold'), fill=SUB)
    d.text((xs[1], y), 'AI ルート', font=C.font(44, 'bold'), fill=ACCENT)
    d.text((xs[0], y + 60), ' → '.join(fixed_runs[0]['route']), font=C.font(34), fill=SUB)
    d.text((xs[1], y + 60), ' → '.join(ai_runs[0]['route']), font=C.font(34), fill=ACCENT)
    y += 130
    for name, f, fmt, unit in rows:
        d.line((x0, y - 20, W - 160, y - 20), fill=LINE, width=2)
        d.text((x0, y), name, font=C.font(44, 'bold'), fill=FG)
        a, b = mean(fixed_runs, f), mean(ai_runs, f)
        extra = 0
        higher = name in ('熊の発見数', '巡回地点の AI リスク合計') and b > a + 1e-9
        for xx, v, hi, runs in ((xs[0], a, False, fixed_runs), (xs[1], b, higher, ai_runs)):
            d.text((xx, y - 6), (fmt % v) + unit, font=C.font(60 if hi else 52, 'bold'),
                   fill=GOOD if hi else FG)
            if name == '熊の発見数' and points and n == 1:
                inc = incidental(runs[0], points)
                if inc:
                    d.text((xx, y + 60), 'うち %d 頭は %s' % (len(inc), '、'.join(inc)),
                           font=C.font(28, 'bold'), fill=(255, 200, 0))
                    extra = 40
        y += 100 + extra
    for i, line in enumerate(note.split('\n')[::-1]):
        d.text((90, H - 70 - 40 * i), line, font=C.font(26), fill=SUB)
    img.save(os.path.join(out, '08_compare.png'))


def card_result(out, s):
    img, d = new()
    d.text((110, 90), 'Mission Completed', font=C.font(96, 'bold'), fill=GOOD)
    d.text((114, 220), '巡回ミッション完了（strategy=%s）' % s['strategy'], font=C.font(40), fill=SUB)
    checked = s.get('num_checked', sum(1 for r in s['results']
                                        if r['status'] in ('clear', 'bear_found')))
    items = [('選んだ巡回地点', '%d（%s）' % (s['num_points'], ' → '.join(s['route']))),
             ('確認できた地点', '%d / %d' % (checked, s['num_points'])),
             ('熊の発見', '%d 頭（停止して位置・時刻・信頼度を記録）' % s['bears_found']),
             ('移動距離', '%.1f m' % s['distance_m']),
             ('所要時間', '%.0f 秒（シミュレーション時間）' % s['duration_s'])]
    y = 340
    for k, v in items:
        d.text((150, y), '・' + k, font=C.font(46, 'bold'), fill=FG)
        d.text((700, y), v, font=C.font(46), fill=FG)
        y += 100
    img.save(os.path.join(out, '09_result.png'))


def card_closing(out):
    img, d = new()
    t1 = 'AI の予測を、ロボットの具体的な巡回行動へ'
    t2 = '予測（TTM）→ 地点選択 → 自律巡回（Nav2）→ 熊の発見（YOLOv8）・停止・退避'
    t3 = '今後：実環境の画像での熊判定、複数日・複数配置での評価、他の予測モデルとの比較'
    y = 330
    for t, f, col in ((t1, C.font(72, 'bold'), FG), (t2, C.font(40), ACCENT),
                      (t3, C.font(34), SUB)):
        w, h = C.text_size(d, t, f)
        d.text(((W - w) // 2, y), t, font=f, fill=col)
        y += h + 80
    w, _ = C.text_size(d, 'KumaWatch Patrol', C.font(40, 'bold'))
    d.text(((W - w) // 2, H - 150), 'KumaWatch Patrol', font=C.font(40, 'bold'), fill=SUB)
    img.save(os.path.join(out, '10_closing.png'))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', default='video_work/cards')
    ap.add_argument('--points', default=os.path.join(ROOT, 'src', 'kumawatch_ai', 'data',
                                                      'patrol_points.yaml'))
    ap.add_argument('--risk-csv', default=os.path.join(ROOT, 'src', 'kumawatch_ai', 'data',
                                                        'kumawatch_ttm_scores.csv'))
    ap.add_argument('--summary', help='AI 巡回の patrol_summary.json(結果画面)')
    ap.add_argument('--compare', nargs='+', help='固定巡回の patrol_summary.json(比較画面。複数なら平均)')
    ap.add_argument('--compare-ai', nargs='+', help='比較に使う AI 巡回の結果(省略時は --summary)')
    ap.add_argument('--compare-note', default='このシミュレーション条件（1 日の予測・1 つの熊の配置）での結果。'
                    'AI が一般に優れていることを示すものではない')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    card_title(args.out)
    card_background(args.out)
    card_ai_risk(args.out, args.points, args.risk_csv)
    card_system(args.out)
    if args.summary:
        with open(args.summary, encoding='utf-8') as f:
            ai = json.load(f)
        card_result(args.out, ai)
        if args.compare:
            fixed = [json.load(open(p, encoding='utf-8')) for p in args.compare]
            ais = [json.load(open(p, encoding='utf-8')) for p in args.compare_ai] \
                if args.compare_ai else [ai]
            with open(args.points, encoding='utf-8') as f:
                pts = yaml.safe_load(f)['points']
            card_compare(args.out, fixed, ais, args.compare_note, pts)
    card_closing(args.out)
    print('作成しました: %s' % ', '.join(sorted(os.listdir(args.out))))


if __name__ == '__main__':
    main()
