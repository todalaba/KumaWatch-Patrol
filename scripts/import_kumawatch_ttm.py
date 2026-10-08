#!/usr/bin/env python3
"""KumaWatch の TTM 予測スコアを、KumaWatch Patrol の 5 地点に割り当てる.

研究プロジェクト KumaWatch(非公開の研究リポジトリ。実行にはそのチェックアウトが必要)にある
IBM Granite TinyTimeMixer(TTM)のゼロショット予測(山形県 10km グリッド 144 セル × 2025 年 365 日)から、
指定した 1 日分を取り出し、次の規則で 5 地点にセルを割り当てる。

  - その日に実際に熊の出没があったセル(TTM の順位が高い 2 つ)→ P4(ごみ集積所)と P2(林縁)
    この 2 地点には Gazebo ワールドで熊を配置している(= 実際の出没を再現)
  - 出没がなかったセルのうち
      TTM 順位が最も高いセル  → P3(通学路)   … AI の「空振り」も含める
      TTM 順位が 40 位に最も近いセル  → P5(河川沿い)
      TTM 順位が 100 位に最も近いセル → P1(住宅地)

地図と地点名は架空で、各地点の「リスク」だけが実在のセルの TTM 予測値である。
リスクは TTM スコアのその日の県内順位を百分位に直した値(1.0 = 144 セル中 1 位)。
TTM スコアは出没件数の予測値で確率ではないため、順位で比較できる形にしている。

使い方:
  python3 scripts/import_kumawatch_ttm.py --kumawatch-dir ../KumaWatch --date 2025-06-07
"""

import argparse
import csv
import datetime
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, 'src', 'kumawatch_ai', 'data', 'kumawatch_ttm_scores.csv')
MODEL = 'IBM Granite TTM 1536-96-R2 (KumaWatch, zero-shot)'


def parse_date(s):
    s = s.strip()[:10].replace('/', '-')
    y, m, d = (int(v) for v in s.split('-'))
    return datetime.date(y, m, d)


def load_wide(path, date):
    """wide 形式 CSV(Date + セル列)から指定日の {cell: 値} を返す."""
    with open(path, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            if parse_date(row['Date']) == date:
                return {k: float(v) for k, v in row.items() if k[:1].isdigit() and '_' in k}
    raise SystemExit('%s に %s の行がありません' % (path, date))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--kumawatch-dir', default=os.path.join(os.path.dirname(ROOT), 'KumaWatch'))
    ap.add_argument('--date', default='2025-06-07')
    ap.add_argument('--out', default=OUT)
    args = ap.parse_args()

    kw = args.kumawatch_dir
    date = parse_date(args.date)
    if date.year != 2025:
        sys.exit('TTM スコアは 2025 年(評価期間)のみです')
    scores = load_wide(os.path.join(kw, 'data', 'scores', 'yamagata_ttm_scores_2025.csv'), date)
    observed = load_wide(os.path.join(kw, 'data', 'yamagata_10km_daily_timeseries.csv'), date)

    cells = sorted(scores, key=lambda c: (-scores[c], c))
    # 同点(TTM スコアが 0 のセルが多い)は平均順位にする
    rank = {}
    i = 0
    while i < len(cells):
        j = i
        while j + 1 < len(cells) and scores[cells[j + 1]] == scores[cells[i]]:
            j += 1
        for k in range(i, j + 1):
            rank[cells[k]] = (i + j) / 2.0 + 1.0
        i = j + 1
    n = len(cells)
    positives = [c for c in cells if observed.get(c, 0) > 0]
    negatives = [c for c in cells if observed.get(c, 0) == 0]
    if len(positives) < 2:
        sys.exit('%s は出没セルが %d 個しかありません(2 個以上の日を選んでください)'
                 % (date, len(positives)))

    def nearest_rank(target, pool):
        return min(pool, key=lambda c: (abs(rank[c] - target), rank[c]))

    assign = {'P4': positives[0], 'P2': positives[1]}
    pool = list(negatives)
    assign['P3'] = pool.pop(0)
    assign['P5'] = nearest_rank(40, pool)
    pool.remove(assign['P5'])
    assign['P1'] = nearest_rank(100, pool)

    fields = ['point_id', 'risk', 'model', 'date', 'cell_id', 'ttm_score', 'ttm_rank', 'n_cells',
              'observed_sightings']
    with open(args.out, 'w', newline='', encoding='utf-8') as f:
        f.write('# KumaWatch TTM predictions (Yamagata 10 km grid, %s) mapped to fictional patrol '
                'points by scripts/import_kumawatch_ttm.py. Data: KumaWatch, CC BY 4.0.\n' % date)
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for pid in sorted(assign):
            c = assign[pid]
            w.writerow({
                'point_id': pid,
                'risk': '%.4f' % (1.0 - (rank[c] - 1) / (n - 1)),
                'model': MODEL,
                'date': date.isoformat(),
                'cell_id': c,
                'ttm_score': '%.5f' % scores[c],
                'ttm_rank': '%g' % rank[c],
                'n_cells': n,
                'observed_sightings': int(observed.get(c, 0)),
            })
    print('%s: 出没セル %d 個 / %d セル' % (date, len(positives), n))
    print('  地点  セル    TTM順位  risk    実際の出没')
    for pid in sorted(assign):
        c = assign[pid]
        print('  %s   %-6s  %5.1f   %.3f   %d' % (pid, c, rank[c], 1.0 - (rank[c] - 1) / (n - 1),
                                                 int(observed.get(c, 0))))
    print('→', args.out)


if __name__ == '__main__':
    main()
