#!/usr/bin/env python3
"""ROS なしで巡回ミッションを机上シミュレーションする(動作確認・比較表・図の作成用).

mission_planner と同じ地点選択ロジック(route_planning.py)で巡回ルートを作り、
生成済みの占有格子地図上の A* 経路に沿ってロボットを動かして、
bear_proximity_monitor と同じ判定(熊から detection_radius 以内)で熊の発見を数える。

Gazebo + Nav2 の実走行とは経路・距離が一致しない(Nav2 は DWB で滑らかに走る)ため、
結果はあくまで見積もり。実験結果には Gazebo での実走行の値を使うこと。

使い方:
  python3 scripts/offline_dry_run.py                 # utility / topk / fixed の比較表
  python3 scripts/offline_dry_run.py --plot docs/overview.png
"""

import argparse
import heapq
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for pkg in ('kumawatch_ai', 'kumawatch_mission', 'kumawatch_gazebo'):
    sys.path.insert(0, os.path.join(ROOT, 'src', pkg))

from kumawatch_ai.risk_model import load_patrol_points, read_risk_scores  # noqa: E402
from kumawatch_gazebo import world_builder as wb  # noqa: E402
from kumawatch_mission.route_planning import nearest_within, plan_patrol  # noqa: E402

LAYOUT = os.path.join(ROOT, 'src', 'kumawatch_gazebo', 'config', 'world_layout.yaml')
DATA = os.path.join(ROOT, 'src', 'kumawatch_ai', 'data')


class GridPlanner:
    """占有格子上の 8 近傍 A*(ロボット半径ぶん障害物を膨張)."""

    def __init__(self, layout, inflate_m=0.35):
        self.layout = layout
        self.res = layout.get('map_resolution', 0.05)
        self.b = layout['bounds']
        # 熊・シカも走行時には障害物(地図にはないが LiDAR で見える)
        grid = wb.rasterize(layout, extra_shapes=[
            wb.animal_footprint(a) for a in layout.get('animals', [])])
        self.h = len(grid)
        self.w = len(grid[0])
        n = int(math.ceil(inflate_m / self.res))
        occ = [[False] * self.w for _ in range(self.h)]
        offsets = [(dr, dc) for dr in range(-n, n + 1) for dc in range(-n, n + 1)
                   if dr * dr + dc * dc <= n * n]
        for r in range(self.h):
            for c in range(self.w):
                if grid[r][c]:
                    for dr, dc in offsets:
                        rr, cc = r + dr, c + dc
                        if 0 <= rr < self.h and 0 <= cc < self.w:
                            occ[rr][cc] = True
        self.occ = occ

    def cell(self, x, y):
        return (int((y - self.b['y_min']) / self.res), int((x - self.b['x_min']) / self.res))

    def xy(self, cell):
        r, c = cell
        return (self.b['x_min'] + (c + 0.5) * self.res, self.b['y_min'] + (r + 0.5) * self.res)

    def _nearest_free(self, cell):
        r0, c0 = cell
        for rad in range(0, 40):
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    r, c = r0 + dr, c0 + dc
                    if 0 <= r < self.h and 0 <= c < self.w and not self.occ[r][c]:
                        return (r, c)
        raise RuntimeError('no free cell near %s' % (cell,))

    def path(self, start, goal):
        s = self._nearest_free(self.cell(*start))
        g = self._nearest_free(self.cell(*goal))
        moves = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
                 (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                 (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]
        dist = {s: 0.0}
        prev = {}
        heap = [(0.0, s)]
        while heap:
            _, cur = heapq.heappop(heap)
            if cur == g:
                break
            d0 = dist[cur]
            for dr, dc, cost in moves:
                nb = (cur[0] + dr, cur[1] + dc)
                if not (0 <= nb[0] < self.h and 0 <= nb[1] < self.w) or self.occ[nb[0]][nb[1]]:
                    continue
                nd = d0 + cost
                if nd < dist.get(nb, float('inf')):
                    dist[nb] = nd
                    prev[nb] = cur
                    hh = math.hypot(nb[0] - g[0], nb[1] - g[1])
                    heapq.heappush(heap, (nd + hh, nb))
        if g not in dist:
            return None
        cells = [g]
        while cells[-1] != s:
            cells.append(prev[cells[-1]])
        return [start] + [self.xy(c) for c in reversed(cells)] + [goal]


def simulate(plan, planner, bears, base, radius, return_to_base=True):
    """経路に沿って進み、熊から radius 以内に入ったらその地点の巡回を打ち切る."""
    pos = (base['x'], base['y'])
    found = []
    handled = set()
    distance = 0.0
    trace = [pos]
    legs = [(c['id'], (c['x'], c['y'])) for c in plan['route']]
    if return_to_base:
        legs.append(('base', (base['x'], base['y'])))
    results = []
    for pid, goal in legs:
        path = planner.path(pos, goal)
        status = 'unreachable' if path is None else 'clear'
        for a, b in zip(path or [], (path or [])[1:]):
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            steps = max(1, int(seg / 0.05))
            stop = False
            for i in range(1, steps + 1):
                p = (a[0] + (b[0] - a[0]) * i / steps, a[1] + (b[1] - a[1]) * i / steps)
                distance += seg / steps
                pos = p
                hit = nearest_within(p, bears, radius)
                if hit and hit[0]['id'] not in handled:
                    handled.add(hit[0]['id'])
                    found.append({'point_id': pid, 'bear_id': hit[0]['id'],
                                  'distance_m': round(hit[1], 2), 'at': p})
                    status = 'bear_found'
                    stop = True
                    break
            trace.append(pos)
            if stop:
                break
        if pid != 'base':
            results.append((pid, status))
    return {'results': results, 'found': found, 'distance_m': distance, 'trace': trace}


def candidates_from_data():
    points = load_patrol_points(os.path.join(DATA, 'patrol_points.yaml'))
    risks = read_risk_scores(os.path.join(DATA, 'kumawatch_ttm_scores.csv'))
    return [dict(p, risk=risks[p['id']]) for p in points]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--k', type=int, default=3)
    ap.add_argument('--lam', type=float, default=0.02)
    ap.add_argument('--radius', type=float, default=2.5)
    ap.add_argument('--fixed', default='P1,P2,P3')
    ap.add_argument('--plot', help='俯瞰図(PNG)の出力先')
    args = ap.parse_args()

    layout = wb.load_yaml(LAYOUT)
    base = wb.load_yaml(os.path.join(DATA, 'patrol_points.yaml'))['base']
    cands = candidates_from_data()
    bears = wb.bears(layout)
    planner = GridPlanner(layout)
    start = (base['x'], base['y'])

    runs = {}
    for strategy in ('utility', 'topk', 'fixed'):
        plan = plan_patrol(cands, args.k, strategy, start, args.lam,
                           fixed_ids=args.fixed.split(','), return_to_start=True)
        runs[strategy] = (plan, simulate(plan, planner, bears, base, args.radius))

    print('机上シミュレーション (K=%d, λ=%.3f, 検知半径 %.1f m, A* 経路, 詰所へ帰還込み)' % (
        args.k, args.lam, args.radius))
    print('%-8s %-16s %-26s %6s %9s' % ('strategy', 'route', 'result', 'bears', 'dist[m]'))
    for strategy, (plan, sim) in runs.items():
        print('%-8s %-16s %-26s %6d %9.2f' % (
            strategy, '→'.join(c['id'] for c in plan['route']),
            ' '.join('%s:%s' % (p, 'BEAR' if s == 'bear_found' else s) for p, s in sim['results']),
            len(sim['found']), sim['distance_m']))

    if args.plot:
        plot(args.plot, layout, base, cands, runs)
        print('図: %s' % args.plot)


def plot(path, layout, base, cands, runs):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import patches
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(13, 6.6))
    for ax, strategy in zip(axes, ('utility', 'fixed')):
        plan, sim = runs[strategy]
        b = layout['bounds']
        ax.set_facecolor((0.30, 0.45, 0.22))
        for s in layout.get('surfaces', []):
            ax.add_patch(patches.Rectangle((s['x_min'], s['y_min']), s['x_max'] - s['x_min'],
                                           s['y_max'] - s['y_min'], color=s['color']))
        for shape in wb.static_obstacles(layout):
            if shape[0] == 'circle':
                ax.add_patch(patches.Circle(shape[1:3], shape[3] * 2, color=(0.10, 0.38, 0.12)))
            else:
                _, cx, cy, sx, sy, yaw = shape
                if sx > 1.6 and sy > 15:  # 河川の進入禁止領域は地面色で表示済み
                    continue
                r = patches.Rectangle((cx - sx / 2, cy - sy / 2), sx, sy,
                                      angle=math.degrees(yaw), rotation_point='center',
                                      color=(0.85, 0.82, 0.75), ec='k', lw=0.4)
                ax.add_patch(r)
        tx, ty = zip(*sim['trace'])
        ax.plot(tx, ty, '-', color=(0.2, 0.7, 1.0), lw=2.2)
        cmap = plt.get_cmap('RdYlGn_r')
        sel = {c['id'] for c in plan['route']}
        for c in cands:
            ax.scatter(c['x'], c['y'], s=260, color=cmap(c['risk']), ec='w' if c['id'] in sel
                       else 'k', lw=2.5 if c['id'] in sel else 0.8, zorder=5)
            ax.annotate('%s %.2f' % (c['id'], c['risk']), (c['x'], c['y']),
                        xytext=(8, 8), textcoords='offset points', color='w', fontsize=10,
                        fontweight='bold', zorder=6)
        for i, c in enumerate(plan['route']):
            ax.annotate('#%d' % (i + 1), (c['x'], c['y']), xytext=(-8, -20),
                        textcoords='offset points', color='yellow', fontsize=11,
                        fontweight='bold', zorder=6)
        for a in layout.get('animals', []):
            ax.scatter(a['x'], a['y'], marker='P' if a['species'] == 'bear' else 'X',
                       s=180, color='k' if a['species'] == 'bear' else (0.8, 0.6, 0.3),
                       ec='w', zorder=6)
            ax.annotate(a['species'], (a['x'], a['y']), xytext=(8, -12),
                        textcoords='offset points', color='w', fontsize=8)
        for f in sim['found']:
            ax.add_patch(patches.Circle(f['at'], 0.35, fill=False, ec='red', lw=2.5, zorder=7))
        ax.scatter(base['x'], base['y'], marker='s', s=160, color=(0.2, 0.4, 0.9), ec='w',
                   zorder=6)
        ax.set_xlim(b['x_min'], b['x_max'])
        ax.set_ylim(b['y_min'], b['y_max'])
        ax.set_aspect('equal')
        ax.set_title('%s: %s  |  bears found %d  |  %.1f m' % (
            strategy, ' → '.join(c['id'] for c in plan['route']), len(sim['found']),
            sim['distance_m']), fontsize=11)
        ax.set_xticks([])
        ax.set_yticks([])
    fig.suptitle('KumaWatch Patrol (offline dry run): AI route vs fixed route', fontsize=13)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path, dpi=110)


if __name__ == '__main__':
    main()
