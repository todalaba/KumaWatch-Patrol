"""巡回地点の選択と巡回順の決定(ROS 非依存).

地点選択の戦略:
  topk     リスク p_i の上位 K 地点
  utility  効用 U_i = w_i * p_i - lambda * d_i の上位 K 地点(既定)
           p_i: リスク, w_i: 社会的重要度, d_i: 出発地点からの距離 [m]
  fixed    リスクを使わない固定ルート(比較用ベースライン)

巡回順は、選んだ K 地点を出発地点から回る開路の総移動距離が最小になる順
(K <= 8 は全順列の厳密解、それより多い場合は最近傍法)。fixed は指定順のまま。
"""

import itertools
import math

STRATEGIES = ('topk', 'utility', 'fixed')
EXACT_ORDER_MAX = 8


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def utility(candidate, start, lam):
    return candidate['importance_weight'] * candidate['risk'] - lam * distance(
        start, (candidate['x'], candidate['y']))


def route_length(start, route, end=None):
    """start → route の各地点 → (end) の直線距離の合計."""
    total = 0.0
    prev = start
    for c in route:
        total += distance(prev, (c['x'], c['y']))
        prev = (c['x'], c['y'])
    if end is not None:
        total += distance(prev, end)
    return total


def select_points(candidates, k, strategy='utility', start=(0.0, 0.0), lam=0.02,
                  fixed_ids=()):
    """巡回する地点を選ぶ. 各地点 dict に 'utility' を付けたコピーのリストを返す.

    candidates: [{'id', 'x', 'y', 'risk', 'importance_weight', ...}, ...]
    """
    if strategy not in STRATEGIES:
        raise ValueError('unknown strategy %r (choose from %s)' % (strategy, STRATEGIES))
    if k <= 0:
        return []
    scored = []
    for c in candidates:
        c = dict(c)
        c.setdefault('importance_weight', 1.0)
        c['distance_from_start'] = distance(start, (c['x'], c['y']))
        c['utility'] = utility(c, start, lam)
        scored.append(c)

    if strategy == 'fixed':
        by_id = {c['id']: c for c in scored}
        missing = [i for i in fixed_ids if i not in by_id]
        if missing:
            raise ValueError('fixed route contains unknown point ids: %s' % missing)
        return [by_id[i] for i in list(fixed_ids)[:k]]

    key = (lambda c: c['risk']) if strategy == 'topk' else (lambda c: c['utility'])
    # 同点は地点 ID 順で決定的に
    ranked = sorted(scored, key=lambda c: (-key(c), c['id']))
    return ranked[:k]


def order_route(selected, start, end=None):
    """総移動距離が最小になる巡回順を返す."""
    if len(selected) <= 1:
        return list(selected)
    if len(selected) <= EXACT_ORDER_MAX:
        best = min(itertools.permutations(selected),
                   key=lambda perm: (route_length(start, perm, end), [c['id'] for c in perm]))
        return list(best)
    remaining = list(selected)
    route = []
    pos = start
    while remaining:
        nxt = min(remaining, key=lambda c: (distance(pos, (c['x'], c['y'])), c['id']))
        route.append(nxt)
        remaining.remove(nxt)
        pos = (nxt['x'], nxt['y'])
    return route


def plan_patrol(candidates, k, strategy='utility', start=(0.0, 0.0), lam=0.02,
                fixed_ids=(), return_to_start=False):
    """地点選択と巡回順決定をまとめて行う.

    戻り値: {'strategy', 'k', 'lambda', 'route': [地点 dict...], 'planned_length_m'}
    """
    selected = select_points(candidates, k, strategy, start, lam, fixed_ids)
    end = start if return_to_start else None
    route = selected if strategy == 'fixed' else order_route(selected, start, end)
    return {
        'strategy': strategy,
        'k': k,
        'lambda': lam,
        'route': route,
        'planned_length_m': route_length(start, route, end),
    }


def format_candidate_table(candidates, start, lam, selected_ids=()):
    """候補地点の一覧(リスク・重み・距離・効用・選択)を表形式の文字列で返す."""
    lines = ['  ID   category        risk   w     d[m]   U=w*p-λd  selected',
             '  ' + '-' * 62]
    for c in sorted(candidates, key=lambda c: c['id']):
        d = distance(start, (c['x'], c['y']))
        u = utility(c, start, lam)
        lines.append('  %-4s %-14s  %.3f  %.2f  %5.1f   %+.3f     %s' % (
            c['id'], c.get('category', ''), c['risk'], c['importance_weight'], d, u,
            '*' if c['id'] in selected_ids else ''))
    return '\n'.join(lines)


def nearest_within(position, objects, radius):
    """position から radius 以内で最も近い物体を返す. なければ None.

    objects: [{'id', 'x', 'y', ...}, ...]
    戻り値: (object, distance) または None
    """
    best = None
    for o in objects:
        d = distance(position, (o['x'], o['y']))
        if d <= radius and (best is None or d < best[1]):
            best = (o, d)
    return best


def retreat_point(robot, bear, distance_m=3.0, bounds=(-9.0, 9.0, -9.0, 9.0)):
    """熊から離れる向きに distance_m 下がった退避地点 (x, y, yaw) を返す.

    熊 → ロボットの向きにさらに下がる。ワールドの範囲 bounds=(xmin, xmax, ymin, ymax) に収める。
    yaw は退避地点で熊の方を向く向き(退避後も熊を見張る)。
    """
    dx, dy = robot[0] - bear[0], robot[1] - bear[1]
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        dx, dy, norm = 0.0, -1.0, 1.0   # 熊と同じ位置なら、とりあえず -y 方向(詰所側)へ
    x = robot[0] + dx / norm * distance_m
    y = robot[1] + dy / norm * distance_m
    xmin, xmax, ymin, ymax = bounds
    x = min(max(x, xmin), xmax)
    y = min(max(y, ymin), ymax)
    yaw = math.atan2(bear[1] - y, bear[0] - x)
    return x, y, yaw
