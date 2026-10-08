import itertools
import os
import sys

import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)

from kumawatch_mission.route_planning import (  # noqa: E402
    distance, format_candidate_table, nearest_within, order_route, plan_patrol,
    route_length, select_points)

START = (0.0, -8.5)
# 論文用シナリオと同じ値(src/kumawatch_ai/data を参照)
CANDS = [
    dict(id='P1', category='residential', x=-5.5, y=-5.5, risk=0.187, importance_weight=1.1),
    dict(id='P2', category='forest_edge', x=-4.0, y=3.5, risk=0.847, importance_weight=1.0),
    dict(id='P3', category='school_route', x=3.0, y=-1.0, risk=0.737, importance_weight=1.2),
    dict(id='P4', category='garbage', x=5.5, y=2.0, risk=0.905, importance_weight=1.0),
    dict(id='P5', category='river', x=7.0, y=-3.0, risk=0.427, importance_weight=1.0),
]


def ids(route):
    return [c['id'] for c in route]


def test_topk_selects_highest_risk():
    assert ids(select_points(CANDS, 3, 'topk', START)) == ['P4', 'P2', 'P3']


def test_utility_selects_expected_points():
    sel = select_points(CANDS, 3, 'utility', START, lam=0.02)
    assert set(ids(sel)) == {'P2', 'P3', 'P4'}
    # 通学路は重み 1.2 のため、効用ではリスク上位の P4 より上位になる
    assert ids(sel)[0] == 'P3'


def test_large_lambda_prefers_nearby_points():
    sel = select_points(CANDS, 3, 'utility', START, lam=0.2)
    assert 'P1' in ids(sel)  # 近い住宅地が選ばれる
    assert 'P2' not in ids(sel)


def test_fixed_route_keeps_order_and_validates_ids():
    assert ids(select_points(CANDS, 3, 'fixed', START, fixed_ids=['P1', 'P2', 'P3'])) == [
        'P1', 'P2', 'P3']
    with pytest.raises(ValueError):
        select_points(CANDS, 3, 'fixed', START, fixed_ids=['P9'])


def test_unknown_strategy_rejected():
    with pytest.raises(ValueError):
        select_points(CANDS, 3, 'random', START)


def test_budget_edge_cases():
    assert select_points(CANDS, 0, 'utility', START) == []
    assert len(select_points(CANDS, 10, 'utility', START)) == len(CANDS)


def test_order_route_is_optimal():
    sel = select_points(CANDS, 4, 'topk', START)
    best = min(route_length(START, p) for p in itertools.permutations(sel))
    assert route_length(START, order_route(sel, START)) == pytest.approx(best)


def test_order_route_with_return():
    sel = select_points(CANDS, 3, 'utility', START)
    route = order_route(sel, START, end=START)
    best = min(route_length(START, p, START) for p in itertools.permutations(sel))
    assert route_length(START, route, START) == pytest.approx(best)


def test_greedy_fallback_for_many_points():
    many = [dict(id='Q%02d' % i, x=float(i), y=0.0, risk=0.5, importance_weight=1.0)
            for i in range(12)]
    route = order_route(many, (-1.0, 0.0))
    assert ids(route) == ['Q%02d' % i for i in range(12)]


def test_plan_patrol_scenario():
    plan = plan_patrol(CANDS, 3, 'utility', START, 0.02)
    assert set(ids(plan['route'])) == {'P2', 'P3', 'P4'}
    assert plan['planned_length_m'] == pytest.approx(route_length(START, plan['route']))
    fixed = plan_patrol(CANDS, 3, 'fixed', START, fixed_ids=['P1', 'P2', 'P3'])
    assert ids(fixed['route']) == ['P1', 'P2', 'P3']


def test_nearest_within():
    bears = [dict(id='b1', x=0.0, y=2.0), dict(id='b2', x=0.0, y=1.0)]
    obj, d = nearest_within((0.0, 0.0), bears, 2.5)
    assert obj['id'] == 'b2' and d == pytest.approx(1.0)
    assert nearest_within((10.0, 0.0), bears, 2.5) is None


def test_table_marks_selected():
    table = format_candidate_table(CANDS, START, 0.02, {'P4'})
    line = [ln for ln in table.splitlines() if ln.strip().startswith('P4')][0]
    assert line.rstrip().endswith('*')
    assert distance((0, 0), (3, 4)) == 5
