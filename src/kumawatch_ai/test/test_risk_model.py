import math
import os
import sys

import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)

from kumawatch_ai.risk_model import (  # noqa: E402
    derive_features, is_twilight, load_patrol_points, MODEL_FEATURES, read_point_features,
    read_risk_scores, RiskModel, roc_auc, sigmoid)

DATA = os.path.join(PKG, 'data')


def test_sigmoid_is_stable_and_symmetric():
    assert sigmoid(0) == pytest.approx(0.5)
    assert sigmoid(800) == pytest.approx(1.0)
    assert sigmoid(-800) == pytest.approx(0.0)
    assert sigmoid(2.0) + sigmoid(-2.0) == pytest.approx(1.0)


def test_twilight_hours():
    assert [h for h in range(24) if is_twilight(h)] == [4, 5, 6, 7, 16, 17, 18, 19]


def test_derive_features_order_and_forest_decay():
    raw = dict(past_sightings_30d=3, sighted_yesterday=1, dist_to_forest_m=0, garbage_site=1,
               hour=17, temperature_c=16, precipitation_mm=0.5)
    x = derive_features(raw)
    assert len(x) == len(MODEL_FEATURES)
    assert x[MODEL_FEATURES.index('forest_proximity')] == pytest.approx(1.0)
    far = derive_features(dict(raw, dist_to_forest_m=1500))
    assert far[MODEL_FEATURES.index('forest_proximity')] < 1e-4


def test_roc_auc():
    assert roc_auc([0, 0, 1, 1], [0.1, 0.2, 0.8, 0.9]) == pytest.approx(1.0)
    assert roc_auc([0, 1, 0, 1], [0.5, 0.5, 0.5, 0.5]) == pytest.approx(0.5)
    assert math.isnan(roc_auc([1, 1], [0.1, 0.2]))


def test_saved_model_reproduces_precomputed_scores():
    """risk_predictor の mode:=csv と mode:=model が同じ値を出すこと."""
    model = RiskModel.load(os.path.join(DATA, 'risk_model.json'))
    feats = read_point_features(os.path.join(DATA, 'point_features.csv'))
    scores = read_risk_scores(os.path.join(DATA, 'simulated_risk_scores.csv'))
    assert set(feats) == set(scores)
    for pid, raw in feats.items():
        assert model.predict_proba(raw) == pytest.approx(scores[pid], abs=6e-4)


def test_scenario_ranking():
    """シナリオ: ごみ集積所 > 林縁 > 通学路 > 河川沿い > 住宅地."""
    scores = read_risk_scores(os.path.join(DATA, 'simulated_risk_scores.csv'))
    assert sorted(scores, key=scores.get, reverse=True) == ['P4', 'P2', 'P3', 'P5', 'P1']


def test_model_quality_recorded():
    model = RiskModel.load(os.path.join(DATA, 'risk_model.json'))
    assert model.metrics['test_auc'] > 0.7


def test_patrol_points_match_features():
    points = load_patrol_points(os.path.join(DATA, 'patrol_points.yaml'))
    feats = read_point_features(os.path.join(DATA, 'point_features.csv'))
    assert [p['id'] for p in points] == sorted(feats)
    assert all(p['importance_weight'] > 0 for p in points)


def test_risk_scores_reject_out_of_range(tmp_path):
    bad = tmp_path / 'bad.csv'
    bad.write_text('point_id,risk\nP1,1.5\n')
    with pytest.raises(ValueError):
        read_risk_scores(str(bad))


def _ttm_rows():
    import csv
    with open(os.path.join(DATA, 'kumawatch_ttm_scores.csv'), encoding='utf-8') as f:
        return {r['point_id']: r for r in csv.DictReader(ln for ln in f if not ln.startswith('#'))}


def test_ttm_scores_cover_points_and_are_percentiles():
    rows = _ttm_rows()
    points = load_patrol_points(os.path.join(DATA, 'patrol_points.yaml'))
    assert set(rows) == {p['id'] for p in points}
    risks = read_risk_scores(os.path.join(DATA, 'kumawatch_ttm_scores.csv'))
    for pid, r in rows.items():
        n = int(r['n_cells'])
        assert risks[pid] == pytest.approx(1 - (float(r['ttm_rank']) - 1) / (n - 1), abs=1e-4)
        assert 'TTM' in r['model']


def test_ttm_scenario_bears_are_real_sightings():
    """熊を置いた P2・P4 は、その日に実際に出没があったセル。ほかは出没なし."""
    rows = _ttm_rows()
    assert {pid for pid, r in rows.items() if int(r['observed_sightings']) > 0} == {'P2', 'P4'}
    assert len({r['date'] for r in rows.values()}) == 1
