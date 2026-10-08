"""熊出没リスク推定モデル(ロジスティック回帰).

ROS に依存しない純粋な Python モジュール。推論は標準ライブラリのみで動作し、
学習(:func:`fit_logistic_regression`)のみ numpy を使う。

入力となる地点特徴量(1地点・1日あたり):

========================  ==========================================
列名                       意味
========================  ==========================================
past_sightings_30d        過去30日の目撃回数
sighted_yesterday         前日の目撃有無 (0/1)
dist_to_forest_m          林縁までの距離 [m]
garbage_site              ごみ集積所の有無 (0/1)
hour                      巡回予定時刻 [時, 0-23]
temperature_c             気温 [℃]
precipitation_mm          降水量 [mm]
========================  ==========================================

モデルはこれらを次の説明変数に変換してから線形結合する:
``forest_proximity = exp(-dist/150)``(林に近いほど 1)、
``twilight``(薄明薄暮 4-7時/16-19時なら 1)など。
"""

import csv
import json
import math

RAW_FEATURES = [
    'past_sightings_30d',
    'sighted_yesterday',
    'dist_to_forest_m',
    'garbage_site',
    'hour',
    'temperature_c',
    'precipitation_mm',
]

MODEL_FEATURES = [
    'past_sightings_30d',
    'sighted_yesterday',
    'forest_proximity',
    'garbage_site',
    'twilight',
    'temperature_c',
    'precipitation_mm',
]

FOREST_DECAY_M = 150.0


def is_twilight(hour):
    """熊の活動が活発な薄明薄暮(4-7時, 16-19時)なら 1."""
    h = int(hour) % 24
    return 1.0 if (4 <= h <= 7 or 16 <= h <= 19) else 0.0


def derive_features(raw):
    """生の地点特徴量 dict をモデル入力ベクトル(MODEL_FEATURES 順)に変換する."""
    return [
        float(raw['past_sightings_30d']),
        float(raw['sighted_yesterday']),
        math.exp(-float(raw['dist_to_forest_m']) / FOREST_DECAY_M),
        float(raw['garbage_site']),
        is_twilight(raw['hour']),
        float(raw['temperature_c']),
        float(raw['precipitation_mm']),
    ]


def sigmoid(z):
    if z >= 0:
        return 1.0 / (1.0 + math.exp(-z))
    ez = math.exp(z)
    return ez / (1.0 + ez)


class RiskModel:
    """学習済みロジスティック回帰モデル(標準化込み)."""

    def __init__(self, intercept, coef, mean, std, name='logistic_regression', metrics=None):
        if not (len(coef) == len(mean) == len(std) == len(MODEL_FEATURES)):
            raise ValueError('coef/mean/std must have %d elements' % len(MODEL_FEATURES))
        self.intercept = float(intercept)
        self.coef = [float(c) for c in coef]
        self.mean = [float(m) for m in mean]
        self.std = [float(s) if float(s) > 0 else 1.0 for s in std]
        self.name = name
        self.metrics = metrics or {}

    def predict_proba(self, raw):
        x = derive_features(raw)
        z = self.intercept
        for xi, c, m, s in zip(x, self.coef, self.mean, self.std):
            z += c * (xi - m) / s
        return sigmoid(z)

    def to_dict(self):
        return {
            'name': self.name,
            'type': 'logistic_regression',
            'raw_features': RAW_FEATURES,
            'model_features': MODEL_FEATURES,
            'forest_decay_m': FOREST_DECAY_M,
            'intercept': self.intercept,
            'coef': self.coef,
            'mean': self.mean,
            'std': self.std,
            'metrics': self.metrics,
        }

    def save(self, path):
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(self.to_dict(), f, indent=2, ensure_ascii=False)
            f.write('\n')

    @classmethod
    def load(cls, path):
        with open(path, encoding='utf-8') as f:
            d = json.load(f)
        if d.get('model_features') != MODEL_FEATURES:
            raise ValueError('model file %s has incompatible features' % path)
        return cls(d['intercept'], d['coef'], d['mean'], d['std'],
                   name=d.get('name', 'logistic_regression'), metrics=d.get('metrics'))


def fit_logistic_regression(X, y, l2=1e-3, max_iter=100, tol=1e-8):
    """IRLS(ニュートン法)による L2 正則化付きロジスティック回帰.

    X: (n, d) の説明変数(標準化前), y: (n,) の 0/1.
    戻り値: (intercept, coef, mean, std)
    """
    import numpy as np

    X = np.asarray(X, dtype=float)
    y = np.asarray(y, dtype=float)
    mean = X.mean(axis=0)
    std = X.std(axis=0)
    std[std == 0] = 1.0
    Z = np.hstack([np.ones((X.shape[0], 1)), (X - mean) / std])
    w = np.zeros(Z.shape[1])
    reg = l2 * np.eye(Z.shape[1])
    reg[0, 0] = 0.0  # 切片は正則化しない
    for _ in range(max_iter):
        p = 1.0 / (1.0 + np.exp(-Z @ w))
        grad = Z.T @ (p - y) + reg @ w
        H = (Z * (p * (1 - p))[:, None]).T @ Z + reg
        step = np.linalg.solve(H, grad)
        w -= step
        if np.max(np.abs(step)) < tol:
            break
    return float(w[0]), w[1:].tolist(), mean.tolist(), std.tolist()


def roc_auc(y_true, scores):
    """順位ベース(Mann-Whitney U)の ROC-AUC. 同順位は平均順位で扱う."""
    pairs = sorted(zip(scores, y_true))
    n = len(pairs)
    ranks = [0.0] * n
    i = 0
    while i < n:
        j = i
        while j + 1 < n and pairs[j + 1][0] == pairs[i][0]:
            j += 1
        for k in range(i, j + 1):
            ranks[k] = (i + j) / 2.0 + 1.0
        i = j + 1
    n_pos = sum(1 for _, t in pairs if t)
    n_neg = n - n_pos
    if n_pos == 0 or n_neg == 0:
        return float('nan')
    rank_sum = sum(r for r, (_, t) in zip(ranks, pairs) if t)
    return (rank_sum - n_pos * (n_pos + 1) / 2.0) / (n_pos * n_neg)


def read_point_features(path):
    """地点特徴量 CSV を {point_id: {列名: 値}} で読む."""
    out = {}
    with open(path, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            pid = row.pop('point_id')
            out[pid] = {k: float(v) for k, v in row.items() if k in RAW_FEATURES}
    return out


def _csv_rows(f):
    """'#' で始まるコメント行を除いた CSV の行."""
    return csv.DictReader(line for line in f if not line.lstrip().startswith('#'))


def read_risk_scores(path):
    """事前計算済みリスクスコア CSV(point_id, risk, ...)を {point_id: risk} で読む."""
    out = {}
    with open(path, newline='', encoding='utf-8') as f:
        for row in _csv_rows(f):
            risk = float(row['risk'])
            if not 0.0 <= risk <= 1.0:
                raise ValueError('risk for %s out of [0,1]: %s' % (row['point_id'], risk))
            out[row['point_id']] = risk
    return out


def read_score_model_name(path):
    """スコア CSV の model 列(あれば)を返す."""
    with open(path, newline='', encoding='utf-8') as f:
        for row in _csv_rows(f):
            return row.get('model') or ''
    return ''


def load_patrol_points(path):
    """patrol_points.yaml を読み、地点 dict のリストを返す."""
    import yaml

    with open(path, encoding='utf-8') as f:
        d = yaml.safe_load(f)
    points = d['points']
    seen = set()
    for p in points:
        for key in ('id', 'name', 'category', 'x', 'y'):
            if key not in p:
                raise ValueError('patrol point missing "%s": %r' % (key, p))
        if p['id'] in seen:
            raise ValueError('duplicate patrol point id %s' % p['id'])
        seen.add(p['id'])
        p.setdefault('yaw', 0.0)
        p.setdefault('importance_weight', 1.0)
    return points
