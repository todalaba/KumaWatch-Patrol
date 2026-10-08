#!/usr/bin/env python3
"""模擬目撃データでリスクモデルを学習し、巡回候補地点のスコアを事前計算する.

出力(既定: src/kumawatch_ai/data/):
  risk_model.json            学習済みロジスティック回帰(係数・標準化パラメータ・検証指標)
  simulated_risk_scores.csv  各候補地点のリスク推定値(risk_predictor ノードが読み込む)

模擬データは「地点×日」の観測で、真の生成モデル(TRUE_*)から熊出没の有無を
サンプリングしている。実際の熊出没データは使っていない(公開用の架空データ)。

使い方:
  python3 scripts/train_risk_model.py [--seed 2026] [--n 8000]
"""

import argparse
import csv
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'kumawatch_ai'))

from kumawatch_ai.risk_model import (  # noqa: E402
    MODEL_FEATURES, RiskModel, derive_features, fit_logistic_regression,
    read_point_features, roc_auc)

DATA_DIR = os.path.join(ROOT, 'src', 'kumawatch_ai', 'data')

# 模擬データの真の生成モデル(logit 空間; MODEL_FEATURES の順、標準化前)
TRUE_INTERCEPT = -3.2
TRUE_COEF = {
    'past_sightings_30d': 0.25,
    'sighted_yesterday': 1.3,
    'forest_proximity': 2.0,
    'garbage_site': 1.8,
    'twilight': 0.6,
    'temperature_c': 0.04,
    'precipitation_mm': -0.1,
}
TEMP_REF = 15.0


def simulate(n, rng):
    past = rng.poisson(1.5, n)
    yday = (rng.random(n) < np.clip(0.08 + 0.06 * past, 0, 0.6)).astype(int)
    dist = rng.uniform(0, 600, n)
    garbage = (rng.random(n) < 0.15).astype(int)
    hour = rng.integers(0, 24, n)
    temp = rng.normal(15, 6, n)
    precip = np.where(rng.random(n) < 0.4, rng.exponential(4.0, n), 0.0)
    rows = [dict(past_sightings_30d=past[i], sighted_yesterday=yday[i],
                 dist_to_forest_m=dist[i], garbage_site=garbage[i], hour=hour[i],
                 temperature_c=temp[i], precipitation_mm=precip[i]) for i in range(n)]
    X = np.array([derive_features(r) for r in rows])
    beta = np.array([TRUE_COEF[f] for f in MODEL_FEATURES])
    offset = np.zeros(len(MODEL_FEATURES))
    offset[MODEL_FEATURES.index('temperature_c')] = TEMP_REF
    logit = TRUE_INTERCEPT + (X - offset) @ beta
    y = (rng.random(n) < 1 / (1 + np.exp(-logit))).astype(int)
    return X, y


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seed', type=int, default=2026)
    ap.add_argument('--n', type=int, default=8000, help='模擬観測数(地点×日)')
    ap.add_argument('--out-dir', default=DATA_DIR)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    X, y = simulate(args.n, rng)
    n_train = int(0.75 * args.n)
    intercept, coef, mean, std = fit_logistic_regression(X[:n_train], y[:n_train])

    Zt = (X[n_train:] - np.array(mean)) / np.array(std)
    p_test = 1 / (1 + np.exp(-(intercept + Zt @ np.array(coef))))
    metrics = {
        'n_train': n_train,
        'n_test': int(args.n - n_train),
        'positive_rate_train': round(float(y[:n_train].mean()), 4),
        'test_auc': round(roc_auc(y[n_train:].tolist(), p_test.tolist()), 4),
        'test_brier': round(float(np.mean((p_test - y[n_train:]) ** 2)), 4),
        'seed': args.seed,
    }
    model = RiskModel(intercept, coef, mean, std, name='logistic_regression_v1', metrics=metrics)
    model_path = os.path.join(args.out_dir, 'risk_model.json')
    model.save(model_path)

    feats = read_point_features(os.path.join(args.out_dir, 'point_features.csv'))
    score_path = os.path.join(args.out_dir, 'simulated_risk_scores.csv')
    with open(score_path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['point_id', 'risk', 'model'])
        for pid, raw in feats.items():
            w.writerow([pid, '%.3f' % model.predict_proba(raw), model.name])

    print('model  : %s' % model_path)
    print('metrics: %s' % metrics)
    print('coef (standardized):')
    for name, c in zip(MODEL_FEATURES, coef):
        print('  %-20s %+.3f' % (name, c))
    print('scores : %s' % score_path)
    for pid, raw in feats.items():
        p = model.predict_proba(raw)
        print('  %s  %.3f  %s' % (pid, p, '#' * int(math.floor(p * 40))))


if __name__ == '__main__':
    main()
