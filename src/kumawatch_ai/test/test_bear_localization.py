import math
import os
import sys

import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)

from kumawatch_ai.bear_localization import (  # noqa: E402
    bearing_from_bbox, BearTracker, estimate_range, focal_length_px, range_from_bbox_height,
    range_from_scan, to_map)


def test_bearing_sign_and_center():
    f = focal_length_px(640, 1.06)
    assert bearing_from_bbox(320, 640, f) == pytest.approx(0.0)
    assert bearing_from_bbox(100, 640, f) > 0      # 画像の左 = ロボットの左 = 正
    assert bearing_from_bbox(640, 640, f) == pytest.approx(-0.53, abs=1e-6)


def test_range_from_bbox_height():
    f = focal_length_px(640, 1.06)
    # 体高 1.09 m の熊が 3 m 先にいるときの枠の高さ
    h = f * 1.09 / 3.0
    assert range_from_bbox_height(h, f) == pytest.approx(3.0)
    assert math.isinf(range_from_bbox_height(0, f))


def test_range_from_scan_window_and_wraparound():
    ranges = [float('inf')] * 360
    ranges[0] = 2.5          # 正面
    ranges[358] = 2.4        # 正面から右に 2 度(角度 -2 度は index 358)
    inc = 2 * math.pi / 360
    assert range_from_scan(ranges, 0.0, inc, 0.0) == pytest.approx(2.4)
    assert range_from_scan(ranges, 0.0, inc, math.radians(90)) is None


def test_estimate_range_prefers_scan_unless_inconsistent():
    assert estimate_range(3.0, 3.3) == 3.0
    assert estimate_range(0.8, 3.3) == 3.3      # 手前の木を拾った LiDAR は使わない
    assert estimate_range(None, 3.3) == 3.3


def test_to_map():
    x, y = to_map(1.0, 2.0, math.pi / 2, 0.0, 3.0)
    assert (x, y) == (pytest.approx(1.0), pytest.approx(5.0))


def test_tracker_needs_consecutive_hits_and_keeps_id():
    tr = BearTracker(min_hits=3)
    assert tr.update([(5.0, 5.0, 0.9)]) == []
    assert tr.update([]) == []                  # 途切れたらやり直し
    assert tr.update([(5.0, 5.0, 0.9)]) == []
    assert tr.update([(5.1, 5.0, 0.9)]) == []
    confirmed = tr.update([(5.0, 5.1, 0.9)])
    assert [t['id'] for t in confirmed] == ['bear_cam_1']
    # 見失ってから再び見えても同じ熊として扱う
    for _ in range(10):
        tr.update([])
    again = tr.update([(5.2, 5.0, 0.8)])
    assert [t['id'] for t in again] == ['bear_cam_1']
    # 離れた場所の熊は別の ID
    for _ in range(3):
        other = tr.update([(-4.0, 5.0, 0.8)])
    assert [t['id'] for t in other] == ['bear_cam_2']
