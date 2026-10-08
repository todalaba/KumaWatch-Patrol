"""動画撮影用の機能(退避地点・字幕の文言・追従カメラの計算・HUD)のテスト(ROS 不要)."""

import math
import os
import sys

import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)

from kumawatch_mission.chase_math import chase_pose, quaternion, smooth_angle  # noqa: E402
from kumawatch_mission.mission_text import (bear_panel_lines, format_event,  # noqa: E402
                                            status_lines)
from kumawatch_mission.route_planning import retreat_point  # noqa: E402


def test_retreat_point_moves_away_from_bear_and_faces_it():
    x, y, yaw = retreat_point((0.0, 0.0), (0.0, 2.0), 3.0)
    assert (round(x, 6), round(y, 6)) == (0.0, -3.0)
    assert math.isclose(yaw, math.pi / 2)            # 熊(+y)の方を向く


def test_retreat_point_is_clamped_to_world():
    x, y, _ = retreat_point((8.5, 0.0), (7.0, 0.0), 3.0, bounds=(-9, 9, -9, 9))
    assert x == 9.0 and y == 0.0


def test_retreat_point_when_bear_on_robot():
    x, y, _ = retreat_point((1.0, 1.0), (1.0, 1.0), 2.0)
    assert (x, y) == (1.0, -1.0)


def test_format_event_lines():
    ev = {'event': 'navigate', 'index': 1, 'total': 3, 'point_id': 'P2',
          'detail': {'name_en': 'Forest edge', 'risk': 0.979}}
    assert format_event(ev) == 'Goal 1/3: P2 Forest edge  (AI risk 0.98)  navigating'
    bear = {'event': 'bear_detected', 'detail': {'bear_id': 'bear_1', 'point_id': 'P2',
                                                 'confidence': 0.91, 'distance_m': 2.46,
                                                 'action': 'Stop and report'}}
    assert format_event(bear).startswith('BEAR DETECTED  bear_1 at P2  conf 0.91  2.5 m')
    done = {'event': 'mission_completed', 'detail': {'num_points': 3, 'num_checked': 3,
                                                     'bears_found': 2, 'distance_m': 33.4,
                                                     'duration_s': 158.7}}
    assert '3/3 points checked, 2 bear(s) found' in format_event(done)
    assert format_event({'event': 'unknown'}) is None


def test_status_and_bear_panel():
    st = {'state': 'PATROL', 'index': 2, 'total': 3, 'point_id': 'P4',
          'name_en': 'Garbage station', 'risk': 1.0, 'bears_found': 1, 'distance_m': 20.0}
    l1, l2 = status_lines(st)
    assert l1 == 'Mission 2/3: P4 Garbage station'
    assert l2.startswith('AI risk 1.00  |  Navigating')
    assert status_lines({'state': 'FINISHED', 'arrived': 1, 'total': 3})[0] == 'Mission completed'
    lines = bear_panel_lines({'confidence': 0.9, 'distance_m': 3.0, 'point_id': 'P2',
                              'action': 'Stop and report'})
    assert lines[0] == 'BEAR DETECTED' and lines[2] == 'Location: Patrol point P2'


def test_chase_camera_geometry():
    x, y, z, pitch, yaw = chase_pose(1.0, 2.0, math.pi / 2, 1.5, 1.2, 0.4)
    assert math.isclose(x, 1.0, abs_tol=1e-9) and math.isclose(y, 0.5)   # ロボットの後ろ
    assert z == 1.2 and yaw == math.pi / 2
    qx, qy, qz, qw = quaternion(0.0, math.pi)
    assert math.isclose(qz, 1.0, abs_tol=1e-9) and math.isclose(qw, 0.0, abs_tol=1e-9)
    assert math.isclose(sum(v * v for v in quaternion(0.4, 1.0)), 1.0)
    # 角度の平滑化は -π/π をまたいでも近い向きへ回る
    assert abs(smooth_angle(3.1, -3.1, 0.5) - math.pi) < 0.05


def test_hud_draws_without_error():
    np = pytest.importorskip('numpy')
    pytest.importorskip('cv2')
    from kumawatch_mission.hud import draw_hud
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    out = draw_hud(img.copy(), {'state': 'EMERGENCY_STOP', 'index': 1, 'total': 3,
                                'point_id': 'P2', 'risk': 0.98},
                   {'confidence': 0.9, 'distance_m': 2.5, 'point_id': 'P2'})
    assert out.shape == img.shape and out.any()
