import math
import os
import sys
from xml.dom import minidom

import pytest
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.dirname(PKG)
sys.path.insert(0, PKG)

from kumawatch_gazebo import world_builder as wb  # noqa: E402

LAYOUT = os.path.join(PKG, 'config', 'world_layout.yaml')
WORLD = os.path.join(PKG, 'worlds', 'kumawatch_patrol.world')
POINTS = os.path.join(SRC, 'kumawatch_ai', 'data', 'patrol_points.yaml')
MAP_DIR = os.path.join(SRC, 'kumawatch_navigation', 'maps')

needs_repo = pytest.mark.skipif(
    not (os.path.exists(POINTS) and os.path.isdir(MAP_DIR)),
    reason='ソースツリー(src/ 以下の他パッケージ)が必要')


@pytest.fixture(scope='module')
def layout():
    return wb.load_yaml(LAYOUT)


@pytest.fixture(scope='module')
def points():
    with open(POINTS, encoding='utf-8') as f:
        d = yaml.safe_load(f)
    return d['points'] + [dict(d['base'], id='base')]


@pytest.fixture(scope='module')
def grid(layout):
    return wb.rasterize(layout)


def test_world_is_valid_xml_with_expected_models():
    doc = minidom.parse(WORLD)
    names = {m.getAttribute('name') for m in doc.getElementsByTagName('model')}
    assert {'ground_plane', 'bear_1', 'bear_2', 'deer_1', 'garbage_station'} <= names


@needs_repo
def test_generated_files_are_up_to_date(layout, points):
    """world_layout.yaml を変えたら scripts/generate_world.py を実行し忘れていないか."""
    pts = [p for p in points if p['id'] != 'base']
    with open(WORLD, encoding='utf-8') as f:
        assert f.read() == wb.build_world_sdf(layout, pts)
    with open(os.path.join(MAP_DIR, 'kumawatch_map.pgm'), 'rb') as f:
        pgm = f.read()
    tmp = os.path.join(os.path.dirname(WORLD), '.tmp_test_map.pgm')
    try:
        wb.write_pgm(wb.rasterize(layout), tmp)
        with open(tmp, 'rb') as f:
            assert f.read() == pgm
    finally:
        os.remove(tmp)


def test_map_dimensions(layout, grid):
    b = layout['bounds']
    res = layout['map_resolution']
    assert len(grid) == round((b['y_max'] - b['y_min']) / res)
    assert len(grid[0]) == round((b['x_max'] - b['x_min']) / res)
    # 外周は壁
    assert all(v == wb.OCCUPIED for v in grid[0])
    assert all(row[0] == wb.OCCUPIED for row in grid)


@needs_repo
def test_patrol_points_have_clearance(layout, grid, points):
    """巡回地点と詰所は障害物から 0.5 m 以上離れている(Nav2 のゴールに到達できる)."""
    res = layout['map_resolution']
    b = layout['bounds']
    n = int(0.5 / res)
    for p in points:
        r0 = int((p['y'] - b['y_min']) / res)
        c0 = int((p['x'] - b['x_min']) / res)
        for dr in range(-n, n + 1):
            for dc in range(-n, n + 1):
                if dr * dr + dc * dc <= n * n:
                    assert grid[r0 + dr][c0 + dc] == wb.FREE, p['id']


@needs_repo
def test_bear_placement_matches_scenario(layout, points):
    """熊は P2(林縁)と P4(ごみ集積所)の近くだけにいる(検知半径 2.5 m)."""
    bears = wb.bears(layout)
    near = {p['id'] for p in points
            if any(math.hypot(p['x'] - a['x'], p['y'] - a['y']) <= 2.5 for a in bears)}
    assert near == {'P2', 'P4'}


def test_animals_not_in_static_map(layout, grid):
    """動物は地図に載せない(ロボットにとって事前に未知の物体)."""
    for a in layout['animals']:
        assert wb.is_free(grid, layout, a['x'], a['y'])


def test_pgm_orientation(tmp_path):
    lay = {'bounds': {'x_min': 0, 'x_max': 1, 'y_min': 0, 'y_max': 1}, 'map_resolution': 0.5}
    grid = [[wb.OCCUPIED, wb.FREE], [wb.FREE, wb.FREE]]  # row0 = y_min 側
    path = tmp_path / 'm.pgm'
    wb.write_pgm(grid, str(path))
    data = path.read_bytes()
    pixels = data[-4:]
    # 画像の上端は y_max 側なので、占有セルは最終行の先頭に来る
    assert list(pixels) == [254, 254, 0, 254]
    wb.write_map_yaml(str(tmp_path / 'm.yaml'), 'm.pgm', lay)
    meta = yaml.safe_load((tmp_path / 'm.yaml').read_text())
    assert meta['origin'] == [0.0, 0.0, 0.0] and meta['resolution'] == 0.5
