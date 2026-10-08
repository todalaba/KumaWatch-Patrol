#!/usr/bin/env python3
"""world_layout.yaml から Gazebo ワールドと Nav2 用の地図を生成する.

出力:
  src/kumawatch_gazebo/worlds/kumawatch_patrol.world
  src/kumawatch_navigation/maps/kumawatch_map.pgm
  src/kumawatch_navigation/maps/kumawatch_map.yaml

使い方: python3 scripts/generate_world.py
"""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'kumawatch_gazebo'))
sys.path.insert(0, os.path.join(ROOT, 'src', 'kumawatch_ai'))

from kumawatch_ai.risk_model import load_patrol_points  # noqa: E402
from kumawatch_gazebo import world_builder as wb  # noqa: E402

LAYOUT = os.path.join(ROOT, 'src', 'kumawatch_gazebo', 'config', 'world_layout.yaml')
POINTS = os.path.join(ROOT, 'src', 'kumawatch_ai', 'data', 'patrol_points.yaml')
WORLD = os.path.join(ROOT, 'src', 'kumawatch_gazebo', 'worlds', 'kumawatch_patrol.world')
MAP_DIR = os.path.join(ROOT, 'src', 'kumawatch_navigation', 'maps')


def main():
    layout = wb.load_yaml(LAYOUT)
    points = load_patrol_points(POINTS)
    base = wb.load_yaml(POINTS)['base']

    with open(WORLD, 'w', encoding='utf-8') as f:
        f.write(wb.build_world_sdf(layout, points))
    print('world: %s' % WORLD)

    grid = wb.rasterize(layout)
    wb.write_pgm(grid, os.path.join(MAP_DIR, 'kumawatch_map.pgm'))
    wb.write_map_yaml(os.path.join(MAP_DIR, 'kumawatch_map.yaml'), 'kumawatch_map.pgm', layout)
    print('map  : %s/kumawatch_map.{pgm,yaml} (%dx%d)' % (MAP_DIR, len(grid[0]), len(grid)))

    for p in points + [dict(base, id='base')]:
        if not wb.is_free(grid, layout, p['x'], p['y']):
            sys.exit('ERROR: %s (%.2f, %.2f) is inside an obstacle' % (p['id'], p['x'], p['y']))


if __name__ == '__main__':
    main()
