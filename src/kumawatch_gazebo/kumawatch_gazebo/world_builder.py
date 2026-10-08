"""world_layout.yaml から Gazebo ワールド(SDF)と Nav2 用の占有格子地図を生成する.

ROS 非依存。scripts/generate_world.py から使う。地図とワールドを同じ定義から作るので、
SLAM で地図を作らなくても両者の座標が一致する(事前地図方式)。
"""

import math

import yaml

FREE = 0
OCCUPIED = 100


def load_yaml(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def bears(layout):
    """熊の既知配置 [{'id', 'x', 'y'}, ...]."""
    return [dict(a) for a in layout.get('animals', []) if a.get('species') == 'bear']


# ---------------------------------------------------------------------------
# 形状(地図のラスタライズ用)
# ---------------------------------------------------------------------------

def _rect(cx, cy, sx, sy, yaw=0.0):
    return ('rect', float(cx), float(cy), float(sx), float(sy), float(yaw))


def _circle(cx, cy, r):
    return ('circle', float(cx), float(cy), float(r))


def wall_rects(layout):
    b = layout['bounds']
    t = layout['wall']['thickness']
    w = b['x_max'] - b['x_min']
    h = b['y_max'] - b['y_min']
    cx = (b['x_max'] + b['x_min']) / 2.0
    cy = (b['y_max'] + b['y_min']) / 2.0
    return [
        ('wall_north', _rect(cx, b['y_max'] - t / 2, w, t)),
        ('wall_south', _rect(cx, b['y_min'] + t / 2, w, t)),
        ('wall_east', _rect(b['x_max'] - t / 2, cy, t, h)),
        ('wall_west', _rect(b['x_min'] + t / 2, cy, t, h)),
    ]


def fence_rect(f):
    dx = f['x2'] - f['x1']
    dy = f['y2'] - f['y1']
    return _rect((f['x1'] + f['x2']) / 2, (f['y1'] + f['y2']) / 2,
                 math.hypot(dx, dy), f['thickness'], math.atan2(dy, dx))


def static_obstacles(layout):
    """地図に載せる静的障害物の形状リスト(動物は含めない)."""
    shapes = [s for _, s in wall_rects(layout)]
    for b in layout.get('buildings', []):
        shapes.append(_rect(b['x'], b['y'], b['size_x'], b['size_y'], b.get('yaw', 0.0)))
    r = layout['tree']['trunk_radius']
    for x, y in layout.get('trees', []):
        shapes.append(_circle(x, y, r))
    for x, y in layout.get('poles', []):
        shapes.append(_circle(x, y, layout['pole']['radius']))
    for f in layout.get('fences', []):
        shapes.append(fence_rect(f))
    for b in layout.get('boxes', []):
        shapes.append(_rect(b['x'], b['y'], b['size_x'], b['size_y'], b.get('yaw', 0.0)))
    for k in layout.get('keepout', []):
        shapes.append(_rect((k['x_min'] + k['x_max']) / 2, (k['y_min'] + k['y_max']) / 2,
                            k['x_max'] - k['x_min'], k['y_max'] - k['y_min']))
    return shapes


def _bbox(shape):
    if shape[0] == 'circle':
        _, cx, cy, r = shape
        return cx - r, cx + r, cy - r, cy + r
    _, cx, cy, sx, sy, yaw = shape
    ex = abs(sx / 2 * math.cos(yaw)) + abs(sy / 2 * math.sin(yaw))
    ey = abs(sx / 2 * math.sin(yaw)) + abs(sy / 2 * math.cos(yaw))
    return cx - ex, cx + ex, cy - ey, cy + ey


def _inside(shape, x, y):
    if shape[0] == 'circle':
        _, cx, cy, r = shape
        return (x - cx) ** 2 + (y - cy) ** 2 <= r * r
    _, cx, cy, sx, sy, yaw = shape
    dx, dy = x - cx, y - cy
    c, s = math.cos(yaw), math.sin(yaw)
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    return abs(lx) <= sx / 2 and abs(ly) <= sy / 2


def animal_footprint(a):
    """動物の衝突箱の床面投影(SDF の collision と同じ大きさ)."""
    if a['species'] == 'bear':
        sx, sy, off = 1.3, 0.6, 0.05
    else:
        sx, sy, off = 1.0, 0.35, 0.1
    yaw = a.get('yaw', 0.0)
    return _rect(a['x'] + off * math.cos(yaw), a['y'] + off * math.sin(yaw), sx, sy, yaw)


def rasterize(layout, resolution=None, extra_shapes=()):
    """占有格子を返す. grid[row][col], row=0 が y_min 側(ROS の OccupancyGrid と同じ向き).

    extra_shapes: 静的障害物に加えて塗りつぶす形状(机上シミュレーションで動物を足す等)
    """
    res = resolution or layout.get('map_resolution', 0.05)
    b = layout['bounds']
    width = int(round((b['x_max'] - b['x_min']) / res))
    height = int(round((b['y_max'] - b['y_min']) / res))
    grid = [[FREE] * width for _ in range(height)]
    for shape in list(static_obstacles(layout)) + list(extra_shapes):
        x0, x1, y0, y1 = _bbox(shape)
        c0 = max(0, int(math.floor((x0 - b['x_min']) / res)))
        c1 = min(width - 1, int(math.floor((x1 - b['x_min']) / res)))
        r0 = max(0, int(math.floor((y0 - b['y_min']) / res)))
        r1 = min(height - 1, int(math.floor((y1 - b['y_min']) / res)))
        for row in range(r0, r1 + 1):
            y = b['y_min'] + (row + 0.5) * res
            line = grid[row]
            for col in range(c0, c1 + 1):
                # セルの中心か、形状がセルより細い場合に備えてセル内の点も確認
                x = b['x_min'] + (col + 0.5) * res
                if _inside(shape, x, y) or any(
                        _inside(shape, x + ox, y + oy)
                        for ox in (-res / 2, res / 2) for oy in (-res / 2, res / 2)):
                    line[col] = OCCUPIED
    return grid


def is_free(grid, layout, x, y, resolution=None):
    res = resolution or layout.get('map_resolution', 0.05)
    b = layout['bounds']
    col = int((x - b['x_min']) / res)
    row = int((y - b['y_min']) / res)
    return grid[row][col] == FREE


def write_pgm(grid, path):
    """map_server 形式の PGM(P5)を書く. 画像の上端 = y_max 側."""
    height = len(grid)
    width = len(grid[0])
    data = bytearray()
    for row in reversed(range(height)):
        data.extend(0 if v == OCCUPIED else 254 for v in grid[row])
    with open(path, 'wb') as f:
        f.write(b'P5\n# KumaWatch Patrol map (generated from world_layout.yaml)\n')
        f.write(b'%d %d\n255\n' % (width, height))
        f.write(bytes(data))


def write_map_yaml(path, image_name, layout, resolution=None):
    res = resolution or layout.get('map_resolution', 0.05)
    b = layout['bounds']
    with open(path, 'w', encoding='utf-8') as f:
        f.write('# generated by scripts/generate_world.py from world_layout.yaml\n')
        f.write('image: %s\n' % image_name)
        f.write('mode: trinary\n')
        f.write('resolution: %.3f\n' % res)
        f.write('origin: [%.3f, %.3f, 0.0]\n' % (b['x_min'], b['y_min']))
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.25\n')


# ---------------------------------------------------------------------------
# SDF 生成
# ---------------------------------------------------------------------------

def _rgb(c, a=1.0):
    return '%.3f %.3f %.3f %.2f' % (c[0], c[1], c[2], a)


def _material(color):
    return ('<material><ambient>%s</ambient><diffuse>%s</diffuse>'
            '<specular>0.05 0.05 0.05 1</specular></material>' % (_rgb(color), _rgb(color)))


def _box_geom(sx, sy, sz):
    return '<geometry><box><size>%.3f %.3f %.3f</size></box></geometry>' % (sx, sy, sz)


def _cyl_geom(r, length):
    return '<geometry><cylinder><radius>%.3f</radius><length>%.3f</length></cylinder></geometry>' % (
        r, length)


def _sphere_geom(r):
    return '<geometry><sphere><radius>%.3f</radius></sphere></geometry>' % r


def _pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return '<pose>%.3f %.3f %.3f %.3f %.3f %.3f</pose>' % (x, y, z, roll, pitch, yaw)


def _visual(name, pose, geom, color):
    return '<visual name="%s">%s%s%s</visual>' % (name, pose, geom, _material(color))


def _collision(name, pose, geom):
    return '<collision name="%s">%s%s</collision>' % (name, pose, geom)


def _model(name, pose, body):
    return ('<model name="%s"><static>true</static>%s<link name="link">%s</link></model>'
            % (name, pose, body))


def _box_model(name, x, y, sx, sy, sz, yaw, color, collide=True, z0=0.0):
    geom = _box_geom(sx, sy, sz)
    local = _pose(0, 0, z0 + sz / 2)
    body = _visual('visual', local, geom, color)
    if collide:
        body += _collision('collision', local, geom)
    return _model(name, _pose(x, y, 0, 0, 0, yaw), body)


def building_model(b):
    sx, sy, h = b['size_x'], b['size_y'], b['height']
    body = _visual('walls', _pose(0, 0, h / 2), _box_geom(sx, sy, h), b['color'])
    body += _collision('collision', _pose(0, 0, h / 2), _box_geom(sx, sy, h))
    # 切妻屋根: 2枚の傾いた板
    slope = 0.5
    half = sy / 2 + 0.2
    roof_w = half / math.cos(slope)
    for i, sgn in enumerate((1, -1)):
        body += _visual('roof_%d' % i,
                        _pose(0, sgn * half / 2, h + half * math.tan(slope) / 2, sgn * -slope, 0, 0),
                        _box_geom(sx + 0.3, roof_w, 0.08), b['roof_color'])
    return _model(b['name'], _pose(b['x'], b['y'], 0, 0, 0, b.get('yaw', 0.0)), body)


def tree_model(i, x, y, spec):
    r, th, cr = spec['trunk_radius'], spec['trunk_height'], spec['canopy_radius']
    body = _visual('trunk', _pose(0, 0, th / 2), _cyl_geom(r, th), (0.35, 0.22, 0.10))
    body += _collision('collision', _pose(0, 0, th / 2), _cyl_geom(r, th))
    body += _visual('canopy', _pose(0, 0, th + cr * 0.6), _sphere_geom(cr), (0.10, 0.38, 0.12))
    body += _visual('canopy_top', _pose(0, 0, th + cr * 1.4), _sphere_geom(cr * 0.65),
                    (0.12, 0.42, 0.14))
    return _model('tree_%02d' % i, _pose(x, y, 0), body)


def pole_model(i, x, y, spec):
    r, h = spec['radius'], spec['height']
    body = _visual('pole', _pose(0, 0, h / 2), _cyl_geom(r, h), spec['color'])
    body += _collision('collision', _pose(0, 0, h / 2), _cyl_geom(r, h))
    body += _visual('crossarm', _pose(0, 0, h - 0.2), _box_geom(0.8, 0.08, 0.08), (0.3, 0.3, 0.3))
    return _model('pole_%02d' % i, _pose(x, y, 0), body)


def bear_model(a):
    """熊(ツキノワグマ風)をプリミティブで組み立てる. 前方 = +x."""
    fur = (0.10, 0.07, 0.05)
    muzzle = (0.45, 0.33, 0.22)
    chest = (0.85, 0.82, 0.75)
    body = ''
    body += _visual('body', _pose(0, 0, 0.62), _box_geom(1.10, 0.60, 0.55), fur)
    body += _visual('hump', _pose(0.25, 0, 0.88), _sphere_geom(0.26), fur)
    body += _visual('rump', _pose(-0.42, 0, 0.66), _sphere_geom(0.30), fur)
    body += _visual('head', _pose(0.68, 0, 0.82), _sphere_geom(0.24), fur)
    body += _visual('muzzle', _pose(0.90, 0, 0.76), _box_geom(0.18, 0.14, 0.12), muzzle)
    body += _visual('nose', _pose(0.99, 0, 0.78), _sphere_geom(0.035), (0.02, 0.02, 0.02))
    body += _visual('ear_l', _pose(0.62, 0.15, 1.02), _sphere_geom(0.07), fur)
    body += _visual('ear_r', _pose(0.62, -0.15, 1.02), _sphere_geom(0.07), fur)
    body += _visual('eye_l', _pose(0.86, 0.09, 0.88), _sphere_geom(0.025), (0.0, 0.0, 0.0))
    body += _visual('eye_r', _pose(0.86, -0.09, 0.88), _sphere_geom(0.025), (0.0, 0.0, 0.0))
    body += _visual('chest_mark', _pose(0.56, 0, 0.60), _box_geom(0.04, 0.40, 0.10), chest)
    for n, (lx, ly) in enumerate(((0.38, 0.2), (0.38, -0.2), (-0.38, 0.2), (-0.38, -0.2))):
        body += _visual('leg_%d' % n, _pose(lx, ly, 0.2), _cyl_geom(0.09, 0.40), fur)
    # LiDAR(高さ約 0.2 m)に確実に写るよう、胴体全体を覆う衝突箱を置く
    body += _collision('collision', _pose(0.05, 0, 0.5), _box_geom(1.3, 0.6, 1.0))
    return _model(a['id'], _pose(a['x'], a['y'], 0, 0, 0, a.get('yaw', 0.0)), body)


def deer_model(a):
    """熊以外の動物(シカ). 前方 = +x."""
    coat = (0.55, 0.38, 0.20)
    body = ''
    body += _visual('body', _pose(0, 0, 0.85), _box_geom(0.90, 0.32, 0.35), coat)
    body += _visual('neck', _pose(0.45, 0, 1.10), _box_geom(0.14, 0.14, 0.45), coat)
    body += _visual('head', _pose(0.58, 0, 1.33), _box_geom(0.28, 0.14, 0.14), coat)
    body += _visual('tail', _pose(-0.47, 0, 0.95), _sphere_geom(0.06), (0.9, 0.9, 0.9))
    for n, (lx, ly) in enumerate(((0.35, 0.1), (0.35, -0.1), (-0.35, 0.1), (-0.35, -0.1))):
        body += _visual('leg_%d' % n, _pose(lx, ly, 0.34), _cyl_geom(0.035, 0.68), coat)
    body += _collision('collision', _pose(0.1, 0, 0.6), _box_geom(1.0, 0.35, 1.2))
    return _model(a['id'], _pose(a['x'], a['y'], 0, 0, 0, a.get('yaw', 0.0)), body)


def point_marker_model(p):
    """巡回候補地点を示す地面の円(見た目のみ)."""
    body = _visual('disc', _pose(0, 0, 0.006), _cyl_geom(0.35, 0.004), (0.95, 0.85, 0.10))
    body += _visual('center', _pose(0, 0, 0.008), _cyl_geom(0.08, 0.004), (0.10, 0.10, 0.10))
    return _model('patrol_point_%s' % p['id'], _pose(p['x'], p['y'], 0), body)


GROUND = (
    '<model name="ground_plane"><static>true</static><link name="link">'
    '<collision name="collision"><geometry><plane><normal>0 0 1</normal><size>60 60</size>'
    '</plane></geometry><surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction>'
    '</surface></collision>'
    '<visual name="visual"><cast_shadows>false</cast_shadows><geometry><plane>'
    '<normal>0 0 1</normal><size>60 60</size></plane></geometry>'
    '<material><ambient>0.30 0.45 0.22 1</ambient><diffuse>0.30 0.45 0.22 1</diffuse>'
    '</material></visual></link></model>'
)

SUN = (
    '<light name="sun" type="directional"><cast_shadows>false</cast_shadows>'
    '<pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.85 1</diffuse>'
    '<specular>0.2 0.2 0.2 1</specular><attenuation><range>1000</range>'
    '<constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>'
    '<direction>-0.4 0.3 -0.9</direction></light>'
)


def build_world_sdf(layout, patrol_points=()):
    """ワールド全体の SDF 文字列を返す."""
    parts = [GROUND, SUN]
    h = layout['wall']['height']
    for name, (_, cx, cy, sx, sy, yaw) in wall_rects(layout):
        parts.append(_box_model(name, cx, cy, sx, sy, h, yaw, (0.60, 0.60, 0.58)))
    for i, s in enumerate(layout.get('surfaces', [])):
        parts.append(_box_model(
            'surface_%s' % s['name'],
            (s['x_min'] + s['x_max']) / 2, (s['y_min'] + s['y_max']) / 2,
            s['x_max'] - s['x_min'], s['y_max'] - s['y_min'], 0.002, 0.0, s['color'],
            collide=False, z0=0.001 * (i + 1)))
    for b in layout.get('buildings', []):
        parts.append(building_model(b))
    for i, (x, y) in enumerate(layout.get('trees', [])):
        parts.append(tree_model(i, x, y, layout['tree']))
    for i, (x, y) in enumerate(layout.get('poles', [])):
        parts.append(pole_model(i, x, y, layout['pole']))
    for f in layout.get('fences', []):
        _, cx, cy, length, t, yaw = fence_rect(f)
        parts.append(_box_model(f['name'], cx, cy, length, t, f['height'], yaw, f['color']))
    for b in layout.get('boxes', []):
        parts.append(_box_model(b['name'], b['x'], b['y'], b['size_x'], b['size_y'],
                                b['size_z'], b.get('yaw', 0.0), b['color']))
    for a in layout.get('animals', []):
        if a['species'] == 'bear':
            parts.append(bear_model(a))
        elif a['species'] == 'deer':
            parts.append(deer_model(a))
        else:
            raise ValueError('unknown species %r' % a['species'])
    for p in patrol_points:
        parts.append(point_marker_model(p))

    header = (
        '<?xml version="1.0"?>\n'
        '<!-- KumaWatch Patrol world: generated by scripts/generate_world.py '
        'from world_layout.yaml. Do not edit by hand. -->\n'
        '<sdf version="1.6">\n<world name="kumawatch_patrol">\n'
        '<gravity>0 0 -9.8</gravity>\n'
        '<physics type="ode"><max_step_size>0.001</max_step_size>'
        '<real_time_factor>1</real_time_factor>'
        '<real_time_update_rate>1000</real_time_update_rate></physics>\n'
        '<scene><ambient>0.6 0.6 0.6 1</ambient><background>0.62 0.78 0.92 1</background>'
        '<shadows>false</shadows></scene>\n'
        '<gui fullscreen="0"><camera name="user_camera">'
        '<pose>0 -19 17 0 0.78 1.5708</pose><view_controller>orbit</view_controller>'
        '</camera></gui>\n'
        # 動画用の追従カメラ(chase_camera ノード)がモデルの位置を読み書きするためのサービス
        '<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">'
        '<ros><namespace>/gazebo</namespace></ros><update_rate>1.0</update_rate></plugin>\n'
    )
    return pretty_xml(header + '\n'.join(parts) + '\n</world>\n</sdf>\n')


def pretty_xml(text):
    """読みやすいように字下げした XML を返す(コメントと宣言は保持)."""
    from xml.dom import minidom

    pretty = minidom.parseString(text).toprettyxml(indent='  ')
    lines = [ln for ln in pretty.splitlines() if ln.strip()]
    # minidom は宣言を 1 行目に出すので、元の宣言に揃える
    lines[0] = '<?xml version="1.0"?>'
    return '\n'.join(lines) + '\n'
