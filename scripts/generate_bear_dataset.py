#!/usr/bin/env python3
"""Gazebo で熊検知(YOLO)用の学習画像とラベルを自動生成する.

KumaWatch Patrol のワールドにカメラを 1 台置き、gazebo_ros_state でカメラと熊・シカを
ランダムな位置へ動かしながら撮影する。熊の 3D の大きさ(SDF と同じ)をカメラ画像へ投影して
バウンディングボックスを計算するので、手作業のラベル付けは不要。
木・建物で熊が完全に隠れる視点は地図上のレイキャストで除外する。

使い方(ROS 2 Humble + Gazebo Classic。別端末で先に gzserver を起動しておく):
  python3 scripts/generate_bear_dataset.py --make-world /tmp/bear_dataset.world
  gzserver /tmp/bear_dataset.world -slibgazebo_ros_init.so        # 別端末
  python3 scripts/generate_bear_dataset.py --out datasets/kuma_bear --n 600

出力(YOLO 形式):
  <out>/images/{train,val}/*.png, <out>/labels/{train,val}/*.txt, <out>/data.yaml
"""

import argparse
import math
import os
import random
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'kumawatch_gazebo'))
from kumawatch_gazebo import world_builder as wb  # noqa: E402

LAYOUT = os.path.join(ROOT, 'src', 'kumawatch_gazebo', 'config', 'world_layout.yaml')
WORLD = os.path.join(ROOT, 'src', 'kumawatch_gazebo', 'worlds', 'kumawatch_patrol.world')

# 熊モデル(world_builder.bear_model)の見た目の外接直方体 [x0, x1, y0, y1, z0, z1](モデル座標)
BEAR_BOX = (-0.72, 1.03, -0.30, 0.30, 0.0, 1.09)
CAMERA_NAME = 'datacam'
CAM_W, CAM_H = 640, 480
CAM_HFOV = 1.06          # waffle(1.03)と happy_mini(1.086)の中間
CLASS_NAMES = ['bear']


def make_world(path):
    """学習データ撮影用のワールド(本番ワールド + カメラ)を書き出す."""
    with open(WORLD, encoding='utf-8') as f:
        world = f.read()
    # gazebo_ros_state(カメラ・熊を動かすサービス)は本番ワールドに含まれている
    extra = f'''
    <model name="{CAMERA_NAME}">
      <static>true</static>
      <pose>0 -8 0.3 0 0 1.5708</pose>
      <link name="link">
        <sensor name="cam" type="camera">
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <camera>
            <horizontal_fov>{CAM_HFOV}</horizontal_fov>
            <image><width>{CAM_W}</width><height>{CAM_H}</height></image>
            <clip><near>0.05</near><far>60</far></clip>
          </camera>
          <plugin name="datacam" filename="libgazebo_ros_camera.so">
            <ros><namespace>/datacam</namespace></ros>
            <camera_name>cam</camera_name>
          </plugin>
        </sensor>
      </link>
    </model>
  </world>'''
    world = world.replace('</world>', extra, 1)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(world)
    print('wrote', path)


def project_box(cam, obj, box=BEAR_BOX, hfov=CAM_HFOV, w=CAM_W, h=CAM_H):
    """物体の外接直方体を画像へ投影し、(x0, y0, x1, y1, 見えている割合) を返す. 見えなければ None.

    cam: (x, y, z, yaw)、obj: (x, y, yaw)。カメラは水平(ピッチ 0)で +x 方向を向く。
    """
    f = (w / 2.0) / math.tan(hfov / 2.0)
    us, vs = [], []
    cy, sy = math.cos(obj[2]), math.sin(obj[2])
    cc, sc = math.cos(cam[3]), math.sin(cam[3])
    for bx in box[0:2]:
        for by in box[2:4]:
            for bz in box[4:6]:
                wx = obj[0] + cy * bx - sy * by
                wy = obj[1] + sy * bx + cy * by
                dx, dy, dz = wx - cam[0], wy - cam[1], bz - cam[2]
                xc = cc * dx + sc * dy          # 前方
                yc = -sc * dx + cc * dy         # 左
                if xc < 0.1:
                    return None                 # カメラの後ろ・近すぎ
                us.append(w / 2.0 - f * yc / xc)
                vs.append(h / 2.0 - f * dz / xc)
    x0, x1, y0, y1 = min(us), max(us), min(vs), max(vs)
    full = (x1 - x0) * (y1 - y0)
    cx0, cx1 = max(0.0, x0), min(float(w), x1)
    cy0, cy1 = max(0.0, y0), min(float(h), y1)
    if cx1 <= cx0 or cy1 <= cy0 or full <= 0:
        return None
    return cx0, cy0, cx1, cy1, (cx1 - cx0) * (cy1 - cy0) / full


def occluded(grid, layout, a, b, step=0.05):
    """地図上で a→b の直線が静的障害物(木・建物など)に遮られるか."""
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(d / step))
    for i in range(1, n - int(0.8 / step)):     # 熊自身の手前 0.8 m までを調べる
        t = i / n
        x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
        if not is_free(grid, layout, x, y, margin=0.0):
            return True
    return False


def is_free(grid, layout, x, y, margin=0.3):
    """地図の範囲内で、障害物から margin 以上離れているか."""
    b = layout['bounds']
    if not (b['x_min'] + margin < x < b['x_max'] - margin
            and b['y_min'] + margin < y < b['y_max'] - margin):
        return False
    return all(wb.is_free(grid, layout, x + dx, y + dy)
               for dx in (-margin, 0, margin) for dy in (-margin, 0, margin))


def free_spot(grid, layout, rng, margin=1.0):
    b = layout['bounds']
    while True:
        x = rng.uniform(b['x_min'] + 1.5, 8.0)
        y = rng.uniform(b['y_min'] + 1.5, b['y_max'] - 1.5)
        if is_free(grid, layout, x, y, margin):
            return x, y


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--make-world', help='撮影用ワールドを書き出して終了')
    ap.add_argument('--out', default=os.path.join(ROOT, 'datasets', 'kuma_bear'))
    ap.add_argument('--n', type=int, default=600)
    ap.add_argument('--val-ratio', type=float, default=0.15)
    ap.add_argument('--neg-ratio', type=float, default=0.3, help='熊が写らない画像の割合')
    ap.add_argument('--seed', type=int, default=7)
    args = ap.parse_args()
    if args.make_world:
        make_world(args.make_world)
        return

    import cv2
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import SetEntityState
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    layout = wb.load_yaml(LAYOUT)
    grid = wb.rasterize(layout)
    rng = random.Random(args.seed)
    animals = {a['id']: dict(a) for a in layout['animals']}
    bears = [k for k, a in animals.items() if a['species'] == 'bear']
    others = [k for k, a in animals.items() if a['species'] != 'bear']

    rclpy.init()
    node = Node('bear_dataset_generator')
    cli = node.create_client(SetEntityState, '/gazebo/set_entity_state')
    if not cli.wait_for_service(timeout_sec=30.0):
        sys.exit('/gazebo/set_entity_state が見つかりません(--make-world のワールドで gzserver を起動してください)')
    latest = {}

    def on_image(msg):
        latest['msg'] = msg
        latest['t'] = time.time()

    node.create_subscription(Image, '/datacam/cam/image_raw', on_image, qos_profile_sensor_data)

    def set_pose(name, x, y, z, yaw):
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = name
        req.state.pose.position.x, req.state.pose.position.y = float(x), float(y)
        req.state.pose.position.z = float(z)
        req.state.pose.orientation.z = math.sin(yaw / 2)
        req.state.pose.orientation.w = math.cos(yaw / 2)
        req.state.reference_frame = 'world'
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)

    def grab(after):
        deadline = time.time() + 10.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if latest.get('t', 0) > after:
                m = latest['msg']
                img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
                return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return None

    for split in ('train', 'val'):
        os.makedirs(os.path.join(args.out, 'images', split), exist_ok=True)
        os.makedirs(os.path.join(args.out, 'labels', split), exist_ok=True)

    saved = pos = 0
    while saved < args.n:
        # 動物をランダムな場所・向きに置き直す(背景を多様にする)
        poses = {}
        for k in bears + others:
            if rng.random() < 0.5:
                x, y = animals[k]['x'], animals[k]['y']     # 本番の配置
            else:
                x, y = free_spot(grid, layout, rng)
            poses[k] = (x, y, rng.uniform(-math.pi, math.pi))
            set_pose(k, poses[k][0], poses[k][1], 0.0, poses[k][2])

        negative = rng.random() < args.neg_ratio
        z = rng.choice([rng.uniform(0.15, 0.35), rng.uniform(0.85, 1.1)])   # waffle / happy_mini の高さ
        if negative:
            # シカ・木・建物・何もない方向を撮る
            tx, ty, _ = poses[rng.choice(others)] if rng.random() < 0.5 else (*free_spot(grid, layout, rng), 0)
            d = rng.uniform(1.5, 6.0)
        else:
            tx, ty, _ = poses[rng.choice(bears)]
            d = rng.uniform(1.2, 7.0)
        ang = rng.uniform(-math.pi, math.pi)
        cx, cy = tx + d * math.cos(ang), ty + d * math.sin(ang)
        if not is_free(grid, layout, cx, cy):
            continue
        yaw = math.atan2(ty - cy, tx - cx) + rng.uniform(-0.35, 0.35) * CAM_HFOV
        cam = (cx, cy, z, yaw)

        labels = []
        skip = False
        drawn = []          # 手前の熊から順に処理し、奥の熊が手前の熊に隠れる場合を除く
        for k in sorted(bears, key=lambda k: math.hypot(poses[k][0] - cx, poses[k][1] - cy)):
            box = project_box(cam, poses[k])
            if box is None:
                continue
            x0, y0, x1, y1, vis = box
            hidden = occluded(grid, layout, (cx, cy), poses[k][:2])
            area = (x1 - x0) * (y1 - y0)
            covered = max((max(0.0, min(x1, b[2]) - max(x0, b[0])) *
                           max(0.0, min(y1, b[3]) - max(y0, b[1])) for b in drawn), default=0.0)
            if covered > 0.5 * area:
                continue
            if hidden or vis < 0.35 or (x1 - x0) < 10 or (y1 - y0) < 10:
                skip = skip or (not hidden and vis >= 0.1)   # 端で少しだけ写るものは曖昧なので捨てる
                continue
            drawn.append((x0, y0, x1, y1))
            labels.append('0 %.6f %.6f %.6f %.6f' % (
                (x0 + x1) / 2 / CAM_W, (y0 + y1) / 2 / CAM_H, (x1 - x0) / CAM_W, (y1 - y0) / CAM_H))
        if skip or (not negative and not labels) or (negative and labels):
            continue

        set_pose(CAMERA_NAME, cx, cy, z, yaw)
        t0 = time.time()
        img = grab(t0 + 0.3)            # カメラ移動後の新しいフレームを待つ
        if img is None:
            print('カメラ画像が来ません', file=sys.stderr)
            continue
        split = 'val' if rng.random() < args.val_ratio else 'train'
        stem = 'img_%05d' % saved
        cv2.imwrite(os.path.join(args.out, 'images', split, stem + '.png'), img)
        with open(os.path.join(args.out, 'labels', split, stem + '.txt'), 'w') as f:
            f.write('\n'.join(labels) + ('\n' if labels else ''))
        saved += 1
        pos += bool(labels)
        if saved % 50 == 0:
            print('%d / %d 枚 (熊あり %d)' % (saved, args.n, pos), flush=True)

    with open(os.path.join(args.out, 'data.yaml'), 'w') as f:
        f.write('path: %s\ntrain: images/train\nval: images/val\nnames:\n  0: bear\n'
                % os.path.abspath(args.out))
    # 本番の配置に戻す
    for k, a in animals.items():
        set_pose(k, a['x'], a['y'], 0.0, a.get('yaw', 0.0))
    print('完了: %d 枚 (熊あり %d) → %s' % (saved, pos, args.out))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
