#!/usr/bin/env python3
"""Gazebo で作った画像で YOLOv8 を追加学習し、熊検出モデルを作る.

COCO で学習済みの YOLOv8n にも「bear」クラスはあるが、Gazebo の熊(基本図形の組み合わせ)は
熊と認識しない(信号機やフリスビーと誤認する)。そこで scripts/generate_bear_dataset.py で
自動ラベル付けした画像で追加学習(ファインチューニング)する。

使い方:
  pip3 install ultralytics
  python3 scripts/train_bear_detector.py --data datasets/kuma_bear/data.yaml --epochs 40

出力: src/kumawatch_ai/models/kumawatch_bear_yolov8n.pt(bear_detector ノードの既定の重み)
"""

import argparse
import os
import shutil

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEST = os.path.join(ROOT, 'src', 'kumawatch_ai', 'models', 'kumawatch_bear_yolov8n.pt')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', default=os.path.join(ROOT, 'datasets', 'kuma_bear', 'data.yaml'))
    ap.add_argument('--base', default='yolov8n.pt', help='初期値にする学習済みモデル')
    ap.add_argument('--epochs', type=int, default=40)
    ap.add_argument('--imgsz', type=int, default=416)
    ap.add_argument('--batch', type=int, default=16)
    ap.add_argument('--device', default='cpu', help='GPU があれば 0')
    ap.add_argument('--project', default=os.path.join(ROOT, 'runs'))
    args = ap.parse_args()

    from ultralytics import YOLO

    model = YOLO(args.base)
    model.train(data=args.data, epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
                device=args.device, project=args.project, name='bear', exist_ok=True, seed=0,
                patience=15, verbose=False)
    best = os.path.join(args.project, 'bear', 'weights', 'best.pt')
    metrics = YOLO(best).val(data=args.data, imgsz=args.imgsz, device=args.device, verbose=False)
    print('val: precision %.3f  recall %.3f  mAP50 %.3f  mAP50-95 %.3f' % (
        metrics.box.mp, metrics.box.mr, metrics.box.map50, metrics.box.map))
    os.makedirs(os.path.dirname(DEST), exist_ok=True)
    shutil.copy(best, DEST)
    print('→', DEST)


if __name__ == '__main__':
    main()
