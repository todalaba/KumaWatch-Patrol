# 学習データと学習プログラム

KumaWatch Patrol の 2 つの AI について、入力データと、学習・推論に使ったプログラムをまとめたフォルダです。

| AI | 学習の有無 | このフォルダにあるもの | プログラム |
|---|---|---|---|
| AI 1：熊出没リスク（TTM） | **本作品では学習していない**（IBM が事前学習したモデルをゼロショットで使用） | なし（公開用に事前計算した 5 地点のスコアだけを `src/kumawatch_ai/data/` に収録） | `../scripts/import_kumawatch_ttm.py`（5 地点への割り当て。KumaWatch の非公開リポジトリが必要） |
| AI 2：熊の画像判定（YOLOv8） | **追加学習した** | 学習画像 700 枚と正解ラベル、学習の設定と各エポックの結果 | `../scripts/generate_bear_dataset.py`（データ生成）、`../scripts/train_bear_detector.py`（学習） |

---

## 1. TTM（このリポジトリには入力データと推論プログラムを含めない）

本リポジトリでは、公開用に事前計算済みのリスクスコアを使用する。実際の自治体データおよび非公開研究コードは含まない。

- 使用するのは `src/kumawatch_ai/data/kumawatch_ttm_scores.csv`（5 地点分）だけです。研究プロジェクト KumaWatch（非公開の研究リポジトリ）で
  IBM Granite TinyTimeMixer（TTM 1536-96-R2）を **追加学習せず（ゼロショット）** 実行した 2025-06-07 の予測を、
  `scripts/import_kumawatch_ttm.py` で架空の 5 地点に割り当てたものです。
- 予測の元になった山形県の 10 km グリッドの日別出没件数（山形県の公開野生動物出没データ、CC BY 4.0 を集計したもの）、
  1 年分の予測結果、推論のノートブック（IBM watsonx.ai の API を呼ぶもの）は含めていません。
- 各列：`risk`（その日の県内 144 セル中の順位を百分位にした値）、`ttm_score`（TTM の予測値）、`ttm_rank`、
  `observed_sightings`（その日のそのセルの出没件数。Gazebo で熊を置く地点の決定に使用）。

---

## 2. YOLOv8（`yolo/`）

COCO で学習済みの YOLOv8n を、Gazebo で撮影した画像で追加学習しました。

| ファイル | 内容 |
|---|---|
| `dataset/images/{train,val}/*.png` | 学習画像 595 枚・検証画像 105 枚（640×480、Gazebo でレンダリング） |
| `dataset/labels/{train,val}/*.txt` | 正解ラベル（YOLO 形式：`クラス x中心 y中心 幅 高さ`、画像サイズで正規化。クラス 0 = bear）。熊が写っていない画像は空ファイル |
| `dataset/data.yaml` | データセット設定 |
| `run/args.yaml` | 学習時の設定（エポック数・画像サイズ・データ拡張など、Ultralytics が保存したもの） |
| `run/results.csv` | 各エポックの損失と検証指標（precision / recall / mAP） |
| `run/labels.jpg`・`run/train_batch0.jpg` | 正解ラベルの分布と、学習バッチの例（データ拡張後の画像と枠） |

データの作り方（`scripts/generate_bear_dataset.py`）：

- 本番と同じ Gazebo ワールドにカメラを 1 台置き、カメラと熊 2 頭・シカ 1 頭をランダムな位置・向きへ動かしながら撮影した（乱数の種 7）。
- カメラの高さは waffle（約 0.2 m）と Happy Mini（約 1 m）の 2 通り、熊までの距離は 1.2〜7 m。
- 正解の枠は、熊の 3D の大きさ（SDF と同じ）をカメラ画像へ投影して自動で計算した。手作業のラベル付けはしていない。
- 木や建物で熊が隠れる視点、画面の端に少しだけ写る曖昧な視点は除外した。
- 約 3 割は熊が写らない画像（シカ・木・建物・何もない方向）。

学習（`scripts/train_bear_detector.py`）：

```bash
pip3 install ultralytics
python3 scripts/train_bear_detector.py --data training/yolo/dataset/data.yaml --epochs 40
```

- 初期値：`yolov8n.pt`（COCO 学習済み）、画像サイズ 416、バッチ 16、CPU、乱数の種 0。
- 40 エポックの予定だったが、検証の精度が頭打ちになったため 23 エポック目で止め、それまでの最良の重みを使った
  （`src/kumawatch_ai/models/kumawatch_bear_yolov8n.pt`）。
- 検証 105 枚：precision 0.94、recall 0.99、mAP50 0.985、mAP50-95 0.815。

`dataset/data.yaml` の `path` は、このフォルダからの相対パスにしてあるので、そのまま学習に使えます。

---

## 参考：模擬データのロジスティック回帰（旧版）

レベル1で使っていた模擬データのリスクモデルは、データを学習時に乱数で生成するので、データファイルはありません。
`scripts/train_risk_model.py`（乱数の種 2026）を実行すると、同じデータ・同じモデルが再現されます。
