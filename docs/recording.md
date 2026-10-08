# 動画の撮り方（提出動画 3 分 30 秒向け）

KumaWatch Patrol の動画素材を撮る方法をまとめます。
講義の Docker 環境（[docs/lecture_env.md](lecture_env.md)）でも、Ubuntu 22.04 の PC でも使えます。

| 撮りたいもの | 方法 | 手軽さ |
|---|---|---|
| Gazebo・RViz・端末をまとめた画面 | **A. ホスト PC の画面録画**（OBS など） | ◎ いちばん簡単 |
| 同上（コンテナ内で録画） | **B. `scripts/record_screen.sh`**（ffmpeg） | ○ ホストに何も入れなくてよい |
| ロボット視点のカメラ映像と、YOLO の検出枠 | **C. `record:=true`**（`image_recorder` ノード） | ◎ 熊を検知した瞬間の静止画も保存 |
| 後から何度でも撮り直せる記録 | **D. rosbag** | ○ 同じ走行を別の画面構成で撮れる |

---

## A. ホスト PC の画面録画（おすすめ）

講義環境はブラウザ（`localhost:6080`）に表示されるので、ホスト PC の録画機能でブラウザを撮るのがいちばん確実です。
画質も良く、コンテナの負荷も増えません。

| OS | 方法 |
|---|---|
| Windows | **OBS Studio**（無料）の「ウィンドウキャプチャ」でブラウザを指定。簡易的には Xbox Game Bar（`Win + Alt + R` で録画開始／停止） |
| macOS | **OBS Studio**、または `Cmd + Shift + 5` →「選択部分を収録」 |
| Linux | **OBS Studio**、または GNOME の `Ctrl + Shift + Alt + R` |

きれいに撮るコツ:

1. コンテナを `-e RESOLUTION=1920x1080` で起動し（[lecture_env.md](lecture_env.md) の手順 1）、
   ブラウザを全画面（`F11`）にします。noVNC の設定（左端のタブ）でスケーリングを「ローカル」にします。
2. デスクトップ上で Gazebo（左）と RViz（右）を並べ、下に端末を小さく置くと、1 画面で全体が伝わります。
3. OBS の設定: 出力を「録画フォーマット mp4」「エンコーダ x264（または NVENC）」「30 fps」にします。

## B. コンテナ内で画面を録画する（`scripts/record_screen.sh`）

ホストに録画ソフトを入れられない場合は、コンテナ内のデスクトップを ffmpeg で録画します。

```bash
cd ~/airobot_ws/src/KumaRobot
./scripts/record_screen.sh            # 初回は ffmpeg を自動でインストール。Ctrl+C で停止・保存
./scripts/record_screen.sh -r 20 -o ~/demo.mp4   # フレームレートと保存先を指定
```

- 保存先の既定は `~/kumawatch_patrol_output/videos/screen_<日時>.mp4` です
  （H.264 形式で、Windows・Mac・スマホで再生できます）。
- 別の端末で録画を始めてから `ros2 launch kumawatch_mission kumawatch_patrol.launch.py` を起動します。
- CPU を使うので、Gazebo が重くなる場合は `-r 10` に下げるか、A の方法を使ってください。

## C. ロボットのカメラ映像を録画する（`image_recorder`）

ロボット（waffle / happy_mini）のカメラ `/camera/image_raw` を MP4 に保存します。

- 映像の上部に進み具合（`Mission 1/3: P2 Forest edge` / `AI risk 0.98 | Navigating | bears 0 | 12.3 m`）を表示します。
- 熊を検知したときは画面を赤枠で囲み、下部に大きなパネル（`BEAR DETECTED` / 信頼度・距離 / 地点 / 行動）を表示し、その瞬間の静止画（JPEG）も保存します。
- 録画の開始時刻を `<動画名>.json` に書きます（動画の自動編集で使います。[docs/video_plan.md](video_plan.md)）。

```bash
ros2 launch kumawatch_mission kumawatch_patrol.launch.py record:=true
```

または、すでに起動している巡回に後から追加します。

```bash
ros2 run kumawatch_mission image_recorder --ros-args -p use_sim_time:=true
```

- 保存先: `~/kumawatch_patrol_output/videos/` に次の 3 種類
  - `<日時>_camera.mp4`: カメラ映像そのもの（30 fps）
  - `<日時>_detection.mp4`: YOLO の検出枠と「bear 0.93 3.2m」の表示入り（推論頻度 5 fps）。動画の「熊検知」の場面に使えます
  - `bear_<ID>_<日時>.jpg`: 熊を検知した瞬間の静止画
- `Ctrl+C` で停止すると MP4 が閉じられます（強制終了するとファイルが壊れます）。
- 動画編集ソフトで読み込めない場合は、H.264 に変換します:
  `ffmpeg -i in.mp4 -c:v libx264 -pix_fmt yuv420p out.mp4`

### 録画がうまくいかないとき

録画中は 10 秒ごとに次のような表示が出ます。まずこれを見てください。

```
[image_recorder]: /camera/image_raw を購読しました(QoS: Reliable)
[image_recorder]: 受信 23 枚(1.4 枚/秒)、動画 348 フレーム = 11.6 秒
```

| 症状 | 原因と対処 |
|---|---|
| 「受信 ○ 枚」がほとんど増えない | Gazebo がカメラ画像をほとんど描画できていません（GPU なしで 1920×1080 の waffle カメラは重い）。`robot_model:=happy_mini`（640×480）で起動するか、Gazebo の画面を消す（`gui:=false`）。`ros2 topic hz /camera/image_raw` で届く頻度を確認できます |
| 「画像がまだ 1 枚も届いていません」 | `ros2 topic list \| grep image` でトピック名を確認し、`-p image_topic:=...` で指定する |
| 左上の状態が「-」のまま | `mission_planner` が動いていません（巡回を起動する前に録画だけ始めた場合など）。映像は録れています |
| 動画が再生できない（壊れている） | `Ctrl+C` 以外で止めた（端末を閉じた・`kill -9`）。`Ctrl+C` で止めると正しく保存されます |

- 画像がまばらにしか届かなくても、画像の時刻に合わせてフレームを並べるので、動画の長さは撮影していた時間と同じになります（コマ送りのような動きになります）。
- 横幅 1280 px を超える画像は縮小して保存します（`-p max_width:=0` で縮小しない）。

## C2. ロボットを追いかける映像（`chase_camera`）

Gazebo にカメラを 1 台出して、ロボットの後ろ斜め上から撮り続けます。俯瞰よりロボットの動きと向きが分かります。

```bash
ros2 run kumawatch_mission chase_camera --ros-args -p use_sim_time:=true -p mode:=chase   # far / top も可
ros2 run kumawatch_mission image_recorder --ros-args -p use_sim_time:=true \
    -p image_topic:=/chase_cam/image_raw -p label:=chase -p fps:=15.0
```

Gazebo の画面そのものを追従させる場合は `./scripts/video/gazebo_follow.sh` を実行します。
提出動画の構成と編集の手順は [docs/video_plan.md](video_plan.md) を参照してください。

## D. rosbag で走行を記録して、後から撮り直す

一度の走行を記録しておけば、RViz の視点を変えながら何度でも撮り直せます。

```bash
# 記録(巡回を起動した後、別の端末で)
ros2 bag record -o patrol_ai /tf /tf_static /map /scan /odom /plan \
  /kumawatch/risk_markers /kumawatch/patrol_markers /kumawatch/mission_state \
  /kumawatch/bear_detection /kumawatch/patrol_summary

# 再生(シミュレーションを止めてから。RViz を開いて再生)
rviz2 -d ~/airobot_ws/install/kumawatch_navigation/share/kumawatch_navigation/rviz/kumawatch_patrol.rviz &
ros2 bag play patrol_ai --clock
```

カメラ映像（`/camera/image_raw`）も記録できますが、1 分で数 GB になるので、必要なときだけ加えてください。

## 撮影プランの例（企画書の動画構成に対応）

| 時間 | 場面 | 撮り方 |
|---|---|---|
| 0:00–0:25 | 社会的課題（住宅地と森林の境界） | Gazebo の俯瞰（A）。マウスでカメラをゆっくり回す |
| 0:25–0:50 | AI の判断（5 地点のリスク） | RViz のリスク棒グラフと端末の「巡回計画」表（A）、`docs/overview.png` |
| 0:50–2:00 | 自律巡回（2 倍速） | Gazebo と RViz を並べて録画（A または B） |
| 2:00–2:35 | 熊を AI で検知して停止 | 検出枠入りの映像（C の `_detection.mp4`）＋ 端末の `!!! 熊を検知` |
| 2:35–3:10 | 固定巡回との比較 | `strategy:=fixed` でもう一度撮影し、左右に並べる |
| 3:10–3:30 | まとめ | 端末の結果表と RViz の最終画面 |

### よく使う ffmpeg の編集コマンド

```bash
# 2 倍速(長い移動の場面)
ffmpeg -i in.mp4 -filter:v "setpts=0.5*PTS" -an out_2x.mp4

# 一部を切り出す(1 分 10 秒から 35 秒間)
ffmpeg -ss 00:01:10 -i in.mp4 -t 35 -c:v libx264 -pix_fmt yuv420p clip.mp4

# 固定巡回(左)と AI 巡回(右)を左右に並べる(高さを 720 に揃える)
ffmpeg -i fixed.mp4 -i ai.mp4 -filter_complex \
  "[0:v]scale=-2:720[l];[1:v]scale=-2:720[r];[l][r]hstack" -c:v libx264 -pix_fmt yuv420p compare.mp4
```

字幕や説明の文字は、動画編集ソフト（DaVinci Resolve、Clipchamp、iMovie など）で入れるのが簡単です。
