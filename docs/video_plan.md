# 説得力のある提出動画の作り方（KumaWatch Patrol）

今の動画（Gazebo・RViz・端末を 3 分間並べたもの）は、ROS 2 で自律走行していることは伝わります。
一方で「AI が何を判断し、ロボットが何を達成したか」が伝わりにくい、という課題がありました。
このページは、その改善のための仕組みと、撮影から編集までの手順をコマンドの単位でまとめたものです。

- 構成見本（既存の映像を再編集したもの）: `scripts/video/examples/draft_from_existing.yaml`
- 撮影のしかた（録画の基本）: [docs/recording.md](recording.md)

## 1. 課題と、それに対応する仕組み

| 課題 | 仕組み（このリポジトリで追加） | 使い方 |
|---|---|---|
| AI の役割が映像から分からない | AI の判断の表の画面（5 地点のリスク・効用・選んだ 3 地点）を自動で作る。数値はデータファイルから読む | `scripts/video/make_cards.py` の `03_ai_risk.png` |
| ロボットが小さく、動きを追いにくい | **追従カメラ**：Gazebo にカメラを出してロボットの後ろ斜め上から撮る（`chase_camera`）。Gazebo の画面自体を追従させる方法もある | `ros2 run kumawatch_mission chase_camera` / `scripts/video/gazebo_follow.sh` |
| LiDAR の青い扇形が邪魔 | `sensor_viz:=false` で LiDAR とカメラの視野表示を消して起動 | `ros2 launch ... sensor_viz:=false` |
| 熊の発見場面が明確でない | ロボットのカメラ映像に **HUD** を重ねる：上に「Mission 1/3: P2 Forest edge / AI risk 0.98 / Navigating」、検知時は赤枠と「BEAR DETECTED / Confidence / Location / Action」。YOLO の検出枠も入る。編集で効果音も入れる | `record:=true`（image_recorder） |
| 熊を見つけたあとの行動 | **安全地点への退避**：停止・記録のあと、熊から 3 m 離れた地点へ移動してから巡回を続ける（RViz に「SAFE POINT」） | `on_bear:=retreat` |
| 3 画面が小さく文字が読めない | 編集で場面ごとに拡大・並べ替え（Gazebo 大＋RViz、カメラ映像大、RViz 全画面など） | `scripts/video/compose.py` |
| 赤いログがエラーに見える | 進み具合だけを短い英文（緑）で出す端末 | `ros2 run kumawatch_mission mission_console` |
| 冒頭に目的の説明がない | タイトル・背景・システム構成の画面 | `make_cards.py`（01, 02, 04） |
| 到着が分かりにくい | HUD と字幕に「Mission i/3・地点名・Arrived - observing」 | `/kumawatch/mission_status` を HUD・字幕が使う |
| 映像の変化が少ない | 移動は倍速（「2× Speed」の札）、出発・熊・退避・到着は等速。場面ごとに画面構成を切り替える | `compose.py`（`speed`） |
| 完了が示されていない | 「Mission Completed」の結果画面（数値は実際の結果ファイルから） | `make_cards.py --summary` |
| AI の効果が伝わらない | 固定ルートと AI ルートの**走行軌跡を並べたアニメーション**と比較表（実際の走行結果。巡回地点の AI リスク合計も並べる） | `route_animation.py`、`make_cards.py --compare` |
| 画面録画に RViz が映っていない | 走行記録（位置と出来事）から、地図の上をロボットが動く映像を作って RViz の代わりにする（熊の発見位置・退避先・状態も表示） | `scripts/video/live_map.py` → `edl_from_run.py --map` |

さらに、編集のための記録を自動で残すようにしました。

- `mission_planner` が `~/kumawatch_patrol_output/<日時>_<strategy>/mission_events.csv` に、出来事（出発・到着・熊の検知・退避・帰還）の時刻を書く
- `record_screen.sh` と `image_recorder` が、録画の開始時刻を `<動画名>.json` に書く
- `scripts/video/edl_from_run.py` がこの 2 つを突き合わせ、**編集リストを自動で作る**（熊の検知の 3 秒前から等速、移動は倍速、など）

## 2. 撮影（講義の Docker 環境）

事前に最新版を取得して、ビルドし直します。

```bash
cd ~/airobot_ws/src/KumaRobot && git pull
cd ~/airobot_ws && colcon build --symlink-install --packages-up-to kumawatch_mission && source ~/.bashrc
```

端末を 5 つ開き、順に起動します（`robot_model:=happy_mini` はカメラが軽いのでおすすめです）。

```bash
# 端末1: 巡回(熊を見つけたら退避、LiDAR の扇形を消す、カメラ映像を HUD 付きで録画)
ros2 launch kumawatch_mission kumawatch_patrol.launch.py robot_model:=happy_mini \
    on_bear:=retreat sensor_viz:=false record:=true nav2_delay:=10

# 端末2: 追従カメラ(Gazebo が開いてから)と、その映像の録画
ros2 run kumawatch_mission chase_camera --ros-args -p use_sim_time:=true -p mode:=chase &
ros2 run kumawatch_mission image_recorder --ros-args -p use_sim_time:=true \
    -p image_topic:=/chase_cam/image_raw -p label:=chase -p fps:=15.0

# 端末3: 進み具合の表示(画面録画に映す端末はこれだけにする)
ros2 run kumawatch_mission mission_console

# 端末4: 画面全体の録画(RViz を映すため。巡回の開始前に始める)
cd ~/airobot_ws/src/KumaRobot && ./scripts/record_screen.sh
```

- 巡回が終わって「Mission completed」が出たら、端末4 → 端末2 → 端末1 の順に `Ctrl+C` で止めます（録画ファイルが正しく閉じられます）。
- スクリプトで自動化するときは、`&` で裏に回したプロセスには Ctrl+C（SIGINT）が届きません。
  `pkill -TERM -f kumawatch_mission/image_recorder` のように SIGTERM で止めてください（image_recorder は SIGTERM でも MP4 を閉じます）。
- 画面は、左に Gazebo、右に RViz、下に端末3 を小さく置くと、後の切り出しが楽です。
- `record_screen.sh` は `-s` を付けずに実行してください（画面全体を録画し、奇数の大きさは自動で偶数にします）。
  `-s 1418x1082` のように画面より小さい大きさを指定すると、**左上のその範囲だけ**が録画され、右側の RViz が入りません。
  画面の大きさは `xdpyinfo | grep dimensions` で確かめられます。
- できるファイル:
  - `~/kumawatch_patrol_output/<日時>_utility/`（結果と `mission_events.csv`・`trajectory.csv`）
  - `~/kumawatch_patrol_output/videos/`（`*_camera.mp4`、`*_detection.mp4`、`*_chase.mp4`、`screen_*.mp4`、それぞれの `.json`）

比較用に、同じ条件で固定ルートも 1 回走らせます（録画は不要）。

```bash
ros2 launch kumawatch_mission kumawatch_patrol.launch.py robot_model:=happy_mini \
    on_bear:=retreat strategy:=fixed gui:=false
```

## 3. 編集（コマンドだけで 1 本にする）

リポジトリのルートで実行します（Python の Pillow・PyYAML と ffmpeg を使います）。

```bash
cd ~/airobot_ws/src/KumaRobot
AI=~/kumawatch_patrol_output/<日時>_utility
FIXED=~/kumawatch_patrol_output/<日時>_fixed
V=~/kumawatch_patrol_output/videos

# (1) 説明の画面(タイトル・背景・AI の判断・構成・結果・比較・まとめ)
python3 scripts/video/make_cards.py --out video_work/cards \
    --summary $AI/patrol_summary.json --compare $FIXED/patrol_summary.json

# (2) 固定ルートと AI ルートの走行軌跡のアニメーション
python3 scripts/video/route_animation.py --left $FIXED --right $AI --out video_work/compare_routes.mp4

# (2b) RViz の代わりの地図の映像(画面録画に RViz が映っていないとき。--map で使う)
python3 scripts/video/live_map.py --run $AI --out video_work/live_map.mp4

# (3) 画面録画の中の RViz の位置を確かめる(30 秒目の画像を書き出す)
python3 scripts/video/edl_from_run.py --run $AI --screen $V/screen_<日時>.mp4 \
    --out video_work/final.yaml --frame-at 30
#     → video_work/final_frame.png を見て、RViz と Gazebo の範囲(x,y,幅,高さ)を決める

# (4) 編集リストを自動で作る
python3 scripts/video/edl_from_run.py --run $AI --screen $V/screen_<日時>.mp4 \
    --camera $V/<日時>_detection.mp4 --chase $V/<日時>_chase.mp4 \
    --gazebo-crop 0,60,1150,700 --rviz-crop 1160,60,760,900 \
    --cards video_work/cards --compare-anim video_work/compare_routes.mp4 --out video_work/final.yaml

# (5) 動画にする(場面を 1 つだけ確かめるときは --only 番号)
python3 scripts/video/compose.py video_work/final.yaml
```

RViz が録画に入っていないときは、(4) の `--rviz-crop` の代わりに `--map video_work/live_map.mp4` を付けます。
`video_work/final.yaml` は普通のテキストなので、字幕の文や秒数を直してから (5) をやり直せます。
字幕のスタイル（`mission` 上部の帯、`lower` 下の字幕、`alert` 熊の警告、`badge` 倍速の札、`label` 小さな札）は
`scripts/video/captions.py` にあります。

## 4. 推奨の構成（約 2 分 30 秒〜3 分）

| 時間 | 内容 | 素材 |
|---|---|---|
| 0:00〜0:14 | タイトルと社会的背景 | `01_title`・`02_background` |
| 0:14〜0:26 | AI のリスク予測と、選んだ 3 地点 | `03_ai_risk` |
| 0:26〜0:38 | ROS 2 のシステム構成、起動した画面（数秒） | `04_system`・画面録画 |
| 0:38〜1:30 | 自律巡回（移動は倍速）。左に追従カメラ、右に RViz。上に Mission i/3 | 追従映像＋画面録画 |
| 1:30〜1:55 | 熊の検知：カメラ映像（YOLO の枠）、BEAR DETECTED、停止、効果音 | 検出映像＋RViz |
| 1:55〜2:10 | 安全地点への退避（RViz の SAFE POINT） | 追従映像＋RViz |
| 2:10〜2:35 | 固定ルートと AI ルートの比較（軌跡のアニメーションと表） | `route_animation`・`08_compare` |
| 2:35〜2:50 | Mission Completed とまとめ | `09_result`・`10_closing` |

## 5. 撮り直さずに角度を変える（記録の再生）

`scripts/video/replay_run.py` は、記録した巡回（`trajectory.csv`・`mission_events.csv`）を Gazebo で再生します。
見た目だけのロボットが記録どおりに動き、前向きカメラ・TF・ミッションの状況も配信するので、
追従カメラの角度（`-p mode:=chase / far / top`）を変えて何度でも撮れます。
Nav2 がその場で走っているわけではないので、動画に使う場合は「走行記録の再生」と明記してください。
このリポジトリでの試し撮り: [docs/videos/KumaWatch_Patrol_preview_new_views.mp4](videos/KumaWatch_Patrol_preview_new_views.mp4)
（Nav2 の走行記録を再生し、追従カメラ・HUD・YOLOv8 の検出・退避を撮ったもの。ロボットは見た目だけのモデル）。

```bash
# Gazebo(ワールドのみ)を起動した状態で
python3 scripts/video/replay_run.py --run ~/kumawatch_patrol_output/<日時>_utility
ros2 run kumawatch_mission chase_camera --ros-args -p robot_name:=replay_robot -p mode:=far
ros2 run kumawatch_ai bear_detector --ros-args -p image_topic:=/replay_cam/image_raw \
    -p camera_info_topic:=/replay_cam/camera_info
```

## 6. 注意

- 数値（リスク・熊の数・距離・時間）は、すべて結果ファイルから自動で入ります。手で書き換えないでください。
- 比較は「この条件での 1 回（または数回）の走行結果」です。字幕にもそう書いてあります。
- 追従カメラと HUD の見た目は、CPU だけの環境では Gazebo が重くなることがあります。重いときは
  `chase_camera` の `-p width:=960 -p height:=540 -p fps:=10` で軽くできます。
