# 講義の演習環境（AIロボット入門 Docker イメージ）で動かす

Physical AI 応用編の講義で使っている Docker イメージ
`airobotbook/ros2-desktop-ai-robot-book-humble`（ブラウザから `localhost:6080` で使う Ubuntu デスクトップ）で
KumaWatch Patrol を動かす手順です。

## このイメージで動く理由

KumaWatch Patrol に必要なものは、講義のイメージにすべて入っています
（[イメージの構築スクリプト](https://github.com/AI-Robot-Book-Humble/docker-ros2-desktop-ai-robot-book-humble/blob/main/ai-robot-book.sh) で確認済み）。

| 必要なもの | 講義イメージでの状態 |
|---|---|
| ROS 2 Humble（desktop） | インストール済み |
| Gazebo Classic 11 + `gazebo_ros_pkgs` | インストール済み |
| Navigation2 / `nav2_bringup` | インストール済み（書籍 第4章用） |
| `turtlebot3_gazebo`（TurtleBot3 のモデルと launch） | `~/airobot_ws/src/turtlebot3_happy_mini` にビルド済み（waffle / waffle_pi / burger / happy_mini） |
| OpenCV・numpy・PyYAML | インストール済み |
| Ultralytics（YOLOv8。熊の画像判定に使用） | インストール済み（書籍 第5章用） |

注意点が 1 つあります。イメージの `~/.bashrc` は `TURTLEBOT3_MODEL=happy_mini` を設定しています。
KumaWatch Patrol の launch は `robot_model` 引数（既定 `waffle`）でモデルを上書きするので、そのまま起動して構いません。

> 動作確認の範囲: 講義イメージそのものでの実行はまだ確認していません。確認済みなのは次の 3 点です。
> (1) 同じ ROS 2 Humble 上でのビルドと Gazebo なしの巡回（CI）。
> (2) 本物の Nav2 での巡回（簡易 LiDAR シミュレータ使用）。
> (3) 講義イメージの `turtlebot3_gazebo` と、launch の引数・TF フレーム名・`/scan` が一致すること。
> 初回は下の「3. まず Gazebo なしで確認」から進めてください。

---

## 1. コンテナを起動する（ホスト PC）

講義の起動コマンドに、次の 3 つを足すことをおすすめします。

- `--name kumawatch`: コンテナに名前を付けます。ビルドした成果を次回も使えます。
- `--shm-size=1g`: Gazebo と RViz を同時に動かすための共有メモリです。
- `-e RESOLUTION=1920x1080`: 動画撮影用に画面をフル HD にします。

```bash
docker pull airobotbook/ros2-desktop-ai-robot-book-humble:latest
docker run -p 6080:80 --shm-size=1g -e RESOLUTION=1920x1080 --name kumawatch \
  airobotbook/ros2-desktop-ai-robot-book-humble:latest
```

ブラウザで <http://localhost:6080> を開き、「接続」を押します（VNC のパスワードは `ubuntu`）。

2 回目以降は、新しく `docker run` せずに同じコンテナを起動します
（`docker run` をやり直すと、ビルドしたものが消えた新しいコンテナになります）。

```bash
docker start kumawatch        # その後ブラウザで localhost:6080
```

NVIDIA GPU があるホストでは、`docker run` に `--gpus all` を付けると Gazebo の描画が速くなります
（[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/) が必要）。

## 2. KumaWatch Patrol を入れてビルドする（コンテナ内の端末）

デスクトップ上で端末（Terminator）を開きます。

### リポジトリを取得する

このリポジトリは **公開（Public）** なので、認証なしでクローンできます。
フォルダ名は、以下の手順と合わせるため `KumaRobot` にします。

```bash
cd ~/airobot_ws/src
git clone https://github.com/todalaba/KumaWatch-Patrol.git KumaRobot
```

ホスト PC でクローンしてコンテナにコピーする場合は、ホスト PC の端末で次を実行します。

```bash
git clone https://github.com/todalaba/KumaWatch-Patrol.git KumaRobot
docker cp KumaRobot kumawatch:/home/ubuntu/airobot_ws/src/
docker exec -u root kumawatch chown -R ubuntu:ubuntu /home/ubuntu/airobot_ws/src/KumaRobot
```

> すでにクローン済みの場合は、最新版に更新してからビルドします:
> `cd ~/airobot_ws/src/KumaRobot && git pull`

### ビルドする

```bash
cd ~/airobot_ws
colcon build --packages-up-to kumawatch_mission
source ~/.bashrc
```

- `--packages-up-to` で KumaWatch Patrol の 5 パッケージだけをビルドします（1 分以内）。
- 講義イメージは `--symlink-install` が既定なので、Python を編集しても再ビルドは不要です
  （`package.xml`・`setup.py`・launch・設定ファイルを追加したときだけ再ビルドします）。
- `~/.bashrc` が `~/airobot_ws/install/setup.bash` を読み込むので、新しく開いた端末でもそのまま使えます。
- `rosdep install` は不要です（必要なものはイメージに入っています）。

確認:

```bash
ros2 pkg list | grep kumawatch
# kumawatch_ai, kumawatch_gazebo, kumawatch_interfaces, kumawatch_mission, kumawatch_navigation
```

## 3. まず Gazebo なしで確認する（1 分）

ロボットを直線で動かす簡易シミュレータ（`fake_nav2`）で、AI のリスク推定から結果表示までの流れを確認します。

```bash
ros2 launch kumawatch_mission dry_run.launch.py time_scale:=8.0
```

端末に「巡回計画」の表が出て、熊を 2 回検知して停止し、最後に「KumaWatch Patrol 巡回結果」の表
（熊発見数 2）が出れば成功です。`Ctrl+C` で終了します。

## 4. Gazebo + Nav2 で巡回する

```bash
ros2 launch kumawatch_mission kumawatch_patrol.launch.py
```

Gazebo（俯瞰）と RViz（地図・リスク・巡回順）が開き、次の順に自動で進みます。

1. AI のリスク推定と巡回計画（端末に表が出る）
2. 詰所 → P2（林縁）→ P4（ごみ集積所）→ P3（通学路）の順に巡回
3. 熊に近づくと停止・記録し（端末に `!!! 熊を検知` と表示）、熊から離れた安全地点へ退避してから次の地点へ
4. 詰所に戻り、端末と RViz に結果を表示

よく使う引数:

| 引数 | 例 | 説明 |
|---|---|---|
| `strategy` | `strategy:=fixed` | 固定ルート（P1→P2→P3）で巡回（比較用） |
| `robot_model` | `robot_model:=happy_mini` | 書籍の Happy Mini で巡回（既定は `waffle`） |
| `gui` | `gui:=false` | Gazebo の画面を出さない（軽くなる。RViz で確認） |
| `nav2_delay` | `nav2_delay:=15` | 重い PC で Nav2 の起動を遅らせる（既定 5 秒） |
| `record` | `record:=true` | カメラ映像と、YOLO の検出枠入り映像を MP4 に録画（[docs/recording.md](recording.md)） |
| `detector` | `detector:=proximity` | 熊の判定を既知座標との距離にする（YOLO を使わない。軽い） |
| `detector_rate` | `detector_rate:=2.0` | YOLO の推論頻度 [Hz]（既定 5。重い PC では下げる） |

### 講義環境（GPU なし）で軽く動かすコツ

- noVNC のコンテナは GPU を使わずソフトウェアで描画するため、Gazebo の画面が重くなります。
  カクつくときは `gui:=false` で Gazebo の画面を消し、RViz だけで確認してください
  （シミュレーション自体は動き続けます）。
- すべてのノードがシミュレーション時間で動くので、PC が遅いと巡回全体がゆっくりになるだけで、結果は変わりません。
- waffle のカメラは 1920×1080 で、GPU なしでは描画が重くなります。`robot_model:=happy_mini`（640×480）にすると軽くなります。
  熊検出モデルは両方のカメラの高さ（約 0.2 m と約 1 m）の画像で学習しています。
- YOLO の推論も CPU で行います。重いときは `detector_rate:=2.0` に下げてください。
- もう一度起動する前に、前回の Gazebo が残っていないか確認します: `killall -9 gzserver gzclient`

### 動いているかを別の端末で確認する

```bash
ros2 topic echo /kumawatch/mission_state          # 状態 (PATROL, EMERGENCY_STOP, ...)
ros2 topic echo /kumawatch/bear_detection         # 熊の検知
ros2 run rqt_image_view rqt_image_view /camera/image_raw   # ロボットのカメラ
ros2 run rqt_image_view rqt_image_view /kumawatch/detection_image   # YOLO の検出枠入り
rqt_graph                                         # ノードとトピックのつながり
```

## 5. 結果を取り出す

結果はコンテナ内の `~/kumawatch_patrol_output/<日時>_<strategy>/` に保存されます
（`patrol_summary.json`・`patrol_results.csv`・`bear_events.csv`、録画した動画は `videos/`）。
ホスト PC で次を実行すると、カレントディレクトリにコピーされます。

```bash
docker cp kumawatch:/home/ubuntu/kumawatch_patrol_output ./
```

## 困ったとき

| 症状 | 対処 |
|---|---|
| 起動直後に `Caught exception in launch ... [Errno 2] No such file or directory: ''` が出て全体が止まる | 古い版の不具合です(Gazebo の launch 引数 `params_file` との名前の衝突)。`git pull` で更新し、ビルドし直してください。新しい版では起動の最初に `[KumaWatch Patrol] robot_model=...` と表示されます。表示されないときは古いビルドが動いています(下の「古いビルドが動いているとき」) |
| それでも止まる | 3 つの端末に分けて起動します(下の「一括起動が止まるとき」)。原因を調べるには `ros2 launch -d kumawatch_mission kumawatch_patrol.launch.py 2>&1 \| grep -B 40 "Errno 2"` の出力を送ってください |
| `Package 'kumawatch_mission' not found` | `source ~/.bashrc`（またはビルドし直す） |
| `Package 'turtlebot3_gazebo' not found` | `source ~/airobot_ws/install/setup.bash`。講義イメージでは `turtlebot3_happy_mini` リポジトリに含まれている |
| Gazebo が真っ黒・起動しない | `killall -9 gzserver gzclient` してから再起動。`--shm-size` を 1g 以上にする |
| 「Nav2 と自己位置推定の起動を待っています...」が続く | `nav2_delay:=15` で起動し直す。RViz の「2D Pose Estimate」で詰所（地図の下中央）を指定する |
| ロボットがその場で回り続ける／止まる | 自己位置推定がずれています。RViz の「2D Pose Estimate」でロボットの実際の位置と向きを指定する |
| `ultralytics がありません` と出て bear_detector が止まる | `pip3 install ultralytics`。または `detector:=proximity` で起動する |
| bear_detector が `numpy.core.multiarray failed to import` で止まる | ultralytics が NumPy 2 と新しい OpenCV を `~/.local` に入れ、講義イメージの matplotlib と合わなくなっています。`pip3 uninstall -y opencv-python && pip3 install --user "numpy<2" && pip3 install --user --force-reinstall --no-deps "opencv-contrib-python==4.5.5.64"` |
| `record_screen.sh` の MP4 が 0 バイト／`width not divisible by 2` | 画面の幅か高さが奇数です。最新版に更新する（自動で偶数に切り詰めます）か、`./scripts/record_screen.sh -s 1418x1082` のように偶数の大きさを指定する |
| `/usr/bin/env: 'bash\r': No such file or directory` | Windows で clone して改行が CRLF になっています。`cd ~/kumawatch_ws/src/KumaRobot && git config core.autocrlf false && git rm --cached -r -q . && git reset --hard HEAD` |
| 熊の前を通っても止まらない | `rqt_image_view /kumawatch/detection_image` で枠が出ているか確認。止まる距離は `report_distance`（既定 4 m）以内 |
| 端末の日本語が文字化けする | 講義イメージは日本語環境なので通常は起きません。`export LANG=ja_JP.UTF-8` を試す |

### 古いビルドが動いているとき

```bash
cd ~/airobot_ws/src/KumaRobot && git checkout main && git pull && git log --oneline -1
grep -c OpaqueFunction ~/airobot_ws/install/kumawatch_navigation/share/kumawatch_navigation/launch/navigation.launch.py
# 0 なら古いビルド。消してからビルドし直す
cd ~/airobot_ws
rm -rf build/kumawatch_* install/kumawatch_*
colcon build --symlink-install --packages-up-to kumawatch_mission
source ~/.bashrc
```

### 一括起動が止まるとき(3 つの端末に分けて起動する)

一括起動(`kumawatch_patrol.launch.py`)と同じことを、端末を分けて順に起動します。
どの端末で止まるかで、原因の場所も分かります。

```bash
# 端末1: Gazebo とロボット
ros2 launch kumawatch_gazebo sim.launch.py robot_model:=happy_mini
# 端末2: Gazebo の画面にロボットが出てから。Nav2 と RViz
ros2 launch kumawatch_navigation navigation.launch.py
# 端末3: RViz に地図が出てから。AI のリスク推定・巡回計画・熊検出
ros2 launch kumawatch_mission patrol_nodes.launch.py
```
