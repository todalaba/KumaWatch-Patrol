// 提出レポートの Word 版を作る(report.html と同じ内容)。
//   node docs/report/build_docx.js   → docs/report/KumaWatch_Patrol_report.docx
// 必要: npm の docx パッケージ(npm install docx)。図は docs/report/figures/ を使う。
const fs = require('fs');
const path = require('path');
const {
  Document, Packer, Paragraph, TextRun, ImageRun, Table, TableRow, TableCell, WidthType,
  AlignmentType, HeadingLevel, BorderStyle, ShadingType, LevelFormat, Footer, PageNumber,
  TableLayoutType, LineRuleType,
} = require('docx');

const FIG = path.join(__dirname, 'figures');
const OUT = path.join(__dirname, 'KumaWatch_Patrol_report.docx');

const SERIF = { ascii: 'Times New Roman', hAnsi: 'Times New Roman', eastAsia: '游明朝' };
const SANS = { ascii: 'Arial', hAnsi: 'Arial', eastAsia: '游ゴシック' };
const MONO = { ascii: 'Consolas', hAnsi: 'Consolas', eastAsia: '游ゴシック' };
const ACCENT = '1F5F8B';
const AI = 'B3541E';
const MUTED = '5B6570';
const HIT = 'FBEEE6';
const RULE = 'CFD6DD';

// A4、余白 17mm。本文幅 = 210 - 34 = 176mm = 9978 DXA
const CONTENT_W = 9978;

// ---- テキストの小さな記法 -------------------------------------------------
// "**太字**"、"[[AI 強調]]"、"`コード`"、"_{下付き}"、"*斜体*" を TextRun に変換する
function runs(text, base = {}) {
  const out = [];
  const re = /(\*\*[^*]+\*\*|\[\[[^\]]+\]\]|`[^`]+`|_\{[^}]+\}|\*[^*]+\*)/g;
  let last = 0;
  let m;
  while ((m = re.exec(text)) !== null) {
    if (m.index > last) out.push(new TextRun({ text: text.slice(last, m.index), ...base }));
    const t = m[0];
    if (t.startsWith('**')) out.push(new TextRun({ text: t.slice(2, -2), bold: true, ...base }));
    else if (t.startsWith('[[')) out.push(new TextRun({ text: t.slice(2, -2), bold: true, color: AI, ...base }));
    else if (t.startsWith('`')) out.push(new TextRun({ text: t.slice(1, -1), font: MONO, size: 17, ...base }));
    else if (t.startsWith('_{')) out.push(new TextRun({ text: t.slice(2, -1), subScript: true, italics: true, ...base }));
    else out.push(new TextRun({ text: t.slice(1, -1), italics: true, ...base }));
    last = m.index + t.length;
  }
  if (last < text.length) out.push(new TextRun({ text: text.slice(last), ...base }));
  return out;
}

const P = (text, opts = {}) => new Paragraph({
  children: runs(text, opts.run || {}),
  alignment: opts.align || AlignmentType.JUSTIFIED,
  spacing: { after: opts.after ?? 100, line: opts.line ?? 330 },
  ...(opts.style ? { style: opts.style } : {}),
});

const H2 = (num, text) => new Paragraph({
  heading: HeadingLevel.HEADING_1,
  children: [new TextRun({ text: `${num}　`, color: ACCENT }), new TextRun(text)],
  border: { bottom: { style: BorderStyle.SINGLE, size: 6, color: RULE, space: 2 } },
  spacing: { before: 260, after: 100 },
  keepNext: true,
});

const H3 = (text) => new Paragraph({
  heading: HeadingLevel.HEADING_2, children: [new TextRun(text)],
  spacing: { before: 160, after: 60 }, keepNext: true,
});

const bullet = (text) => new Paragraph({
  numbering: { reference: 'bullets', level: 0 }, children: runs(text),
  spacing: { after: 50, line: 320 }, alignment: AlignmentType.JUSTIFIED,
});
const numbered = (text) => new Paragraph({
  numbering: { reference: 'numbers', level: 0 }, children: runs(text),
  spacing: { after: 40, line: 320 },
});

const caption = (label, text) => new Paragraph({
  children: [new TextRun({ text: label + '　', bold: true, font: SANS, size: 16 }),
    ...runs(text, { font: SANS, size: 16, color: MUTED })],
  spacing: { before: 40, after: 160, line: 280 },
});

function figure(file, widthPx, ratio) {
  const data = fs.readFileSync(path.join(FIG, file));
  const type = file.endsWith('.png') ? 'png' : 'jpg';
  return new Paragraph({
    alignment: AlignmentType.CENTER, keepNext: true,
    spacing: { before: 80, after: 20, line: 240, lineRule: LineRuleType.AUTO },
    children: [new ImageRun({ type, data, transformation: { width: widthPx, height: Math.round(widthPx * ratio) } })],
  });
}

// ---- 表 -----------------------------------------------------------------------
const border = { style: BorderStyle.SINGLE, size: 4, color: RULE };
const noBorder = { style: BorderStyle.NONE, size: 0, color: 'FFFFFF' };

function table(header, rows, widths, opts = {}) {
  const total = widths.reduce((a, b) => a + b, 0);
  const right = opts.right || [];
  const cell = (text, i, isHead, shade) => new TableCell({
    width: { size: widths[i], type: WidthType.DXA },
    margins: { top: 50, bottom: 50, left: 90, right: 90 },
    shading: shade ? { fill: shade, type: ShadingType.CLEAR, color: 'auto' } : undefined,
    borders: {
      top: noBorder, left: noBorder, right: noBorder,
      bottom: isHead ? { style: BorderStyle.SINGLE, size: 10, color: '1D232A' } : border,
    },
    children: [new Paragraph({
      alignment: right.includes(i) ? AlignmentType.RIGHT : AlignmentType.LEFT,
      spacing: { after: 0, line: 260 },
      children: runs(String(text), { font: SANS, size: 16, bold: isHead || undefined }),
    })],
  });
  return new Table({
    width: { size: total, type: WidthType.DXA }, columnWidths: widths,
    layout: TableLayoutType.FIXED,
    rows: [
      new TableRow({ tableHeader: true, children: header.map((h, i) => cell(h, i, true)) }),
      ...rows.map((r) => new TableRow({
        cantSplit: true,
        children: r.cells.map((c, i) => cell(c, i, false, r.hit ? HIT : undefined)),
      })),
    ],
  });
}
const R = (cells, hit = false) => ({ cells, hit });

// ---- 表紙部分(氏名・動画 URL は入力欄) ---------------------------------------
function fieldCell(label, value, w, placeholder) {
  return [
    new TableCell({
      width: { size: 1300, type: WidthType.DXA },
      margins: { top: 60, bottom: 60, left: 90, right: 90 },
      shading: { fill: 'F3F6F8', type: ShadingType.CLEAR, color: 'auto' },
      borders: { top: border, bottom: border, left: border, right: border },
      children: [new Paragraph({ spacing: { after: 0 },
        children: [new TextRun({ text: label, font: SANS, size: 17, bold: true, color: MUTED })] })],
    }),
    new TableCell({
      width: { size: w, type: WidthType.DXA },
      margins: { top: 60, bottom: 60, left: 120, right: 90 },
      borders: { top: border, bottom: border, left: border, right: border },
      children: [new Paragraph({ spacing: { after: 0 },
        children: [new TextRun({ text: value, font: SANS, size: 18,
          color: placeholder ? '9AA3AC' : '1D232A', italics: placeholder || undefined })] })],
    }),
  ];
}

const metaTable = new Table({
  width: { size: CONTENT_W, type: WidthType.DXA },
  columnWidths: [1300, 3689, 1300, 3689],
  layout: TableLayoutType.FIXED,
  rows: [
    new TableRow({ children: [...fieldCell('提出者', '（ここに氏名を入力）', 3689, true),
      ...fieldCell('提出日', '2026年11月', 3689, false)] }),
    new TableRow({ children: [...fieldCell('動画 URL', '（ここに動画の URL を入力）', 3689, true),
      ...fieldCell('コード', 'github.com/Global-jogasaki/kumawatch-patrol', 3689, false)] }),
  ],
});

// ---- 本文 ---------------------------------------------------------------------
const body = [
  new Paragraph({ spacing: { after: 20 },
    children: [new TextRun({ text: 'Physical AI 応用編 2026　最終課題 提出レポート', font: SANS, size: 16, color: MUTED })] }),
  new Paragraph({ heading: HeadingLevel.TITLE, spacing: { after: 20 },
    children: [new TextRun('KumaWatch Patrol：AI による熊出没リスク巡回ロボット')] }),
  new Paragraph({ spacing: { after: 140 },
    border: { bottom: { style: BorderStyle.SINGLE, size: 12, color: '1D232A', space: 6 } },
    children: [new TextRun({ text: 'Risk-Aware Robotic Patrol for Bear Encounters', font: SANS, size: 20, color: MUTED })] }),
  metaTable,
  new Paragraph({
    spacing: { before: 200, after: 120, line: 320 }, alignment: AlignmentType.JUSTIFIED,
    shading: { fill: 'F3F6F8', type: ShadingType.CLEAR, color: 'auto' },
    border: { left: { style: BorderStyle.SINGLE, size: 18, color: ACCENT, space: 8 } },
    indent: { left: 160, right: 80 },
    children: runs('熊の出没地域では、限られた人員ですべての地点を巡回できない。本作品は、熊出没予測の研究「KumaWatch」の予測 AI（[[TTM]]）が選んだ地点を、ROS 2 と Gazebo 上の移動ロボットが Navigation2 で自律巡回し、カメラ画像の AI 判定（[[YOLOv8]]）で熊を見つけると安全な距離で停止・記録するシステムである。AI を「予測」と「現場確認」の 2 か所で使い、予測結果をロボットの行動につなげた。同じ 3 地点の巡回で、固定ルートの熊発見数 1 に対し、AI ルートは 2 だった。', { size: 18 }),
  }),

  H2('1', '課題の背景'),
  P('近年、熊の人里への出没が増え、自治体は通学路やごみ集積所などの見回りに追われている。しかし人員には限りがあり、「明日はどこを見に行くべきか」を限られた数だけ選ぶ必要がある。筆者らの研究 KumaWatch では、山形県・秋田県の公開出没データを 10 km グリッドに集計し、翌日の出没リスクが高いセルを上位 *K* 個選ぶ問題として、複数の予測手法を比較してきた。一方、予測はあくまで地図上の数値であり、実際に現場を確認し、熊がいれば安全に行動する部分は人に委ねられている。本作品では、この「予測から現場確認まで」をロボットで具体化する。'),

  H2('2', '目的'),
  numbered('AI の出没リスク予測に基づいて、巡回可能な数（*K*＝3）の地点と巡回順を自動で決める。'),
  numbered('Navigation2 で候補地点を自律巡回し、カメラ画像を AI で判定して熊を見つける。'),
  numbered('熊を見つけたら追跡せずに停止し、位置・時刻・信頼度を記録する（安全側の行動）。'),
  numbered('固定ルートの巡回と同じ条件で比較し、AI で地点を選ぶ効果を示す。'),

  H2('3', 'システム構成'),
  figure('system_diagram.png', 640, 250 / 760),
  caption('図1　システム構成。', '橙の枠が AI を使う部分。ノード間は ROS 2 のトピック（リスク・熊検知は独自メッセージ）と Nav2 の `NavigateToPose` アクションでつながる。'),
  P('ソフトウェアは 5 つの ROS 2 パッケージで構成した（表1）。ワールドと Nav2 用の地図は、同じ定義ファイル（`world_layout.yaml`）から自動生成するので、SLAM で地図を作らなくても座標が一致する。熊とシカは地図に載せず、ロボットにとって未知の障害物として LiDAR とカメラで見つける。'),
  table(['ノード', '役割'], [
    R(['`risk_predictor`', 'TTM の予測を 5 地点のリスクとして配信し、RViz に棒グラフで表示']),
    R(['`mission_planner`', '地点選択・巡回順の決定、Nav2 への移動指示、熊検知時の停止・記録、結果表示（状態機械）']),
    R(['`bear_detector`', 'カメラ画像を YOLOv8 で判定し、LiDAR の距離から熊の地図上の位置を推定']),
    R(['`image_recorder`', 'カメラ映像と検出枠入りの映像を状態表示付きで録画、熊検知時の静止画を保存']),
    R(['`bear_proximity_monitor`', '熊の既知座標との距離で検知する簡易版（比較・動作確認用）']),
  ], [2600, 7378]),
  caption('表1　主なノード。', '状態機械は RISK_ASSESSMENT → ROUTE_PLANNING → PATROL ⇄ OBSERVE →（EMERGENCY_STOP）→ RETURN_TO_BASE → FINISHED と遷移する。'),

  H2('4', '実装内容'),
  H3('4.1　AI 1：熊出没リスクの予測（KumaWatch の TTM）'),
  P('リスク予測には、KumaWatch が公開している [[IBM Granite TinyTimeMixer（TTM 1536-96-R2）]]の予測を使った。TTM は大量の時系列で事前学習された時系列予測の基盤モデルで、KumaWatch では山形県 144 セルの日別出没件数の履歴から、学習に使っていない 2025 年の各日の出没をゼロショットで予測している。作品の町は架空だが、5 地点のリスクには実在セルの予測値を割り当てた（表2）。割り当ては次の規則による：2025 年 6 月 7 日に実際に出没があった 2 セルを P2・P4 とし、Gazebo でもそこに熊を置く。出没がなかったセルからは、TTM 順位が最も高いもの（P3、AI の空振り）、40 位付近（P5）、100 位付近（P1）を選んだ。リスクは県内 144 セル中の順位を百分位にした値である。'),
  table(['地点', '環境', 'セル', 'TTM 順位', 'リスク', '実際の出没', 'Gazebo の熊'], [
    R(['P4', 'ごみ集積所', '5_1', '1 / 144', '1.000', 'あり', 'あり'], true),
    R(['P3', '通学路', '4_1', '2 / 144', '0.993', 'なし（空振り）', 'なし']),
    R(['P2', '林縁', '1_10', '4 / 144', '0.979', 'あり', 'あり'], true),
    R(['P5', '河川沿い', '6_0', '40 / 144', '0.727', 'なし', 'なし（シカ）']),
    R(['P1', '住宅地', '1_0', '93 / 144', '0.357', 'なし', 'なし']),
  ], [800, 1500, 1000, 1350, 1100, 2228, 2000], { right: [3, 4] }),
  caption('表2', '5 地点に割り当てた TTM の予測（2025 年 6 月 7 日、山形県 10 km グリッド）。'),

  H3('4.2　巡回地点と巡回順の決定'),
  P('リスクだけでなく、社会的重要度と移動コストを考えた効用で地点を選ぶ。'),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { before: 60, after: 100 },
    children: runs('*U*_{i} ＝ *w*_{i} *p*_{i} − *λ* *d*_{i}', { size: 22 }) }),
  P('*p*_{i} はリスク、*w*_{i} は重要度（通学路 1.2、住宅地 1.1、他 1.0）、*d*_{i} は詰所からの距離 [m]、*λ*＝0.02 である。*U*_{i} の上位 3 地点を選び、詰所から回って戻る総移動距離が最小になる順を全順列で求める。今回は P2（林縁）→ P4（ごみ集積所）→ P3（通学路）となった。比較用に、リスクだけで選ぶ方式と固定ルートも実装した。'),

  H3('4.3　自律巡回と停止'),
  P('`mission_planner` は Nav2 の `NavigateToPose` アクションで各地点へ移動し、到着後に一定時間周囲を確認する。熊の検知を受け取ると、移動中のゴールをキャンセルしてゼロ速度で停止し、熊の位置・時刻・距離・信頼度を記録する。熊を追跡せず、止まって記録してから次の地点へ向かう。巡回後は詰所へ戻り、結果を端末・RViz・ファイルに出力する。'),

  H3('4.4　AI 2：カメラ画像からの熊判定（YOLOv8）'),
  P('COCO で学習済みの YOLOv8 にも「bear」クラスはあるが、Gazebo の熊（基本図形の組み合わせ）は信号機や人と誤認された（図2 左）。そこで Gazebo の画像で[[追加学習（ファインチューニング）]]した。カメラと熊・シカをランダムな位置に動かしながら 700 枚を撮影し、熊の 3D 位置を画像へ投影して正解の枠を自動で付けた。木や建物で隠れる視点は除き、熊が写らない画像も 3 割含めた。学習済みの YOLOv8n を初期値に追加学習した（図2 右）。'),
  figure('yolo_before_after.jpg', 560, 480 / 1296),
  caption('図2　追加学習の前後。', '左：COCO 学習済みの YOLOv8s は Gazebo の熊を「traffic light」と誤認する。右：追加学習後の YOLOv8n は「bear」（信頼度 0.97）と判定する。'),
  P('`bear_detector` は、検出枠の横位置からカメラに対する熊の方位を、その方位の LiDAR の値から距離を求め、ロボットの自己位置と合わせて熊の地図上の位置を推定する。LiDAR が手前の木を拾った場合は、枠の高さからの推定に切り替える。誤検出で止まらないよう、3 フレーム連続で検出し、かつ 4 m 以内に近づいたときに停止を指示する。'),

  H2('5', '工夫した点'),
  bullet('**研究の予測をそのまま使う。**架空の点数ではなく、KumaWatch の TTM の実際の予測値を使い、実際に出没があったセルに熊を置いた。AI の空振り（P3）もあえて含め、予測が外れる場合も見えるようにした。'),
  bullet('**危険度だけでなく、重要度と移動距離も考える。**通学路の重みと移動コストを入れた効用で地点を選び、巡回順は総移動距離が最小になるように決めた。'),
  bullet('**ラベル付けを自動化した追加学習。**シミュレータでは熊の位置が分かるので、画像への投影で正解の枠を自動で作れる。手作業なしで 700 枚の学習データを作り、既存モデルが認識できない対象に対応させた。'),
  bullet('**安全側の行動。**熊を見つけても近づかず、4 m 以内で停止して記録する。連続検出で確定させ、1 フレームの誤検出では止まらないようにした。'),
  bullet('**ワールドと地図を 1 つの定義から生成。**配置を変えても地図が自動で一致する。開けた場所で自己位置推定がずれた問題には、電柱やベンチを目印として加えて対処した。'),

  H2('6', '結果'),
  H3('6.1　固定巡回との比較'),
  figure('overview.png', 560, 726 / 1430),
  caption('図3　巡回の比較（俯瞰図）。', '左：AI 巡回（P2→P4→P3）、右：固定巡回（P1→P2→P3）。円の色はリスク、白枠は選ばれた地点、＋は熊、×はシカ、赤丸は熊の発見位置。'),
  table(['指標', '固定巡回（P1→P2→P3）', 'AI 巡回（P2→P4→P3）'], [
    R(['巡回地点数', '3', '3']),
    R(['熊の発見数', '1', '**2**'], true),
    R(['移動距離（Nav2 による走行）', '28.8 m', '32.5 m']),
    R(['所要時間（シミュレーション時間）', '152.6 s', '153.8 s']),
  ], [3978, 3000, 3000], { right: [1, 2] }),
  caption('表3', '同じ 3 地点・同じ熊の配置での比較（詰所への帰還を含む）。本物の Nav2（AMCL・NavFn・DWB）と本作品の地図・パラメータを、差動二輪とレイキャスト LiDAR の簡易シミュレータ上で動かした値で、熊は既知座標で判定した。'),
  P('同じ 3 地点を回る条件で、AI 巡回は熊を 2 頭、固定巡回は 1 頭見つけた。移動距離は AI 巡回の方が 3.7 m 長いが、所要時間はほぼ同じだった（AI 巡回は熊の手前で止まって次へ向かうため）。ただし、これは 1 日・1 配置のシミュレーションの結果であり、「AI が一般に優れている」ことを示すものではない。実際、1 年を通した評価では、山形県で TTM は単純な静的予測（過去の平均）に有意に劣っている。'),

  H3('6.2　熊判定の精度'),
  table(['Precision（検証 105 枚）', 'Recall（検証 105 枚）', 'mAP@0.5', '巡回中の視点での判定'], [
    R(['**0.94**', '**0.99**', '**0.985**', '**11 / 11**']),
  ], [2494, 2494, 2495, 2495]),
  figure('detection_examples.jpg', 580, 488 / 976),
  caption('図4　巡回中にロボットが見る視点での判定例。', '上段：P2 への接近（信頼度 0.90）、P4 への接近（0.88）、カメラの高い Happy Mini の視点（0.94）。下段：P3 から見た画面の端の熊（5.4 m 先で停止距離の外）、シカ（熊と誤認しない）、熊のいない林縁。'),
  P('巡回中の 11 視点（P2・P4 への接近と到着、熊のいない P1・P3・P5、シカ、林縁など）では、熊が写る視点はすべて検出し（信頼度 0.88〜0.98）、シカ・木・建物を熊と誤認しなかった。ROS 2 ノードとしても、Gazebo のカメラ映像から熊を検出し、停止指示のメッセージを出すことを確認した。'),

  H2('7', '限界と今後の課題'),
  bullet('**シミュレーションの範囲。**Gazebo 内の模擬環境であり、実環境での自己位置推定・通信・天候の影響は検証していない。'),
  bullet('**熊判定は Gazebo の熊でのみ学習・評価した。**基本図形の熊と実物の熊は見た目が大きく違うため、実物の画像での精度は不明である。実運用には実画像での学習と評価が必要である。'),
  bullet('**予測と地点の対応は説明用。**TTM の予測は 10 km 四方のセル単位で、町の中の 1 地点と同じ意味ではない。日付もシナリオのために選んだ 1 日である。'),
  bullet('**今後。**熊検知後の安全地点への退避と発見地点の地図表示、GLM-Logit や静的予測との巡回結果の比較、熊の配置を変えた複数回の試行による発見率の評価を行う。'),

  H2('付録', '動作環境・再現方法・使用した素材'),
  P('動作環境：Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11、Navigation2、TurtleBot3（waffle / Happy Mini）、Ultralytics YOLOv8。講義の演習環境（Docker イメージ `airobotbook/ros2-desktop-ai-robot-book-humble`）で動作する。起動：`ros2 launch kumawatch_mission kumawatch_patrol.launch.py`（固定巡回は `strategy:=fixed`）。詳しい手順はリポジトリの `README.md` と `docs/lecture_env.md` を参照。学習データと学習プログラムは `training/` にまとめた。', { run: { size: 17 } }),
  bullet('KumaWatch（熊出没予測の研究。TTM の予測値。データは CC BY 4.0、出典：山形県の公開野生動物出没データ）（非公開の研究リポジトリ）'),
  bullet('IBM Granite TinyTimeMixer（Apache-2.0）、Ultralytics YOLOv8（AGPL-3.0）'),
  bullet('ROS 2 / Navigation2 / Gazebo（Apache-2.0）、TurtleBot3 シミュレーション（ROBOTIS, Apache-2.0）、Happy Mini（AI ロボット入門）'),
  bullet('熊・シカ・建物・樹木のモデルは基本図形から自作（外部の 3D モデルは不使用）'),
];

const doc = new Document({
  creator: 'KumaWatch Patrol',
  title: 'KumaWatch Patrol 提出レポート',
  styles: {
    default: { document: { run: { font: SERIF, size: 19 },
      paragraph: { spacing: { line: 330, lineRule: LineRuleType.AUTO } } } },
    paragraphStyles: [
      { id: 'Title', name: 'Title', basedOn: 'Normal', next: 'Normal', quickFormat: true,
        run: { font: SANS, size: 34, bold: true, color: '1D232A' } },
      { id: 'Heading1', name: 'Heading 1', basedOn: 'Normal', next: 'Normal', quickFormat: true,
        run: { font: SANS, size: 23, bold: true, color: '1D232A' },
        paragraph: { spacing: { before: 260, after: 100 }, outlineLevel: 0 } },
      { id: 'Heading2', name: 'Heading 2', basedOn: 'Normal', next: 'Normal', quickFormat: true,
        run: { font: SANS, size: 20, bold: true, color: '1D232A' },
        paragraph: { spacing: { before: 160, after: 60 }, outlineLevel: 1 } },
    ],
  },
  numbering: {
    config: [
      { reference: 'bullets', levels: [{ level: 0, format: LevelFormat.BULLET, text: '•', alignment: AlignmentType.LEFT,
        style: { paragraph: { indent: { left: 400, hanging: 260 } } } }] },
      { reference: 'numbers', levels: [{ level: 0, format: LevelFormat.DECIMAL, text: '%1.', alignment: AlignmentType.LEFT,
        style: { paragraph: { indent: { left: 400, hanging: 300 } } } }] },
    ],
  },
  sections: [{
    properties: {
      page: { size: { width: 11906, height: 16838 },
        margin: { top: 964, bottom: 907, left: 964, right: 964, footer: 454 } },
    },
    footers: {
      default: new Footer({ children: [new Paragraph({ alignment: AlignmentType.CENTER,
        children: [new TextRun({ children: [PageNumber.CURRENT], font: SANS, size: 16, color: MUTED })] })] }),
    },
    children: body,
  }],
});

Packer.toBuffer(doc).then((buf) => {
  fs.writeFileSync(OUT, buf);
  console.log('→', OUT);
});
