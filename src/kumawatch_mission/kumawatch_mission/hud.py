"""動画に重ねる HUD(ROS 非依存。image_recorder と動画編集スクリプトで共用).

- 上部の半透明の帯: 「Mission 1/3: P2 Forest edge」「AI risk 0.98 | Navigating | bears 0 | 12.3 m」
- 熊を検知した直後: 画面の赤枠と、下部中央の大きなパネル
  (BEAR DETECTED / Confidence・Distance / Location / Action)
"""

import cv2

from kumawatch_mission.mission_text import bear_panel_lines, status_lines


def _text(img, text, org, scale, color, thick=2):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 3,
                cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def draw_hud(img, status, bear=None):
    """img(BGR)に HUD を描いて返す. status: mission_status の dict, bear: last_bear の dict."""
    h, w = img.shape[:2]
    k = max(0.6, w / 1000.0)
    line1, line2 = status_lines(status or {})
    bar_h = int(78 * k)
    band = img.copy()
    cv2.rectangle(band, (0, 0), (w, bar_h), (0, 0, 0), -1)
    cv2.addWeighted(band, 0.55, img, 0.45, 0, img)
    _text(img, line1, (int(14 * k), int(32 * k)), 0.95 * k, (255, 255, 255), 2)
    _text(img, line2, (int(14 * k), int(64 * k)), 0.72 * k, (170, 255, 170), 2)
    if bear:
        cv2.rectangle(img, (0, 0), (w - 1, h - 1), (0, 0, 230), max(4, int(10 * k)))
        lines = bear_panel_lines(bear)
        pw, ph = min(w - 20, int(560 * k)), int(170 * k)
        x0, y0 = (w - pw) // 2, h - ph - int(24 * k)
        panel = img.copy()
        cv2.rectangle(panel, (x0, y0), (x0 + pw, y0 + ph), (0, 0, 170), -1)
        cv2.addWeighted(panel, 0.85, img, 0.15, 0, img)
        cv2.rectangle(img, (x0, y0), (x0 + pw, y0 + ph), (255, 255, 255), 2)
        _text(img, lines[0], (x0 + int(18 * k), y0 + int(46 * k)), 1.35 * k, (255, 255, 255), 3)
        for i, t in enumerate(lines[1:]):
            _text(img, t, (x0 + int(18 * k), y0 + int((84 + 32 * i) * k)), 0.72 * k,
                  (255, 255, 255), 2)
    return img
