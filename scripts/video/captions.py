"""動画に重ねる字幕・パネルを透明 PNG(1920×1080)として描く(Pillow).

compose.py と make_cards.py から使う。日本語は Noto Sans CJK などのフォントで描く
(講義の Docker イメージには日本語フォントが入っている)。

スタイル:
  lower   画面下の半透明の帯に 1〜2 行(説明の字幕)
  mission 画面上の帯: 1 行目「Mission 1/3: P2 Forest edge」、2 行目「AI risk 0.85 | Navigating」
  alert   画面下中央の赤いパネル: BEAR DETECTED / Confidence / Location / Action
  badge   右上の小さな札(例: 2× Speed)
  label   任意の位置の小さな札(例: Gazebo / RViz)
  title   画面中央の大きな見出し
"""

import os
import subprocess

from PIL import Image, ImageDraw, ImageFont

W, H = 1920, 1080
FONT_CANDIDATES = {
    'bold': ['Noto Sans CJK JP:bold', 'Noto Sans JP:bold', 'IPAexGothic', 'TakaoPGothic',
             'DejaVu Sans:bold'],
    'regular': ['Noto Sans CJK JP', 'Noto Sans JP', 'IPAexGothic', 'TakaoPGothic', 'DejaVu Sans'],
}
_font_cache = {}


def font_path(weight='regular'):
    if weight in _font_cache:
        return _font_cache[weight]
    for name in FONT_CANDIDATES[weight]:
        try:
            path = subprocess.run(['fc-match', '-f', '%{file}', name], capture_output=True,
                                  text=True, check=True).stdout.strip()
        except (OSError, subprocess.CalledProcessError):
            continue
        if path and os.path.exists(path):
            _font_cache[weight] = path
            return path
    raise RuntimeError('日本語フォントが見つかりません(fonts-noto-cjk を入れてください)')


def font(size, weight='regular'):
    return ImageFont.truetype(font_path(weight), size)


def canvas():
    return Image.new('RGBA', (W, H), (0, 0, 0, 0))


def text_size(draw, text, f):
    box = draw.textbbox((0, 0), text, font=f)
    return box[2] - box[0], box[3] - box[1]


def lower(lines, y_bottom=1040, size=46, sub_size=34):
    """画面下の字幕. lines: [主文, (補足)]."""
    img = canvas()
    d = ImageDraw.Draw(img)
    fonts = [font(size, 'bold')] + [font(sub_size)] * (len(lines) - 1)
    sizes = [text_size(d, t, f) for t, f in zip(lines, fonts)]
    pad, gap = 26, 14
    bw = max(w for w, _ in sizes) + pad * 2
    bh = sum(h for _, h in sizes) + gap * (len(lines) - 1) + pad * 2
    x0, y0 = (W - bw) // 2, y_bottom - bh
    d.rounded_rectangle((x0, y0, x0 + bw, y0 + bh), 18, fill=(10, 14, 20, 200))
    y = y0 + pad
    for (t, f), (w, h) in zip(zip(lines, fonts), sizes):
        d.text(((W - w) // 2, y - 4), t, font=f,
               fill=(255, 255, 255, 255) if f is fonts[0] else (190, 220, 255, 255))
        y += h + gap
    return img


def mission(line1, line2, color2=(160, 255, 170)):
    img = canvas()
    d = ImageDraw.Draw(img)
    d.rectangle((0, 0, W, 108), fill=(8, 12, 18, 215))
    d.text((32, 8), line1, font=font(46, 'bold'), fill=(255, 255, 255, 255))
    d.text((34, 62), line2, font=font(32), fill=color2 + (255,))
    return img


def alert(lines, y_bottom=1040, center_x=W // 2):
    img = canvas()
    d = ImageDraw.Draw(img)
    d.rectangle((0, 0, W - 1, H - 1), outline=(230, 20, 20, 255), width=12)
    f0, f1 = font(64, 'bold'), font(36)
    bw, bh = 780, 70 + 52 * (len(lines) - 1) + 60
    x0, y0 = min(max(20, center_x - bw // 2), W - bw - 20), y_bottom - bh
    d.rounded_rectangle((x0, y0, x0 + bw, y0 + bh), 14, fill=(170, 10, 10, 235),
                        outline=(255, 255, 255, 255), width=3)
    d.text((x0 + 34, y0 + 18), lines[0], font=f0, fill=(255, 255, 255, 255))
    for i, t in enumerate(lines[1:]):
        d.text((x0 + 36, y0 + 106 + 52 * i), t, font=f1, fill=(255, 255, 255, 255))
    return img


def badge(text, x_right=1890, y=128):
    img = canvas()
    d = ImageDraw.Draw(img)
    f = font(36, 'bold')
    w, h = text_size(d, text, f)
    x0 = x_right - w - 40
    d.rounded_rectangle((x0, y, x_right, y + h + 26), 22, fill=(255, 200, 0, 235))
    d.text((x0 + 20, y + 6), text, font=f, fill=(20, 20, 20, 255))
    return img


def label(text, x, y, size=30):
    img = canvas()
    d = ImageDraw.Draw(img)
    f = font(size, 'bold')
    w, h = text_size(d, text, f)
    d.rounded_rectangle((x, y, x + w + 28, y + h + 20), 10, fill=(0, 0, 0, 170))
    d.text((x + 14, y + 4), text, font=f, fill=(255, 255, 255, 255))
    return img


def title(lines, sizes=(88, 44, 36)):
    img = canvas()
    d = ImageDraw.Draw(img)
    fonts = [font(s, 'bold' if i == 0 else 'regular') for i, s in enumerate(sizes)]
    fonts += [fonts[-1]] * (len(lines) - len(fonts))
    hs = [text_size(d, t, f)[1] for t, f in zip(lines, fonts)]
    y = (H - sum(hs) - 30 * (len(lines) - 1)) // 2
    for t, f, h in zip(lines, fonts, hs):
        w, _ = text_size(d, t, f)
        d.text(((W - w) // 2, y), t, font=f, fill=(255, 255, 255, 255))
        y += h + 30
    return img


def render(spec):
    """compose.py の captions の 1 項目 → PIL 画像."""
    style = spec.get('style', 'lower')
    text = spec.get('text', '')
    lines = text if isinstance(text, list) else [text]
    if style == 'lower':
        return lower(lines)
    if style == 'mission':
        return mission(lines[0], lines[1] if len(lines) > 1 else '')
    if style == 'alert':
        return alert(lines, center_x=spec.get('center_x', W // 2))
    if style == 'badge':
        return badge(lines[0])
    if style == 'label':
        return label(lines[0], *spec.get('pos', (20, 20)))
    if style == 'title':
        return title(lines)
    raise ValueError('unknown caption style %r' % style)
