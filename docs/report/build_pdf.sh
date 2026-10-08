#!/usr/bin/env bash
# report.html → KumaWatch_Patrol_report.pdf(Chromium のヘッドレス印刷)
#   ./docs/report/build_pdf.sh
# Chromium / Google Chrome が必要。ネットに繋がらない場合は IPA フォントで代替表示される。
set -euo pipefail
cd "$(dirname "$0")"
CHROME="${CHROME:-$(command -v chromium || command -v chromium-browser || command -v google-chrome || echo /opt/pw-browsers/chromium-1194/chrome-linux/chrome)}"
"$CHROME" --headless=new --no-sandbox --disable-gpu --no-pdf-header-footer \
  --virtual-time-budget=15000 --run-all-compositor-stages-before-draw \
  --print-to-pdf="$PWD/KumaWatch_Patrol_report.pdf" "file://$PWD/report.html" 2>/dev/null
echo "→ $PWD/KumaWatch_Patrol_report.pdf"
