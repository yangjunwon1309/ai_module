#!/usr/bin/env bash
set -euo pipefail

WEIGHTS_NAME="checkpoint0149.pth"
FILE_ID="1id6oD_iwiNDD6HyCn2ORgRTIKkPD3tUD"   # Google Drive file id
EXPECTED_SHA256=""  # 필요시 무결성 값

PKG_PATH="$(rospack find reltr_scene_graph)"
DEST_DIR="$PKG_PATH/checkpoints"
DEST="$DEST_DIR/$WEIGHTS_NAME"
mkdir -p "$DEST_DIR"

if [ -f "$DEST" ]; then
  echo "[get_weights] already exists: $DEST"
  exit 0
fi

echo "[get_weights] downloading -> $DEST"

if ! command -v gdown >/dev/null 2>&1; then
  python3 -m pip install --user -q gdown
  export PATH="$HOME/.local/bin:$PATH"
fi

gdown --id "$FILE_ID" -O "$DEST"

if [ -n "$EXPECTED_SHA256" ]; then
  ACTUAL="$(sha256sum "$DEST" | awk '{print $1}')"
  [ "$ACTUAL" = "$EXPECTED_SHA256" ] || { echo "[get_weights] sha256 mismatch"; rm -f "$DEST"; exit 2; }
fi

SIZE="$(stat -c%s "$DEST" 2>/dev/null || wc -c < "$DEST")"
if [ "${SIZE:-0}" -lt 10000000 ]; then
  echo "[get_weights] file too small ($SIZE bytes) — download likely failed."
  rm -f "$DEST"
  exit 3
fi

echo "[get_weights] done: $DEST"
