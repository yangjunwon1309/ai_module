#!/usr/bin/env bash
set -euo pipefail

# === 설정 =========================================================
WEIGHTS_NAME="checkpoint0149.pth"
FILE_ID="1id6oD_iwiNDD6HyCn2ORgRTIKkPD3tUD" # Google Drive 파일 ID
EXPECTED_SHA256="" #(모르면 빈 값 유지)
# ================================================================

# 패키지 위치
PKG_PATH="$(rospack find reltr_scene_graph)"
DEST_DIR="$PKG_PATH/checkpoints"
DEST="$DEST_DIR/$WEIGHTS_NAME"
mkdir -p "$DEST_DIR"

if [ -f "$DEST" ]; then
  echo "[get_weights] already exists: $DEST"
  exit 0
fi

echo "[get_weights] downloading -> $DEST"

# gdown이 없으면 설치 (사용자 홈에 설치)
if ! command -v gdown >/dev/null 2>&1; then
  python3 -m pip install --user -q gdown
  # WSL/일반 우분투에서 PATH에 반영
  export PATH="$HOME/.local/bin:$PATH"
fi

# Google Drive에서 다운로드
gdown --id "$FILE_ID" -O "$DEST"

# (선택) 무결성 검사
if [ -n "$EXPECTED_SHA256" ]; then
  ACTUAL="$(sha256sum "$DEST" | awk '{print $1}')"
  if [ "$ACTUAL" != "$EXPECTED_SHA256" ]; then
    echo "[get_weights] sha256 mismatch!"
    rm -f "$DEST"
    exit 2
  fi
fi

# 간단한 크기 체크(10MB 미만이면 보통 실패로 간주)
SIZE="$(stat -c%s "$DEST" 2>/dev/null || wc -c < "$DEST")"
if [ "${SIZE:-0}" -lt 10000000 ]; then
  echo "[get_weights] file too small ($SIZE bytes) — download likely failed."
  rm -f "$DEST"
  exit 3
fi

echo "[get_weights] done: $DEST"