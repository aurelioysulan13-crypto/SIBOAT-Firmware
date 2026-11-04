#!/usr/bin/env bash
set -euo pipefail

MAIN_DIR="firmware/main/SIBOAT_MAIN-ESP32_FIRMWARE_FINAL_BOSS"
CAM_DIR="firmware/cam/SIBOAT_ESP32-CAM_FIRMWARE_FINAL_BOSS"

required=(
  "$MAIN_DIR/SIBOAT_MAIN-ESP32_FIRMWARE_FINAL_BOSS.ino"
  "$MAIN_DIR/siboat_main_types.h"
  "$MAIN_DIR/siboat_secrets.example.h"
  "$CAM_DIR/SIBOAT_ESP32-CAM_FIRMWARE_FINAL_BOSS.ino"
  "$CAM_DIR/siboat_secrets.example.h"
  ".github/workflows/build-firmware.yml"
  "supabase/SIBOAT_CAM_TRANSPORT_AND_DISCOVERY.sql"
  "siboat_inferencing.zip"
)
for file in "${required[@]}"; do
  [[ -s "$file" ]] || { echo "Missing or empty required file: $file" >&2; exit 1; }
done

grep -Fq '#define FW_VERSION "1.7.1-manual-first"' \
  "$MAIN_DIR/SIBOAT_MAIN-ESP32_FIRMWARE_FINAL_BOSS.ino"
grep -Fq '#define CAM_FW_VERSION "1.5.0-camlink-ota"' \
  "$CAM_DIR/SIBOAT_ESP32-CAM_FIRMWARE_FINAL_BOSS.ino"
grep -Fq '#include "siboat_main_types.h"' \
  "$MAIN_DIR/SIBOAT_MAIN-ESP32_FIRMWARE_FINAL_BOSS.ino"

test ! -e "$MAIN_DIR/siboat_secrets.h"
test ! -e "$CAM_DIR/siboat_secrets.h"
echo "SIBOAT repository preflight passed."
