#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
STUBS="$ROOT/test/native/stubs"
BUILD_DIR="$ROOT/test/native/build"
BIN="$BUILD_DIR/test_logic"

mkdir -p "$BUILD_DIR"

g++ -std=c++11 -Wall -Wextra -Werror \
  -DDFGAS_HOST_TEST=1 \
  -DARDUINO_ESP32_DEV \
  -I"$ROOT" \
  -I"$STUBS" \
  "$ROOT/DFRobot_MultiGasSensor.cpp" \
  "$ROOT/test/native/stubs/stubs.cpp" \
  "$ROOT/test/native/test_logic.cpp" \
  -o "$BIN" \
  -lm

"$BIN"
