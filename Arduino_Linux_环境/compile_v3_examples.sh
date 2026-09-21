#!/usr/bin/env bash
set -u

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUINO_IDE="$BASE_DIR/arduino-1.8.19/arduino"
EXAMPLES_DIR="/home/ruihu/Desktop/yjj/FDC/Deng-s-foc-controller-master/Dengs FOC V3.0/Dengs FOC V3.0 测试例程(支持库SimpleFOC 2.2.1)"
BUILD_ROOT="$BASE_DIR/build/batch_v3"
LOG_ROOT="$BASE_DIR/build/logs_v3"
BOARD_FQBN="esp32:esp32:esp32"

mkdir -p "$BUILD_ROOT" "$LOG_ROOT"

sketches=()
if [ "$#" -eq 0 ]; then
  while IFS= read -r -d '' sketch; do
    sketches+=("$sketch")
  done < <(find "$EXAMPLES_DIR" -name '*.ino' -print0 | sort -z)
else
  for item in "$@"; do
    if [ -d "$item" ]; then
      while IFS= read -r -d '' sketch; do
        sketches+=("$sketch")
      done < <(find "$item" -name '*.ino' -print0 | sort -z)
    elif [ -f "$item" ]; then
      sketches+=("$item")
    else
      echo "Skip missing path: $item"
    fi
  done
fi

if [ "${#sketches[@]}" -eq 0 ]; then
  echo "No .ino sketches found."
  exit 1
fi

pass=0
fail=0

for sketch in "${sketches[@]}"; do
  name="$(basename "${sketch%.*}")"
  hash="$(printf '%s' "$sketch" | sha1sum | cut -c1-10)"
  build_dir="$BUILD_ROOT/${name}_${hash}"
  log_file="$LOG_ROOT/${name}_${hash}.log"

  mkdir -p "$build_dir"
  echo "==> Verifying: $sketch"
  if "$ARDUINO_IDE" --verify --board "$BOARD_FQBN" --pref "build.path=$build_dir" "$sketch" >"$log_file" 2>&1; then
    echo "PASS: $name"
    pass=$((pass + 1))
  else
    echo "FAIL: $name"
    echo "      Log: $log_file"
    fail=$((fail + 1))
  fi
done

echo
echo "Summary: $pass passed, $fail failed"
echo "Logs: $LOG_ROOT"

if [ "$fail" -ne 0 ]; then
  exit 2
fi
