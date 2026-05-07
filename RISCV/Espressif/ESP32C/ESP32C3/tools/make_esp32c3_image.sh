#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 2 ]; then
    echo "usage: $0 <input.elf> <output.bin>" >&2
    exit 2
fi

ELF="$1"
BIN="$2"

python3 -m esptool --chip esp32c3 elf2image \
    --flash_mode dio \
    --flash_freq 40m \
    --flash_size 4MB \
    -o "$BIN" \
    "$ELF"

python3 -m esptool --chip esp32c3 image_info "$BIN" || true
