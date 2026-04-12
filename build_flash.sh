#!/usr/bin/env bash
# build_flash.sh — Build and flash BoilerRoomCntrl
#
# Usage:
#   ./build_flash.sh              - Build Debug and flash
#   ./build_flash.sh debug        - Build Debug and flash
#   ./build_flash.sh release      - Build Release and flash
#   ./build_flash.sh both         - Build Debug and Release, flash Debug

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
OPENOCD_CFG="${PROJECT_DIR}/openocd.cfg"

# ---------- check for tools ----------
if ! command -v openocd &>/dev/null; then
    echo "ERROR: openocd not found. Install it first:" >&2
    echo "  sudo apt install openocd" >&2
    exit 1
fi

if ! command -v make &>/dev/null; then
    echo "ERROR: make not found." >&2
    exit 1
fi

# ---------- flash function ----------
flash_bin() {
    local bin_path="$1"
    local label="$2"

    if [[ ! -f "$bin_path" ]]; then
        echo "ERROR: $bin_path not found." >&2
        return 1
    fi

    local size
    size=$(wc -c < "$bin_path")
    echo ""
    echo "=== Flashing $label ($(basename "$bin_path"), ${size} bytes) ==="

    openocd -f "$OPENOCD_CFG" \
        -c "program \"$bin_path\" 0x08000000 verify reset exit"

    echo "=== $label flashed successfully ==="
}

# ---------- main ----------
MODE="${1:-debug}"

case "$MODE" in
    debug)
        echo "=== Building Debug ==="
        make -C "$PROJECT_DIR" BUILD_TYPE=Debug
        flash_bin "${PROJECT_DIR}/Debug/BoilerRoomCntrl.bin" "Debug"
        ;;
    release)
        echo "=== Building Release ==="
        make -C "$PROJECT_DIR" BUILD_TYPE=Release
        flash_bin "${PROJECT_DIR}/Release/BoilerRoomCntrl.bin" "Release"
        ;;
    both)
        echo "=== Building Debug ==="
        make -C "$PROJECT_DIR" BUILD_TYPE=Debug
        echo "=== Building Release ==="
        make -C "$PROJECT_DIR" BUILD_TYPE=Release
        flash_bin "${PROJECT_DIR}/Debug/BoilerRoomCntrl.bin" "Debug"
        echo ""
        echo "Both builds successful. Debug flashed."
        ;;
    *)
        echo "Usage:" >&2
        echo "  $0              - Build Debug and flash" >&2
        echo "  $0 debug        - Build Debug and flash" >&2
        echo "  $0 release      - Build Release and flash" >&2
        echo "  $0 both         - Build Debug and Release, flash Debug" >&2
        exit 1
        ;;
esac
