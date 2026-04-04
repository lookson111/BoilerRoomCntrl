#!/bin/bash
# Debug launcher for BoilerRoomCntrl
# Usage:
#   ./debug.sh              - Flash and run with HardFault breakpoint
#   ./debug.sh check        - Flash and verify 20s stability (no crash)
#   ./debug.sh interactive  - Interactive GDB session

set -e

PROJ_DIR="$(cd "$(dirname "$0")" && pwd)"
ELF="${PROJ_DIR}/BoilerRoomCntrl.elf"
CFG="${PROJ_DIR}/openocd.cfg"

if [ ! -f "$ELF" ]; then
    echo "Error: ELF file not found. Run 'make' first."
    exit 1
fi

cleanup() {
    echo "Cleaning up..."
    if [ -n "$OPENOCD_PID" ]; then
        kill "$OPENOCD_PID" 2>/dev/null || true
        wait "$OPENOCD_PID" 2>/dev/null || true
    fi
    # Also kill any remaining openocd/gdb from this script
    pkill -f "openocd -f ${CFG}" 2>/dev/null || true
}

trap cleanup INT TERM EXIT

# Kill any leftover OpenOCD instances
pkill -f "openocd -f ${CFG}" 2>/dev/null || true
sleep 1

echo "=== Starting OpenOCD ==="
openocd -f "$CFG" >/dev/null 2>&1 &
OPENOCD_PID=$!

sleep 3

if ! kill -0 "$OPENOCD_PID" 2>/dev/null; then
    echo "Error: OpenOCD failed to start"
    exit 1
fi

echo "OpenOCD started (PID: $OPENOCD_PID)"
echo ""

GDB_OPTS=(
    -q -batch
    -ex "target remote :3333"
    -ex "file ${ELF}"
    -ex "monitor reset halt"
    -ex "load"
    -ex "monitor reset halt"
    -ex "break HardFault_Handler"
)

case "${1:-}" in
    check)
        echo "=== Flashing and verifying stability (20s) ==="
        gdb-multiarch "${GDB_OPTS[@]}" -ex "continue" 2>&1 &
        GDB_PID=$!
        sleep 24
        if kill -0 "$GDB_PID" 2>/dev/null; then
            echo "=== PASS: No crash after 24 seconds ==="
            kill "$GDB_PID" 2>/dev/null || true
            wait "$GDB_PID" 2>/dev/null || true
        else
            echo "=== FAIL: Crash detected ==="
            kill "$GDB_PID" 2>/dev/null || true
            wait "$GDB_PID" 2>/dev/null || true
            exit 1
        fi
        ;;
    interactive)
        echo "=== Interactive GDB session ==="
        echo "Breakpoints: HardFault_Handler"
        echo "Commands: bt (backtrace), c (continue), q (quit)"
        echo ""
        trap - INT  # Let GDB handle Ctrl-C
        gdb-multiarch -q \
            -ex "target remote :3333" \
            -ex "file ${ELF}" \
            -ex "monitor reset halt" \
            -ex "load" \
            -ex "monitor reset halt" \
            -ex "break HardFault_Handler" \
            -ex "continue"
        ;;
    *)
        echo "=== Flashing and running ==="
        echo "Breakpoint: HardFault_Handler"
        echo "Press Ctrl-C to interrupt"
        echo ""
        trap - INT  # Let GDB handle Ctrl-C
        gdb-multiarch -q \
            -ex "target remote :3333" \
            -ex "file ${ELF}" \
            -ex "monitor reset halt" \
            -ex "load" \
            -ex "monitor reset halt" \
            -ex "break HardFault_Handler" \
            -ex "continue"
        ;;
esac
