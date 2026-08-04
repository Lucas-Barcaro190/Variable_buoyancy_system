#!/usr/bin/env bash
# run_renode.sh - Launch Renode VBS RP2040 Simulation

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

ELF_PATH="${1:-$WORKSPACE_ROOT/build/vbs_cpp_portugal.elf}"

echo "=================================================="
echo " Starting VBS Renode Simulation (RP2040 Emulation) "
echo " Target ELF: ${ELF_PATH}"
echo "=================================================="

if [ ! -f "$ELF_PATH" ]; then
    echo "Warning: Firmware binary $ELF_PATH does not exist."
    echo "Please build the project first."
fi

cd "$WORKSPACE_ROOT"
if command -v renode &> /dev/null; then
    renode -e "\$bin=@${ELF_PATH}; include @renode/vbs_simulation.resc"
else
    echo "Error: renode command not found in PATH."
    echo "Download Renode from https://renode.io and add to PATH."
fi
