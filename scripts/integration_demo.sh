#!/usr/bin/env bash
# ============================================
# LGCS Integration Demo Script (macOS / Bash)
# ============================================
# Purpose:
# Runs a repeatable integration demonstration of the
# Landing Gear Control System prototype, including:
#  - Unit tests (QA evidence)
#  - CLI-driven functional scenario
#  - Log and artefact generation
#
# Author: Alex Cooke
# ============================================

set -euo pipefail

echo "============================================"
echo " LGCS Integration Demo"
echo "============================================"
echo ""

# Ensure we are running from repo root (script is assumed in a subfolder e.g. scripts/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."

# Pick a python command that exists (macOS often uses python3)
if command -v python3 >/dev/null 2>&1; then
  PY="python3"
elif command -v python >/dev/null 2>&1; then
  PY="python"
else
  echo "ERROR: Neither python3 nor python found on PATH."
  exit 1
fi

# --------------------------------------------
# Prepare output directories
# --------------------------------------------
echo "[1/5] Preparing output directories..."

mkdir -p logs reports

rm -f logs/cli_commands.csv
rm -f logs/fault_log.txt

# --------------------------------------------
# Run unit tests (QA evidence)
# --------------------------------------------
echo ""
echo "[2/5] Running unit test suite..."

# Run pytest via python to avoid PATH issues
"${PY}" -m pytest \
  --quiet \
  --disable-warnings \
  --junitxml=reports/unit_tests.xml

echo "Unit tests PASSED."

# --------------------------------------------
# Run scripted CLI scenario
# --------------------------------------------
echo ""
echo "[3/5] Running scripted CLI integration scenario..."

CLI_COMMANDS=$'state\nsens mix ok 0 ok 0 fail 0\nalt 1500\nstep 1\nalt 999\nstep 1\nstate\nu\nwow 1\nu\nwow 0\nu\nstep 5\nstate\nsens mix ok 0 ok 1\nstep 1\nstep 5\nstate\nq\n'

# Pipe commands into CLI and save transcript (while also printing to terminal)
printf "%s" "${CLI_COMMANDS}" | "${PY}" main.py | tee logs/cli_demo_output.txt

# --------------------------------------------
# Show generated artefacts
# --------------------------------------------
echo ""
echo "[4/5] Generated artefacts:"

if [[ -f logs/cli_commands.csv ]]; then
  echo "  - logs/cli_commands.csv (command audit log)"
else
  echo "  - WARNING: cli_commands.csv NOT FOUND"
fi

if [[ -f logs/fault_log.txt ]]; then
  echo "  - logs/fault_log.txt (fault recorder output)"
else
  echo "  - no fault log generated (acceptable for scenario)"
fi

echo "  - logs/cli_demo_output.txt (CLI transcript)"
echo "  - reports/unit_tests.xml (test results)"

# --------------------------------------------
# Completion summary
# --------------------------------------------
echo ""
echo "[5/5] Integration demo completed successfully."
echo ""
echo "This demo demonstrates:"
echo " - Component integration (controller, CLI, simulators)"
echo " - Automated QA via pytest"
echo " - Safety and fault-handling behavior"
echo " - Command audit and fault logging"
echo ""
