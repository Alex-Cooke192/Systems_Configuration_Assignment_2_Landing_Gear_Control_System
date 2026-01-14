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
echo "[4/6] Running extended scripted CLI integration scenario in container..."

CLI_COMMANDS=$'
# ----------------------------
# Scenario A: Baseline status
# ----------------------------
state

# ----------------------------
# Scenario B: Nominal deploy (safe altitude / WOW handling)
# ----------------------------
alt 1500
wow 0
u
step 3
state

# ----------------------------
# Scenario C: Attempt retract on ground (safety inhibit example)
# (If your logic prevents gear-up with WOW=1, this should be rejected)
# ----------------------------
wow 1
u
step 1
state

# ----------------------------
# Scenario D: Nominal retract (airborne)
# ----------------------------
wow 0
u
step 5
state

# ----------------------------
# Scenario E: Sensor injection - mixed readings + single failure
# (Exercises FTHR001 style behaviour if your CLI supports it)
# ----------------------------
sens mix ok 0 ok 0 fail 0
state
step 1
state

# ----------------------------
# Scenario F: Persistent conflict (two OK disagreeing sensors)
# - hold for long enough to exceed persistence threshold
# - should trigger PR004 classification timing if implemented
# ----------------------------
sens mix ok 0 ok 1
state
step 1
step 1
step 1
step 1
step 1
state

# ----------------------------
# Scenario G: Return to normal sensors / clear condition
# ----------------------------
sens mix ok 0 ok 0
step 2
state

# ----------------------------
# Scenario H: Boundary altitude behaviour
# (hover around threshold to show no chatter if you have hysteresis)
# ----------------------------
alt 1001
step 1
alt 999
step 1
alt 1001
step 1
state

# ----------------------------
# Scenario I: Quit
# ----------------------------
q
'
printf "%s" "${CLI_COMMANDS}" | docker run --rm -i \
  -v "${ROOT_DIR}/logs:/app/logs" \
  -v "${ROOT_DIR}/reports:/app/reports" \
  "${IMAGE_NAME}" \
  python main.py | tee logs/cli_demo_output.txt

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
