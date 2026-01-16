#!/usr/bin/env bash
# ============================================
# LGCS Integration Demo Script (Docker / Bash)
# ============================================
# Purpose:
# Runs a repeatable containerised integration demonstration
# of the Landing Gear Control System prototype, including:
#  - Unit tests (QA evidence)
#  - CLI-driven functional scenarios
#  - Log and artefact generation
#
# Author: Alex Cooke
# ============================================

set -euo pipefail

IMAGE_NAME="lgcs:demo"

echo "============================================"
echo " LGCS Integration Demo (Containerised)"
echo "============================================"
echo ""

# --------------------------------------------
# Ensure we are running from repo root
# --------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${ROOT_DIR}"

# --------------------------------------------
# Check Docker availability
# --------------------------------------------
if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: Docker not found on PATH."
  exit 1
fi

# --------------------------------------------
# Prepare output directories
# --------------------------------------------
echo "[1/6] Preparing output directories..."

mkdir -p logs reports
rm -f logs/cli_commands.csv
rm -f logs/fault_log.txt
rm -f logs/cli_demo_output.txt
rm -f reports/unit_tests.xml

# --------------------------------------------
# Build container image
# --------------------------------------------
echo ""
echo "[2/6] Building Docker image (${IMAGE_NAME})..."

docker build -t "${IMAGE_NAME}" .

# --------------------------------------------
# Run unit tests (QA evidence) in container
# --------------------------------------------
echo ""
echo "[3/6] Running unit tests in container..."

docker run --rm \
  -v "${ROOT_DIR}/reports:/app/reports" \
  "${IMAGE_NAME}" \
  python -m pytest \
    --quiet \
    --disable-warnings \
    --junitxml=reports/unit_tests.xml

echo "Unit tests PASSED."

# --------------------------------------------
# Run scripted CLI integration scenario
# --------------------------------------------
echo ""
echo "[4/6] Running extended scripted CLI integration scenario in container..."

CLI_COMMANDS=$'
state

alt 1500
wow 0
u
step 3
state

wow 1
u
step 1
state

wow 0
u
step 5
state

sens mix ok 0 ok 0 fail 0
state
step 1
state

sens mix ok 0 ok 1
state
step 10000000
state

sens mix ok 0 ok 0
step 2
state

alt 1001
step 1
alt 999
step 1
alt 1001
step 1
state

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
echo "[5/6] Generated artefacts:"

if [[ -f logs/cli_commands.csv ]]; then
  echo "  - logs/cli_commands.csv (command audit log)"
else
  echo "  - WARNING: logs/cli_commands.csv NOT FOUND"
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
echo "[6/6] Integration demo completed successfully."
echo ""
echo "This demo demonstrates:"
echo " - Containerised execution of LGCS"
echo " - Automated QA via pytest (in-container)"
echo " - CLI-driven integration scenarios"
echo " - Safety and fault-handling behaviour"
echo " - Command audit and fault logging"
echo ""
