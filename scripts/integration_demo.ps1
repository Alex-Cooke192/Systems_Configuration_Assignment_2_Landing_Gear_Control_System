# ============================================
# LGCS Integration Demo Script
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

$ErrorActionPreference = "Stop"

Write-Host "============================================"
Write-Host " LGCS Integration Demo"
Write-Host "============================================"
Write-Host ""

# Ensure we are running from repo root
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location (Resolve-Path "$ScriptDir\..")

# --------------------------------------------
# Prepare output directories
# --------------------------------------------
Write-Host "[1/5] Preparing output directories..."

New-Item -ItemType Directory -Force -Path logs | Out-Null
New-Item -ItemType Directory -Force -Path reports | Out-Null

Remove-Item -Force -ErrorAction SilentlyContinue logs\cli_commands.csv
Remove-Item -Force -ErrorAction SilentlyContinue logs\fault_log.txt

# --------------------------------------------
# Run unit tests (QA evidence)
# --------------------------------------------
Write-Host ""
Write-Host "[2/5] Running unit test suite..."

pytest `
  --quiet `
  --disable-warnings `
  --junitxml=reports\unit_tests.xml

if ($LASTEXITCODE -ne 0) {
    Write-Error "Unit tests failed. Aborting integration demo."
    exit 1
}

Write-Host "Unit tests PASSED."

# --------------------------------------------
# Run scripted CLI scenario
# --------------------------------------------
Write-Host ""
Write-Host "[3/5] Running scripted CLI integration scenario..."

$cliCommands = @"
state
sens mix ok 0 ok 0 fail 0
alt 1500
step 1
alt 999
step 1
state
u
wow 1
u
wow 0
u
step 5
state
sens mix ok 0 ok 1
step 1
step 5
state
q
"@

$cliCommands | python main.py | Tee-Object -FilePath logs\cli_demo_output.txt

# --------------------------------------------
# Show generated artefacts
# --------------------------------------------
Write-Host ""
Write-Host "[4/5] Generated artefacts:"

if (Test-Path logs\cli_commands.csv) {
    Write-Host "  - logs\cli_commands.csv (command audit log)"
} else {
    Write-Warning "  - cli_commands.csv NOT FOUND"
}

if (Test-Path logs\fault_log.txt) {
    Write-Host "  - logs\fault_log.txt (fault recorder output)"
} else {
    Write-Host "  - no fault log generated (acceptable for scenario)"
}

Write-Host "  - logs\cli_demo_output.txt (CLI transcript)"
Write-Host "  - reports\unit_tests.xml (test results)"

# --------------------------------------------
# Completion summary
# --------------------------------------------
Write-Host ""
Write-Host "[5/5] Integration demo completed successfully."
Write-Host ""
Write-Host "This demo demonstrates:"
Write-Host " - Component integration (controller, CLI, simulators)"
Write-Host " - Automated QA via pytest"
Write-Host " - Safety and fault-handling behavior"
Write-Host " - Command audit and fault logging"
Write-Host ""
