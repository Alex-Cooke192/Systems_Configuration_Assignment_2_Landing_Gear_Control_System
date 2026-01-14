# ============================================
# LGCS Integration Demo Script (Docker / PowerShell)
# ============================================
# Purpose:
# Runs a repeatable containerised integration demonstration
# of the Landing Gear Control System prototype, including:
#  - Unit tests (QA evidence) run inside container
#  - CLI-driven functional scenario run inside container
#  - Log and artefact generation exported to host
#
# Author: Alex Cooke
# ============================================

$ErrorActionPreference = "Stop"

$ImageName = "lgcs:demo"

Write-Host "============================================"
Write-Host " LGCS Integration Demo (Containerised)"
Write-Host "============================================"
Write-Host ""

# --------------------------------------------
# Ensure we are running from repo root
# --------------------------------------------
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RootDir = (Resolve-Path "$ScriptDir\..").Path
Set-Location $RootDir

# --------------------------------------------
# Check Docker availability
# --------------------------------------------
if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
    Write-Error "Docker not found on PATH."
    exit 1
}

# Optional: quick daemon check (won't be perfect on all setups)
try {
    docker info | Out-Null
} catch {
    Write-Error "Docker daemon not running / not accessible."
    exit 1
}

# --------------------------------------------
# Prepare output directories
# --------------------------------------------
Write-Host "[1/6] Preparing output directories..."

New-Item -ItemType Directory -Force -Path logs | Out-Null
New-Item -ItemType Directory -Force -Path reports | Out-Null

Remove-Item -Force -ErrorAction SilentlyContinue "logs\cli_commands.csv"
Remove-Item -Force -ErrorAction SilentlyContinue "logs\fault_log.txt"
Remove-Item -Force -ErrorAction SilentlyContinue "logs\cli_demo_output.txt"
Remove-Item -Force -ErrorAction SilentlyContinue "reports\unit_tests.xml"

# --------------------------------------------
# Build container image
# --------------------------------------------
Write-Host ""
Write-Host "[2/6] Building Docker image ($ImageName)..."

docker build -t $ImageName .
if ($LASTEXITCODE -ne 0) {
    Write-Error "Docker build failed. Aborting integration demo."
    exit 1
}

# --------------------------------------------
# Run unit tests (QA evidence) in container
# --------------------------------------------
Write-Host ""
Write-Host "[3/6] Running unit tests in container..."

# Use absolute path for volume mapping (Windows-friendly)
$ReportsAbs = (Resolve-Path "reports").Path

docker run --rm `
  -v "${ReportsAbs}:/app/reports" `
  $ImageName `
  python -m pytest `
    --quiet `
    --disable-warnings `
    --junitxml=reports/unit_tests.xml

if ($LASTEXITCODE -ne 0) {
    Write-Error "Unit tests failed in container. Aborting integration demo."
    exit 1
}

Write-Host "Unit tests PASSED."

# --------------------------------------------
# Run scripted CLI integration scenario in container
# --------------------------------------------
Write-Host ""
Write-Host "[4/6] Running extended scripted CLI integration scenario in container..."

$cliCommands = @"
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
step 1
step 1
step 1
step 1
step 1
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
"@

$LogsAbs = (Resolve-Path "logs").Path

# Feed stdin into container (-i) and tee transcript to logs on host
$cliCommands | docker run --rm -i `
  -v "${LogsAbs}:/app/logs" `
  -v "${ReportsAbs}:/app/reports" `
  $ImageName `
  python main.py | Tee-Object -FilePath "logs\cli_demo_output.txt"

if ($LASTEXITCODE -ne 0) {
    Write-Error "CLI scenario failed in container. Aborting integration demo."
    exit 1
}

# --------------------------------------------
# Show generated artefacts
# --------------------------------------------
Write-Host ""
Write-Host "[5/6] Generated artefacts:"

if (Test-Path "logs\cli_commands.csv") {
    Write-Host "  - logs\cli_commands.csv (command audit log)"
} else {
    Write-Warning "  - logs\cli_commands.csv NOT FOUND"
}

if (Test-Path "logs\fault_log.txt") {
    Write-Host "  - logs\fault_log.txt (fault recorder output)"
} else {
    Write-Host "  - no fault log generated (acceptable for scenario)"
}

if (Test-Path "logs\cli_demo_output.txt") {
    Write-Host "  - logs\cli_demo_output.txt (CLI transcript)"
} else {
    Write-Warning "  - logs\cli_demo_output.txt NOT FOUND"
}

if (Test-Path "reports\unit_tests.xml") {
    Write-Host "  - reports\unit_tests.xml (test results)"
} else {
    Write-Warning "  - reports\unit_tests.xml NOT FOUND"
}

# --------------------------------------------
# Completion summary
# --------------------------------------------
Write-Host ""
Write-Host "[6/6] Integration demo completed successfully."
Write-Host ""
Write-Host "This demo demonstrates:"
Write-Host " - Containerised execution of LGCS"
Write-Host " - Automated QA via pytest (in-container)"
Write-Host " - CLI-driven integration scenarios"
Write-Host " - Safety and fault-handling behaviour"
Write-Host " - Command audit and fault logging"
Write-Host ""
