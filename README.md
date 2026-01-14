# Landing Gear Control System (LGCS) – Coursework Repository

## Overview
This repository contains a complete Python-based simulation and analysis codebase developed for the LGCS coursework. It supports:

- Requirements-to-implementation traceability  
- Deterministic controller behaviour analysis  
- Performance and timing evaluation  
- Fault-handling and fault classification studies  
- Generation of assessment artefacts (logs, reports, presentation support)

All code in this repository is **academic and non-operational**. It is intended solely for analysis, illustration, and assessment purposes.

---

## Safety and Certification Disclaimer
All artefacts in this repository are produced for academic assessment purposes only.  
They do **not** represent flight-certified or safety-approved software and must **not** be used in any operational aviation or safety-critical system.

---

## Repository Structure
The repository structure reflects a complete coursework codebase. The intent is to keep it **auditable**, **testable**, and **traceable**, without unnecessary framework complexity.

Typical contents include:

- `tests/`  
  Unit and performance tests used as objective evidence against requirements (including timing and fault-handling requirements).

- `scripts/` (or top-level runnable scripts)  
  CLI demos, scenario runners, report/log generation utilities.

- `logs/`  
  Generated runtime logs (excluded or cleaned as appropriate).

- `reports/`  
  Generated artefacts used as coursework evidence (plots, summaries, exported results).

- `TRACEABILITY.md`  
  Primary traceability map linking requirements ↔ code ↔ tests ↔ evidence.

---

## Traceability Approach
Traceability is demonstrated through a combination of:

- Requirement identifiers referenced in:
  - Branch names (when appropriate)
  - Commit messages (recommended)
  - Code comments / docstrings (where justified)
  - Tests (as executable evidence)
  - `TRACEABILITY.md` (source of truth)

- Objective evidence produced by:
  - Automated test results
  - Generated logs and reports
  - Reproducible demo scripts

`TRACEABILITY.md` is treated as the **primary index** and should be updated when new behaviour, tests, or artefacts are added.

---

## Branch Naming Convention

Branches are named to reflect **intent and activity**. Requirement identifiers shall only be included when a branch directly targets those requirements.

### 1. Branches Targeting a Single Requirement
Use this pattern when a branch directly implements, verifies, or analyses **one** specific requirement:

