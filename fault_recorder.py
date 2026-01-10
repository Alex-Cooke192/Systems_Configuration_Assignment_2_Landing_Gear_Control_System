"""
Title: Fault Recorder Utility
Author: Alex Cooke
Date Created: 2026-01-10
Last Modified: 2026-01-10
Version: 1.0

Purpose:
Provides a simple, append-only fault recording mechanism for the Landing Gear
Control System (LGCS). Faults are timestamped using an injected clock and
persisted to non-volatile storage for later inspection, debugging, or analysis.

Targeted Requirements:
- None (supporting analysis or tooling only)

Scope and Limitations:
- Fault persistence is file-based and append-only
- No fault de-duplication, severity classification, or rollover handling
- Assumes reliable filesystem access
- Intended for simulation, testing, and academic analysis only

Safety Notice:
This software is for academic and illustrative purposes only.
It is not flight-certified and must not be used in operational systems.

Dependencies:
- Python 3.10+
- dataclasses (standard library)
- pathlib (standard library)
- typing (standard library)

Related Documents:
- LGCS Requirements Specification
- LGCS Fault Handling and Diagnostics Notes

Safety and Certification Disclaimer:
All artefacts in this repository are produced for academic assessment purposes only.
They do not represent certified software and must not be used in real-world aviation
or safety-critical systems.
"""

# fault_recorder.py

from dataclasses import dataclass
from pathlib import Path
from typing import Callable


@dataclass
class FaultRecord:
    timestamp_s: float
    fault_code: str


class FaultRecorder:
    def __init__(self, filepath: str | Path, clock: Callable[[], float]):
        self._path = Path(filepath)
        self._clock = clock

        # Ensures directory exists for persistence target.
        if self._path.parent:
            self._path.parent.mkdir(parents=True, exist_ok=True)

    def record(self, fault_code: str) -> FaultRecord:
        # Records a fault code with timestamp to non-volatile storage (append-only).
        ts = float(self._clock())
        rec = FaultRecord(timestamp_s=ts, fault_code=str(fault_code))

        line = f"{rec.timestamp_s:.6f},{rec.fault_code}\n"
        self._path.open("a", encoding="utf-8").write(line)

        return rec
