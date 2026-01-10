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
