# command_recorder.py
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable
import threading


@dataclass
class CommandRecorder:
    filepath: Path
    clock: Callable[[], float]

    def __post_init__(self) -> None:
        self._lock = threading.Lock()
        self.filepath.parent.mkdir(parents=True, exist_ok=True)

        # Optional: write header once
        if not self.filepath.exists():
            with self.filepath.open("w", encoding="utf-8") as f:
                f.write("timestamp,command,action,success\n")

    def record(
        self,
        *,
        command: str,
        action: str,
        success: bool,
    ) -> None:
        ts = self.clock()
        line = f"{ts:.6f},{command.strip()},{action},{success}\n"

        with self._lock:
            with self.filepath.open("a", encoding="utf-8") as f:
                f.write(line)
                f.flush()
