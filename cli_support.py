"""
Title: CLI Support Utilities and Control Loop Abstractions
Author: Alex Cooke
Date Created: 2026-01-09
Last Modified: 2026-01-10
Version: 1.1

Purpose:
Provides shared support utilities for the LGCS command-line interface and
simulation environment. This module defines mutable wrapper types for interactive
inputs, a thread-safe position sensor bank for fault injection and testing, and
a lightweight control loop abstraction for driving the LandingGearController
either step-wise or in a background thread.

Targeted Requirements:
- None (supporting analysis, simulation, and tooling only)

Scope and Limitations:
- Intended for CLI-driven simulation and test support only.
- ControlLoop timing is approximate and not real-time deterministic.
- Threading model is simplified and not representative of certified avionics tasking.
- Exposes mutable state for ease of interactive testing, not safety-critical use.

Safety Notice:
This software is for academic and illustrative purposes only.
It is not flight-certified and must not be used in operational systems.

Dependencies:
- Python 3.10+
- threading (standard library)
- time (standard library)
- dataclasses (standard library)
- typing (standard library)
- landing_gear_controller.py
- sims/position_simulator.py

Related Documents:
- LGCS Requirements Specification
- LGCS CLI and Simulation Test Architecture Notes

Safety and Certification Disclaimer:
All artefacts in this repository are produced for academic assessment purposes only.
They do not represent certified software and must not be used in real-world aviation
or safety-critical systems.
"""

import threading
import time
from dataclasses import dataclass
from typing import Sequence, Callable, Optional

from landing_gear_controller import LandingGearController
from sims.position_simulator import PositionSensorReading, SensorStatus


@dataclass
class MutableBool:
    value: bool = True


@dataclass
class MutableFloat:
    value: float = 0.0


class PositionSensorBank:
    def __init__(self):
        self._readings: list[PositionSensorReading] = [
            PositionSensorReading(SensorStatus.OK, 0.0),
            PositionSensorReading(SensorStatus.OK, 0.0),
        ]
        self._lock = threading.Lock()

    def set_readings(self, readings: Sequence[PositionSensorReading]) -> None:
        with self._lock:
            self._readings = list(readings)

    def get_readings(self) -> Sequence[PositionSensorReading]:
        with self._lock:
            return list(self._readings)


class ControlLoop:
    def __init__(self, 
                 controller: LandingGearController, 
                 period_s: float = 0.1,
                 on_tick: Optional[Callable] = None,):
        self._controller = controller
        self._period_s = float(period_s)
        self._on_tick = on_tick
        self._running = False
        self._stop_evt = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def period_s(self) -> float:
        return self._period_s

    def set_period(self, period_s: float) -> None:
        self._period_s = max(0.01, float(period_s))

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if not self._running:
            return
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._running = False
        self._thread = None

    def step(self, n: int = 1) -> None:
        for _ in range(max(1, int(n))):
            self._controller.update()
            if self._on_tick:
                self._on_tick(self._controller)

    def _run(self) -> None:
        while not self._stop_evt.is_set():
            self._controller.update()
            if self._on_tick:
                self._on_tick(self._controller)
            time.sleep(self._period_s)
