# cli_support.py
import threading
import time
from dataclasses import dataclass
from typing import Sequence

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
    def __init__(self, controller: LandingGearController, period_s: float = 0.1):
        self._controller = controller
        self._period_s = float(period_s)
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

    def _run(self) -> None:
        while not self._stop_evt.is_set():
            self._controller.update()
            time.sleep(self._period_s)
