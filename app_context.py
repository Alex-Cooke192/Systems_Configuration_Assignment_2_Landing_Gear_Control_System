# app_context.py
from dataclasses import dataclass
from threading import Event
from typing import Callable, Sequence, Optional

from gear_configuration import GearConfiguration
from landing_gear_controller import LandingGearController
from sims.position_simulator import PositionSensorReading

# Import these from cli.py (or move them to a small module like inputs.py)
from cli import MutableBool, MutableFloat, PositionSensorBank, ControlLoop

@dataclass
class AppContext:
    controller: LandingGearController
    config: GearConfiguration
    clock: Callable[[], float]
    shutdown_event: Event

    # Rich CLI “environment” inputs
    altitude: MutableFloat
    normal: MutableBool
    power: MutableBool
    wow: MutableBool
    sensors: PositionSensorBank

    # Rich CLI control loop
    loop: ControlLoop

    def shutdown(self) -> None:
        self.shutdown_event.set()
