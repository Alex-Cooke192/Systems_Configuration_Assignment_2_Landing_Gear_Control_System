# app_context.py
from dataclasses import dataclass
from threading import Event
from typing import Callable

from gear_configuration import GearConfiguration
from landing_gear_controller import LandingGearController

from cli_support import MutableBool, MutableFloat, PositionSensorBank, ControlLoop


@dataclass
class AppContext:
    controller: LandingGearController
    config: GearConfiguration
    clock: Callable[[], float]
    shutdown_event: Event

    altitude: MutableFloat
    normal: MutableBool
    power: MutableBool
    wow: MutableBool
    sensors: PositionSensorBank
    loop: ControlLoop

    def shutdown(self) -> None:
        self.shutdown_event.set()
