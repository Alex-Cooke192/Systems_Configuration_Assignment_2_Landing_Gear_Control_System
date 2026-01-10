from dataclasses import dataclass
from threading import Event
from landing_gear_controller import LandingGearController
from sims.altitude_simulator import AltitudeSimulator
from gear_configuration import GearConfiguration
from typing import Callable

@dataclass
class AppContext:
    controller: LandingGearController
    sim: AltitudeSimulator
    config: GearConfiguration
    clock: Callable[[], float]
    shutdown_event: Event

    def shutdown(self) -> None:
        self.shutdown_event.set()
