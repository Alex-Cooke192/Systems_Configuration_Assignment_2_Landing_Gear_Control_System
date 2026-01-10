"""
Title: Application Context Container for LGCS
Author: Alex Cooke
Date Created: 2026-01-10
Last Modified: 2026-01-10
Version: 1.0

Purpose:
Defines a central application context object for the Landing Gear Control System
(LGCS) simulation. The AppContext aggregates core system components, shared
configuration, environment inputs, and lifecycle control primitives into a
single, explicit container to simplify wiring, dependency management, and
controlled shutdown across the application.

Targeted Requirements:
- None (supporting analysis, integration, and tooling only)

Scope and Limitations:
- Intended for simulation and CLI-driven execution only.
- Acts purely as a dependency container; contains no control or safety logic.
- Not intended to represent certified avionics process partitioning or tasking.

Safety Notice:
This software is for academic and illustrative purposes only.
It is not flight-certified and must not be used in operational systems.

Dependencies:
- Python 3.10+
- dataclasses (standard library)
- threading (standard library)
- typing (standard library)
- gear_configuration.py
- landing_gear_controller.py
- cli_support.py

Related Documents:
- LGCS Requirements Specification
- LGCS System Architecture and Integration Notes

Safety and Certification Disclaimer:
All artefacts in this repository are produced for academic assessment purposes only.
They do not represent certified software and must not be used in real-world aviation
or safety-critical systems.
"""

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
