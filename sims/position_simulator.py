"""
Title: Landing Gear Position Sensor Simulation Models
Author: Alex Cooke
Date Created: 2026-01-09
Last Modified: 2026-01-09
Version: 1.0

Purpose:
Defines data models used to represent simulated landing gear position sensor
outputs for the Landing Gear Control System (LGCS). This module provides a
normalised representation of landing gear extension position together with
explicit sensor health status, enabling fault-tolerant state inference and
sensor disagreement testing within the controller.

Targeted Requirements:
- None (supporting analysis, simulation, and fault-injection tooling only)

Scope and Limitations:
- Models only logical sensor outputs, not physical sensor dynamics.
- Position values are normalised (0.0 = fully UP, 1.0 = fully DOWN).
- Does not simulate noise, latency, hysteresis, or mechanical backlash.
- Intended solely for simulation, testing, and academic evaluation.

Safety Notice:
This software is for academic and illustrative purposes only.
It is not flight-certified and must not be used in operational systems.

Dependencies:
- Python 3.10+
- dataclasses (standard library)
- enum (standard library)

Related Documents:
- LGCS Requirements Specification
- LGCS Sensor Fault Handling Requirements (FTHR Series)
- LGCS Simulation and Test Architecture Notes

Safety and Certification Disclaimer:
All artefacts in this repository are produced for academic assessment purposes only.
They do not represent certified software and must not be used in real-world aviation
or safety-critical systems.
"""

from dataclasses import dataclass
from enum import Enum, auto

class SensorStatus(Enum):
    OK = auto()
    FAILED = auto()

@dataclass(frozen=True)
class PositionSensorReading:
    status: SensorStatus
    position_norm: float  # 0.0 = fully up, 1.0 = fully down
