"""
Title: Landing Gear State Definitions (LGCS GearState Enum)
Author: Alex Cooke
Date Created: 2026-01-09
Last Modified: 2026-01-09
Version: 1.0

Purpose:
Defines the authoritative set of landing gear system states used by the
Landing Gear Control System (LGCS). These states represent both stable
locked configurations and transient motion or fault conditions, and are
used by the landing gear controller state machine to enforce safe and
deterministic behavior.

Targeted Requirements:
- LGCS-FR001: Defines UP, DOWN, and transitional states used to measure deploy timing.
- LGCS-FR002: Provides DOWN_LOCKED and TRANSITIONING_UP states to support safe retract logic.
- LGCS-FR004: Provides FAULT and ABNORMAL states used to inhibit unsafe actuation.

Scope and Limitations:
- This enumeration defines logical system states only; it does not encode
  actuator positions, sensor validity, or fault causes.
- FAULT and ABNORMAL states are treated as externally commanded conditions.
- No hierarchy or substates are modeled.

Safety Notice:
This software is for academic and illustrative purposes only.
It is not flight-certified and must not be used in operational systems.

Dependencies:
- Python 3.10+
- enum (standard library)

Related Documents:
- LGCS Requirements Specification
- LGCS System Architecture Description
- System Safety Assessment (SSA)

Safety and Certification Disclaimer:
All artefacts in this repository are produced for academic assessment purposes only.
They do not represent certified software and must not be used in real-world aviation
or safety-critical systems.
"""

from enum import Enum, auto

class GearState(Enum):
    RESET = auto()
    UP_LOCKED = auto()
    TRANSITIONING_DOWN = auto()
    DOWN_LOCKED = auto()
    TRANSITIONING_UP = auto()
    FAULT = auto()
    ABNORMAL = auto()
    