"""
Title: Landing Gear Hardware Configuration Model (GearConfiguration)
Author: Alex Cooke
Date Created: 2026-01-09
Last Modified: 2026-01-09
Version: 1.0

Purpose:
Defines an immutable data model representing the physical and timing-related
characteristics of the landing gear hardware. This configuration is used by
the Landing Gear Controller to derive worst-case deployment timing and to
evaluate compliance with timing-related functional requirements.

Targeted Requirements:
- LGCS-FR001: Provides computed deploy timing to verify UP-to-DOWN transition
  occurs within the specified maximum duration.

Scope and Limitations:
- Models worst-case theoretical timing only; no stochastic variation or
  sensor feedback is included.
- Assumes constant actuator speed and no mid-stroke interruptions.
- Retract timing is assumed symmetric unless otherwise modeled.
- Configuration values are static and immutable once instantiated.

Safety Notice:
This software is for academic and illustrative purposes only.
It is not flight-certified and must not be used in operational systems.

Dependencies:
- Python 3.10+
- dataclasses (standard library)

Related Documents:
- LGCS Requirements Specification
- LGCS System Architecture Description
- SRATS-006 Timing Derivation Records

Safety and Certification Disclaimer:
All artefacts in this repository are produced for academic assessment purposes only.
They do not represent certified software and must not be used in real-world aviation
or safety-critical systems.
"""

from dataclasses import dataclass

@dataclass(frozen=True)
class GearConfiguration:
    # Immutable landing gear hardware configuration.
    name: str
    pump_latency_ms: int
    actuator_speed_mm_per_100ms: float
    extension_distance_mm: int
    lock_time_ms: int
    requirement_time_ms: int

    def compute_deploy_time_ms(self) -> float:
        # Worst-case theoretical deploy time.
        # Pure calculation, no side effects.
        actuator_speed_mm_per_ms = self.actuator_speed_mm_per_100ms / 100.0

        extension_time_ms = (
            self.extension_distance_mm / actuator_speed_mm_per_ms
        )

        return (
            self.pump_latency_ms +
            extension_time_ms +
            self.lock_time_ms
        )
    
    def meets_deploy_requirement(self) -> bool:
        return self.compute_deploy_time_ms() <= self.requirement_time_ms
    
    



    
