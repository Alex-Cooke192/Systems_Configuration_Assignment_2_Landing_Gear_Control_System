"""
Title: Altitude Simulator Utility
Author: Alex Cooke
Date Created: 2026-01-09
Last Modified: 2026-01-09
Version: 1.0

Purpose:
Provides a lightweight, stochastic altitude simulation model for testing and
supporting the Landing Gear Control System (LGCS). The simulator generates
bounded altitude changes with configurable rates and acceleration and supports
both fixed time-step and injected-clock operation.

Targeted Requirements:
- None (supporting analysis or tooling only)

Scope and Limitations:
- Altitude behavior is randomly generated and not based on aerodynamic models
- Intended solely for test stimulation and simulation support
- Vertical motion is limited to configurable bounds and rates
- Does not model aircraft attitude, airspeed, or environmental effects

Safety Notice:
This software is for academic and illustrative purposes only.
It is not flight-certified and must not be used in operational systems.

Dependencies:
- Python 3.10+
- random (standard library)

Related Documents:
- LGCS Requirements Specification
- LGCS Simulation and Test Architecture Documentation

Safety and Certification Disclaimer:
All artefacts in this repository are produced for academic assessment purposes only.
They do not represent certified software and must not be used in real-world aviation
or safety-critical systems.
"""

import random


class AltitudeSimulator:
    def __init__(
        self,
        min_alt=500.0,          # feet
        max_alt=10_000.0,       # feet
        max_fpm=1500.0,         # max climb/descent rate
        max_accel_fps2=5.0,     # vertical acceleration aggressiveness
        rng: random.Random | None = None,
        clock=None,
    ):
        self.min_alt = float(min_alt)
        self.max_alt = float(max_alt)

        self.rng = rng or random.Random()
        self.clock = clock

        self.altitude = self.rng.uniform(self.min_alt, self.max_alt)
        self.vert_speed = 0.0   # ft/sec
        self.max_speed = float(max_fpm) / 60.0
        self.max_accel = float(max_accel_fps2)

        self._last_time = self.clock() if self.clock else None

    def step(self, dt: float) -> float:
        # Advance simulation by dt seconds and return altitude.
        if dt <= 0.0:
            return self.altitude

        accel = self.rng.uniform(-self.max_accel, self.max_accel)
        self.vert_speed += accel * dt

        self.vert_speed = max(-self.max_speed, min(self.vert_speed, self.max_speed))
        self.altitude += self.vert_speed * dt

        if self.altitude <= self.min_alt:
            self.altitude = self.min_alt
            self.vert_speed = abs(self.vert_speed)
        elif self.altitude >= self.max_alt:
            self.altitude = self.max_alt
            self.vert_speed = -abs(self.vert_speed)

        return self.altitude

    def update(self) -> float:
        # Advance simulation using the injected clock.
        if not self.clock:
            raise RuntimeError(
                "AltitudeSimulator.update() requires a clock; use step(dt) instead."
            )

        now = self.clock()
        dt = now - (self._last_time if self._last_time is not None else now)
        self._last_time = now
        return self.step(dt)

    def read_altitude_ft(self) -> float:
        # Return altitude without advancing the simulation.
        return self.altitude

    def set_altitude_ft(self, altitude_ft: float) -> None:
        # Force altitude to a specific value.
        self.altitude = float(altitude_ft)
