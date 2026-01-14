"""
Title: Landing Gear Control System Application Entry Point
Author: Alex Cooke
Date Created: 2026-01-09
Last Modified: 2026-01-09
Version: 1.0

Purpose:
Initializes and launches the Landing Gear Control System (LGCS) simulation
environment. This script performs system configuration validation, dependency
wiring, signal handling setup, and transfers execution control to the interactive
CLI. It represents the primary executable entry point for the LGCS application.

Targeted Requirements:
- LGCS-FR001: The landing gear system shall transition from the UP state to the
  DOWN state in < 8000 milliseconds following a valid deploy command.

Scope and Limitations:
- Intended for simulation and academic evaluation only
- Performs startup-time configuration validation but not runtime certification
- Assumes a POSIX-compatible environment for signal handling
- Relies on injected simulation components rather than real hardware interfaces

Safety Notice:
This software is for academic and illustrative purposes only.
It is not flight-certified and must not be used in operational systems.

Dependencies:
- Python 3.10+
- logging (standard library)
- signal (standard library)
- threading (standard library)
- pathlib (standard library)

Related Documents:
- LGCS Requirements Specification
- LGCS System Architecture and Integration Notes
- LGCS CLI Interface Documentation

Safety and Certification Disclaimer:
All artefacts in this repository are produced for academic assessment purposes only.
They do not represent certified software and must not be used in real-world aviation
or safety-critical systems.
"""

import logging
import signal
import time
from threading import Event
from pathlib import Path

from gear_configuration import GearConfiguration
from landing_gear_controller import LandingGearController
from app_context import AppContext
from cli import run_rich_cli
from cli_support import MutableBool, MutableFloat, PositionSensorBank, ControlLoop

try:
    from fault_recorder import FaultRecorder
except Exception:
    FaultRecorder = None


def setup_logging():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )


def setup_signal_handlers(ctx: AppContext):
    def _handle_shutdown(signum, frame):
        logging.info("Shutdown signal received (%s)", signum)
        ctx.shutdown()
    signal.signal(signal.SIGINT, _handle_shutdown)
    signal.signal(signal.SIGTERM, _handle_shutdown)


def initialize() -> AppContext:
    logging.info("Initializing application")

    config = GearConfiguration(
        name="LG-1",
        pump_latency_ms=200,
        actuator_speed_mm_per_100ms=50.0,
        extension_distance_mm=500,
        lock_time_ms=300,
        requirement_time_ms=8000,
    )

    deploy_time_ms = config.compute_deploy_time_ms()
    if deploy_time_ms >= 8000:
        raise ValueError(
            f"LGCS-FR001 violated: deploy_time_ms={deploy_time_ms:.1f} (must be < 8000)"
        )

    clock = time.monotonic

    # Rich CLI mutable environment inputs
    altitude = MutableFloat(5000.0)
    normal = MutableBool(True)
    power = MutableBool(True)
    wow = MutableBool(True)
    sensors = PositionSensorBank()

    def altitude_provider() -> float:
        return float(altitude.value)

    def normal_provider() -> bool:
        return bool(normal.value)

    def power_provider() -> bool:
        return bool(power.value)

    def sensors_provider():
        return sensors.get_readings()

    fault_recorder = None
    if FaultRecorder is not None:
        fault_recorder = FaultRecorder(filepath=Path("logs/fault_log.csv"), clock=clock)

    controller = LandingGearController(
        config=config,
        clock=clock,
        altitude_provider=altitude_provider,
        normal_conditions_provider=normal_provider,
        primary_power_present_provider=power_provider,
        position_sensors_provider=sensors_provider,
        fault_recorder=fault_recorder,
    )

    controller.set_weight_on_wheels(wow.value)

    loop = ControlLoop(controller, period_s=0.1)

    return AppContext(
        controller=controller,
        config=config,
        clock=clock,
        shutdown_event=Event(),
        altitude=altitude,
        normal=normal,
        power=power,
        wow=wow,
        sensors=sensors,
        loop=loop,
    )


def main():
    setup_logging()
    ctx = initialize()
    setup_signal_handlers(ctx)

    # CLI owns run/stop/step. main just hands over control.
    raise SystemExit(run_rich_cli(ctx))


if __name__ == "__main__":
    main()
