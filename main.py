#!/usr/bin/env python3

# Main application entry point.
# Owns process lifecycle, initialization, and execution loop.

import logging
import signal
import sys
import time
from landing_gear_controller import LandingGearController
from gear_configuration import GearConfiguration


# Global execution flag controlling main loop state
_running = True


def _handle_shutdown(signum, frame):
    # Handles OS shutdown signals and triggers graceful termination
    global _running
    logging.info("Shutdown signal received (%s)", signum)
    _running = False


def setup_signal_handlers():
    # Registers signal handlers for controlled shutdown
    signal.signal(signal.SIGINT, _handle_shutdown)
    signal.signal(signal.SIGTERM, _handle_shutdown)


def setup_logging():
    # Configures process-wide logging behavior
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )


def initialize():
    # Initializes application resources and dependencies
    logging.info("Initializing application")

    config = GearConfiguration(
        name="LG-1",
        pump_latency_ms=200,
        actuator_speed_mm_per_100ms=50.0,
        extension_distance_mm=500,
        lock_time_ms=300,
        requirement_time_ms=5000,
    )

    # Used to validate LGCS-FR001
    deploy_time_ms = config.compute_deploy_time_ms()
    if deploy_time_ms >= 8000:
        raise ValueError(
            f"LGCS-FR001 violated: deploy_time_ms={deploy_time_ms:.1f} (must be < 8000)"
        )

    controller = LandingGearController(config)
    controller._deploy_requested = True
    return controller


def cycle(controller: LandingGearController):
    # Executes a single iteration of the main processing loop
    logging.debug("Executing cycle")
    controller.update()
    # Placeholder for core workload


def run(controller, loop_sleep: float = 1.0,):
    # Drives the main execution loop and enforces error isolation
    logging.info("Starting main loop")

    while _running:
        try:
            cycle(controller)
        except Exception:
            # Captures unexpected failures without crashing the process
            logging.exception("Unhandled exception in cycle")

        time.sleep(loop_sleep)

    logging.info("Main loop terminated")


def main():
    # Orchestrates application startup sequence
    setup_logging()
    setup_signal_handlers()
    controller = initialize()
    run(controller)


if __name__ == "__main__":
    # Delegates execution to main entrypoint
    sys.exit(main())
