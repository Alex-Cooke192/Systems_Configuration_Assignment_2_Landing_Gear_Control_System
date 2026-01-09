#!/usr/bin/env python3

# Main application entry point.
# Owns process lifecycle, initialization, and execution loop.

import logging
import signal
import time
import threading

from landing_gear_controller import LandingGearController
from gear_configuration import GearConfiguration
from altitude_simulator import AltitudeSimulator


# Global execution flag controlling main loop state
_running = True

# Globals retained for interactive inspection when running with python -i
controller: LandingGearController | None = None
sim: AltitudeSimulator | None = None


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

    # Validates LGCS-FR001
    deploy_time_ms = config.compute_deploy_time_ms()
    if deploy_time_ms >= 8000:
        raise ValueError(
            f"LGCS-FR001 violated: deploy_time_ms={deploy_time_ms:.1f} (must be < 8000)"
        )

    clock = time.monotonic

    altitude_sensor = AltitudeSimulator(clock=clock)

    lgc = LandingGearController(
        config=config,
        clock=clock,
        altitude_provider=altitude_sensor.read_altitude_ft,
        normal_conditions_provider=lambda: True,
    )

    return lgc, altitude_sensor


def control_loop(loop_sleep: float = 0.1):
    # Runs controller update at a fixed tick rate
    global _running, controller, sim

    logging.info("Starting control loop (tick=%.3fs)", loop_sleep)

    while _running:
        try:
            # Altitude simulation advances once per tick
            if sim is not None:
                sim.update()

            if controller is not None:
                controller.update()

        except Exception:
            logging.exception("Unhandled exception in control loop")

        time.sleep(loop_sleep)

    logging.info("Control loop terminated")


def command_loop():
    # Awaits commands while the controller continues running
    global _running, controller, sim

    prompt = (
        "[d]=deploy [u]=retract [wow0]=WOW false [wow1]=WOW true "
        "[alt N]=set altitude [state]=print state [q]=quit > "
    )

    while _running:
        try:
            cmd = input(prompt).strip().lower()
        except (EOFError, KeyboardInterrupt):
            _running = False
            break

        if cmd == "d":
            if controller is not None:
                controller.command_gear_down(True)

        elif cmd == "u":
            if controller is not None:
                controller.command_gear_up(True)

        elif cmd == "wow0":
            if controller is not None:
                controller.set_weight_on_wheels(False)

        elif cmd == "wow1":
            if controller is not None:
                controller.set_weight_on_wheels(True)

        elif cmd.startswith("alt "):
            if sim is None:
                print("Altitude sensor is not available.")
                continue
            try:
                value = float(cmd.split(maxsplit=1)[1])
            except ValueError:
                print("Invalid altitude.")
                continue
            sim.set_altitude_ft(value)

        elif cmd == "state":
            if controller is None:
                print("Controller is not available.")
            else:
                print(controller.state)

        elif cmd == "q":
            _running = False
            break

        elif cmd == "":
            continue

        else:
            print("Unknown command.")

    logging.info("Command loop terminated")


def main():
    # Orchestrates application startup sequence
    global controller, sim

    setup_logging()
    setup_signal_handlers()

    controller, sim = initialize()

    # Starts the controller ticking in the background
    t = threading.Thread(target=control_loop, kwargs={"loop_sleep": 0.1}, daemon=True)
    t.start()

    # Blocks awaiting commands while the control loop runs
    command_loop()

    logging.info("Main loop terminated")


if __name__ == "__main__":
    main()
