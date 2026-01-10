#!/usr/bin/env python3
import logging
import signal
import time
import threading

from landing_gear_controller import LandingGearController
from gear_configuration import GearConfiguration
from sims.altitude_simulator import AltitudeSimulator
from app_context import AppContext


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
        requirement_time_ms=5000,
    )

    # Validates LGCS-FR001 (your existing check)
    deploy_time_ms = config.compute_deploy_time_ms()
    if deploy_time_ms >= 8000:
        raise ValueError(
            f"LGCS-FR001 violated: deploy_time_ms={deploy_time_ms:.1f} (must be < 8000)"
        )

    clock = time.monotonic
    sim = AltitudeSimulator(clock=clock)

    controller = LandingGearController(
        config=config,
        clock=clock,
        altitude_provider=sim.read_altitude_ft,
        normal_conditions_provider=lambda: True,
    )

    from threading import Event
    ctx = AppContext(
        controller=controller,
        sim=sim,
        config=config,
        clock=clock,
        shutdown_event=Event(),
    )
    return ctx


def control_loop(ctx: AppContext, loop_sleep: float = 0.1):
    logging.info("Starting control loop (tick=%.3fs)", loop_sleep)

    while not ctx.shutdown_event.is_set():
        try:
            ctx.sim.update()
            ctx.controller.update()
        except Exception:
            logging.exception("Unhandled exception in control loop")

        time.sleep(loop_sleep)

    logging.info("Control loop terminated")


def command_loop(ctx: AppContext):
    prompt = (
        "[d]=deploy [u]=retract [wow0]=WOW false [wow1]=WOW true "
        "[alt N]=set altitude [state]=print state [q]=quit > "
    )

    while not ctx.shutdown_event.is_set():
        try:
            cmd = input(prompt).strip().lower()
        except (EOFError, KeyboardInterrupt):
            ctx.shutdown()
            break

        if cmd == "d":
            ctx.controller.command_gear_down(True)

        elif cmd == "u":
            ctx.controller.command_gear_up(True)

        elif cmd == "wow0":
            ctx.controller.set_weight_on_wheels(False)

        elif cmd == "wow1":
            ctx.controller.set_weight_on_wheels(True)

        elif cmd.startswith("alt "):
            try:
                value = float(cmd.split(maxsplit=1)[1])
            except ValueError:
                print("Invalid altitude.")
                continue
            ctx.sim.set_altitude_ft(value)

        elif cmd == "state":
            print(ctx.controller.state)

        elif cmd == "q":
            ctx.shutdown()
            break

        elif cmd == "":
            continue

        else:
            print("Unknown command.")

    logging.info("Command loop terminated")


def main():
    setup_logging()
    ctx = initialize()
    setup_signal_handlers(ctx)

    t = threading.Thread(target=control_loop, args=(ctx,), kwargs={"loop_sleep": 0.1}, daemon=True)
    t.start()

    command_loop(ctx)

    logging.info("Main loop terminated")


if __name__ == "__main__":
    main()
