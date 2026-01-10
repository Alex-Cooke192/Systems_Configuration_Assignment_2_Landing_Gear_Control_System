#!/usr/bin/env python3

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence

from gear_configuration import GearConfiguration
from gear_states import GearState
from landing_gear_controller import LandingGearController
from sims.position_simulator import PositionSensorReading, SensorStatus
from cli_support import MutableBool, MutableFloat, PositionSensorBank, ControlLoop
from app_context import AppContext

try:
    from fault_recorder import FaultRecorder
except Exception:
    FaultRecorder = None


def _fmt_sensor(r: PositionSensorReading) -> str:
    status = "OK" if r.status == SensorStatus.OK else "FAILED"
    return f"{status}:{r.position_norm:.3f}"


def _print_status(
    controller: LandingGearController,
    altitude: MutableFloat,
    normal: MutableBool,
    power: MutableBool,
    wow: MutableBool,
    sensors: PositionSensorBank,
) -> None:
    print("\n=== STATUS ===")
    print(f"State: {controller.state.name}")
    print(f"WOW: {wow.value}")
    print(f"Altitude_ft: {altitude.value:.1f}  Normal: {normal.value}")
    print(f"PrimaryPowerPresent: {power.value}")
    print(f"PositionEstimateNorm: {controller.position_estimate_norm}")

    # Maintenance fault visibility
    mf_active = getattr(controller, "_maintenance_fault_active", None)
    mf_codes = getattr(controller, "_maintenance_fault_codes", None)
    if mf_active is not None:
        print(f"MaintenanceFaultActive: {mf_active}")
    if mf_codes is not None:
        print(f"MaintenanceFaultCodes: {sorted(list(mf_codes))}")

    # PR004 visibility
    if hasattr(controller, "_fault_classified_ts") and hasattr(controller, "fault_classification_latency_ms"):
        codes = sorted(list(getattr(controller, "_fault_classified_ts").keys()))
        if codes:
            print("FaultClassificationLatencyMs:")
            for c in codes:
                lat = controller.fault_classification_latency_ms(c)
                print(f"  - {c}: {lat:.3f}" if lat is not None else f"  - {c}: None")

    # FTHR002 debug visibility
    sc_start = getattr(controller, "_sensor_conflict_started_at", None)
    sc_latched = getattr(controller, "_sensor_conflict_fault_latched", None)
    if sc_start is not None or sc_latched is not None:
        print(f"SensorConflictStartedAt: {sc_start}")
        print(f"SensorConflictFaultLatched: {sc_latched}")

    print("Sensors:", ", ".join(_fmt_sensor(r) for r in sensors.get_readings()))
    print("=============\n")


class StateAnnunciator:
    """
    FR003: Emits a visual indication whenever the landing gear state changes.
    CLI-only presentation logic.
    """
    def __init__(self):
        self._last_state = None

    def __call__(self, controller: LandingGearController) -> None:
        state = controller.state
        if state != self._last_state:
            name = state.name if hasattr(state, "name") else str(state)
            print(f"STATE: {name}")
            self._last_state = state


def _reset_controller(controller: LandingGearController) -> None:
    # Reset transition is performed by returning to RESET state and clearing relevant latches.
    controller.enter_state(GearState.RESET)

    if hasattr(controller, "_auto_deploy_latched"):
        controller._auto_deploy_latched = False
    if hasattr(controller, "_low_alt_warning_active"):
        controller._low_alt_warning_active = False
    if hasattr(controller, "_sr004_power_loss_latched"):
        controller._sr004_power_loss_latched = False

    if hasattr(controller, "_sensor_conflict_started_at"):
        controller._sensor_conflict_started_at = None
    if hasattr(controller, "_sensor_conflict_fault_latched"):
        controller._sensor_conflict_fault_latched = False


def _print_help() -> None:
    print(
        """
Commands
  help                         Print help
  q                            Quit

Loop control
  run [period_s]               Start background update loop (default period unchanged)
  stop                         Stop background loop
  step [n]                     Run n update ticks (default 1)
  period <seconds>             Set background period (min 0.01)

Pilot / environment inputs
  d                            Issue deploy command (command_gear_down(True))
  u                            Issue retract command (command_gear_up(True))
  wow 0|1                      Set weight-on-wheels FALSE/TRUE
  alt <feet>                   Set altitude (float)
  normal 0|1                   Set normal conditions FALSE/TRUE
  power 0|1                    Set primary control power present FALSE/TRUE

Position sensors
  sens show                    Print current sensor list
  sens ok <v> ok <v> ...       Set sensor list as OK with position_norm values
  sens mix ok <v> fail <v> ... Set mixed OK/FAILED readings

State / diagnostics
  state                        Print controller state name
  status                       Print full status block
  lat <fault_code>             Print PR004 latency (ms) for a fault code
  faults                       Print maintenance fault codes and recorded fault codes
  reset                        Force controller into RESET and clear latches (FTHR004 gate)
"""
    )


# cli.py
from app_context import AppContext

def run_rich_cli(ctx: AppContext) -> int:
    controller = ctx.controller
    altitude = ctx.altitude
    normal = ctx.normal
    power = ctx.power
    wow = ctx.wow
    sensors = ctx.sensors
    loop = ctx.loop

    annunciator = StateAnnunciator()

    # Attach annunicator to loop tick callback
    loop._on_tick = annunciator 
    # Announce state immediately
    annunciator(controller)

    _print_help()

    while not ctx.shutdown_event.is_set():
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            ctx.shutdown()
            break

        if not cmd:
            continue

        parts = cmd.split()
        op = parts[0].lower()

        if op in ("q", "quit", "exit"):
            ctx.shutdown()
            break

        if op in ("help", "?"):
            _print_help()
            continue

        if op == "run":
            if len(parts) >= 2:
                loop.set_period(float(parts[1]))
            loop.start()
            print(f"Loop running @ {loop.period_s:.3f}s")
            continue

        if op == "stop":
            loop.stop()
            print("Loop stopped")
            continue

        if op == "period":
            if len(parts) != 2:
                print("Usage: period <seconds>")
                continue
            loop.set_period(float(parts[1]))
            print(f"Loop period set to {loop.period_s:.3f}s")
            continue

        if op == "step":
            n = int(parts[1]) if len(parts) >= 2 else 1
            loop.step(n)
            print(f"Stepped {n} ticks")
            continue

        if op == "d":
            accepted = controller.command_gear_down(True)
            print(f"Deploy accepted: {accepted}")
            continue

        if op == "u":
            accepted = controller.command_gear_up(True)
            print(f"Retract accepted: {accepted}")
            continue

        if op == "wow":
            if len(parts) != 2 or parts[1] not in ("0", "1"):
                print("Usage: wow 0|1")
                continue
            wow.value = (parts[1] == "1")
            controller.set_weight_on_wheels(wow.value)
            print(f"WOW set to {wow.value}")
            continue

        if op == "alt":
            if len(parts) != 2:
                print("Usage: alt <feet>")
                continue
            altitude.value = float(parts[1])
            print(f"Altitude set to {altitude.value:.1f} ft")
            continue

        if op == "normal":
            if len(parts) != 2 or parts[1] not in ("0", "1"):
                print("Usage: normal 0|1")
                continue
            normal.value = (parts[1] == "1")
            print(f"Normal conditions set to {normal.value}")
            continue

        if op == "power":
            if len(parts) != 2 or parts[1] not in ("0", "1"):
                print("Usage: power 0|1")
                continue
            power.value = (parts[1] == "1")
            print(f"Primary power present set to {power.value}")
            continue

        if op == "sens":
            if len(parts) == 2 and parts[1].lower() == "show":
                print("Sensors:", ", ".join(_fmt_sensor(r) for r in sensors.get_readings()))
                continue

            if len(parts) < 3:
                print("Usage: sens ok <v> ok <v> ...  OR  sens mix ok <v> fail <v> ...")
                continue

            mode = parts[1].lower()
            tokens = parts[2:]

            new_readings: list[PositionSensorReading] = []

            def parse_pairs(tok: list[str]) -> list[tuple[str, float]]:
                out: list[tuple[str, float]] = []
                if len(tok) % 2 != 0:
                    raise ValueError("sensor tokens must be pairs")
                for i in range(0, len(tok), 2):
                    out.append((tok[i].lower(), float(tok[i + 1])))
                return out

            try:
                pairs = parse_pairs(tokens)
            except Exception as e:
                print(f"Invalid sensor input: {e}")
                continue

            if mode == "ok":
                for kind, v in pairs:
                    if kind != "ok":
                        print("Mode 'ok' only accepts 'ok <v>' pairs")
                        new_readings = []
                        break
                    new_readings.append(PositionSensorReading(SensorStatus.OK, v))

            elif mode == "mix":
                for kind, v in pairs:
                    if kind == "ok":
                        new_readings.append(PositionSensorReading(SensorStatus.OK, v))
                    elif kind in ("fail", "failed"):
                        new_readings.append(PositionSensorReading(SensorStatus.FAILED, v))
                    else:
                        print("Mode 'mix' accepts 'ok <v>' and 'fail <v>' pairs")
                        new_readings = []
                        break
            else:
                print("Usage: sens ok <v> ok <v> ...  OR  sens mix ok <v> fail <v> ...")
                continue

            if not new_readings:
                continue

            sensors.set_readings(new_readings)
            print("Sensors set:", ", ".join(_fmt_sensor(r) for r in sensors.get_readings()))
            continue

        if op == "state":
            print(controller.state.name)
            continue

        if op == "status":
            _print_status(controller, altitude, normal, power, wow, sensors)
            continue

        if op == "lat":
            if len(parts) != 2:
                print("Usage: lat <fault_code>")
                continue
            code = parts[1]
            if hasattr(controller, "fault_classification_latency_ms"):
                lat = controller.fault_classification_latency_ms(code)
                print(f"{code}: {lat} ms")
            else:
                print("PR004 latency API not present")
            continue

        if op == "faults":
            mf_codes = getattr(controller, "_maintenance_fault_codes", set())
            rec = getattr(controller, "_recorded_fault_codes", set())
            print("MaintenanceFaultCodes:", sorted(list(mf_codes)))
            print("RecordedFaultCodes:", sorted(list(rec)))
            continue

        if op == "reset":
            _reset_controller(controller)
            print("Controller reset to RESET state")
            continue

        print("Unknown command. Type 'help'.")

    loop.stop()
    return 0


if __name__ == "__main__":
    from main import initialize
    ctx = initialize()
    run_rich_cli(ctx)

