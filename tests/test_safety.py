"""
Title: Landing Gear Safety Requirements Unit Tests
Author: Alex Cooke
Date Created: 2026-01-10
Last Modified: 2026-01-10
Version: 1.0

Purpose:
Provides unit-level verification of safety-related requirements implemented
in the LandingGearController. These tests validate automatic gear deployment
based on altitude, low-altitude warning annunciation, weight-on-wheels
inhibition logic, and safe behavior during primary power loss and restoration.

Targeted Requirements (Verification Only):
- LGCS-SR001: Verification of automatic landing gear deployment below
  defined altitude thresholds under normal operating conditions.
- LGCS-SR002: Verification of low-altitude warning annunciation when the
  landing gear is not deployed.
- LGCS-SR003: Verification of retract inhibition when weight-on-wheels
  is TRUE and correct retract behavior when conditions permit.
- LGCS-SR004: Verification of safe landing gear behavior during primary
  power loss and recovery.

Scope and Limitations:
- Tests safety logic only; normal and fault-handling logic is exercised
  only where required by safety requirements.
- Uses simulated altitude, power, and weight-on-wheels inputs.
- No real sensors, electrical power sources, or flight hardware are used.
- Does not validate cockpit UI hardware or human factors.

Safety Notice:
This file is a test artefact intended solely for verification and assessment.
It must not be used in operational or flight-certified systems.

Dependencies:
- Python 3.10+
- pytest
- landing_gear_controller.py
- gear_configuration.py
- gear_states.py
- sims.altitude_simulator.py

Related Documents:
- LGCS Unit Test Plan
- LGCS Safety Requirements Specification

Safety and Certification Disclaimer:
All artefacts in this repository are produced for academic assessment purposes only.
They do not represent certified software and must not be used in real-world aviation
or safety-critical systems.
"""

import random
import pytest

from landing_gear_controller import LandingGearController
from gear_configuration import GearConfiguration
from gear_states import GearState
from sims.altitude_simulator import AltitudeSimulator


class FakeClock:
    def __init__(self, start: float = 0.0):
        self._t = float(start)

    def __call__(self) -> float:
        return self._t

    def advance(self, dt: float) -> None:
        self._t += float(dt)


def make_controller_with_fake_clock(
    *,
    normal_conditions: bool = True,
    min_alt: float = 500.0,
    max_alt: float = 10_000.0,
    rng_seed: int = 0,
):
    clock = FakeClock()

    config = GearConfiguration(
        name="TEST",
        pump_latency_ms=0,
        actuator_speed_mm_per_100ms=100.0,
        extension_distance_mm=100,
        lock_time_ms=0,
        requirement_time_ms=8000,
    )

    sim = AltitudeSimulator(
        clock=clock,
        rng=random.Random(rng_seed),
        min_alt=min_alt,
        max_alt=max_alt,
    )

    controller = LandingGearController(
        config=config,
        clock=clock,
        altitude_provider=sim.read_altitude_ft,
        normal_conditions_provider=lambda: normal_conditions,
    )

    return controller, sim, clock


class TestSafety:
    # -----------------------------
    # LGCS-SR001
    # -----------------------------

    def test_sr001_no_auto_deploy_when_altitude_above_1000ft(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=True)

        controller.enter_state(GearState.UP_LOCKED)

        sim.set_altitude_ft(1000.1)
        controller.update()

        assert controller.state == GearState.UP_LOCKED

    def test_sr001_edge_at_exactly_1000ft(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=True)

        controller.enter_state(GearState.UP_LOCKED)

        sim.set_altitude_ft(1000.0)
        controller.update()

        # Threshold behavior definition: deploy on <1000.0 or <=1000.0
        # This test asserts "no deploy at exactly 1000.0"
        assert controller.state == GearState.UP_LOCKED

    def test_sr001_auto_deploy_when_altitude_below_1000ft(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=True)

        controller.enter_state(GearState.UP_LOCKED)

        sim.set_altitude_ft(999.0)
        controller.update()

        assert controller.state in (GearState.TRANSITIONING_DOWN, GearState.DOWN_LOCKED)

    def test_sr001_no_auto_deploy_when_not_normal_conditions(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=False)

        controller.enter_state(GearState.UP_LOCKED)

        sim.set_altitude_ft(999.0)
        controller.update()

        assert controller.state == GearState.UP_LOCKED

    def test_sr001_no_auto_deploy_when_already_down_locked(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=True)

        controller.enter_state(GearState.DOWN_LOCKED)

        sim.set_altitude_ft(999.0)
        controller.update()

        assert controller.state == GearState.DOWN_LOCKED

    def test_sr001_no_auto_deploy_when_already_transitioning_down(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=True)

        controller.enter_state(GearState.TRANSITIONING_DOWN)

        sim.set_altitude_ft(999.0)
        controller.update()

        assert controller.state == GearState.TRANSITIONING_DOWN

    def test_sr001_invalid_altitude_nan_does_not_trigger_deploy(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=True)

        controller.enter_state(GearState.UP_LOCKED)

        sim.set_altitude_ft(float("nan"))
        controller.update()

        assert controller.state == GearState.UP_LOCKED

    def test_sr001_invalid_altitude_negative_does_trigger_deploy_if_treated_as_low(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=True)

        controller.enter_state(GearState.UP_LOCKED)

        sim.set_altitude_ft(-1.0)
        controller.update()

        assert controller.state in (GearState.TRANSITIONING_DOWN, GearState.DOWN_LOCKED)

    # -----------------------------
    # LGCS-SR002
    # -----------------------------

    def test_sr002_no_warning_when_altitude_above_2000ft(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=True)

        controller.enter_state(GearState.UP_LOCKED)

        messages: list[str] = []
        controller.log = lambda msg: messages.append(msg)

        sim.set_altitude_ft(2000.1)
        controller.update()

        assert not any("WARNING: ALTITUDE LOW - LANDING GEAR NOT DEPLOYED" in m for m in messages)

    def test_sr002_edge_at_exactly_2000ft(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=True)

        controller.enter_state(GearState.UP_LOCKED)

        messages: list[str] = []
        controller.log = lambda msg: messages.append(msg)

        sim.set_altitude_ft(2000.0)
        controller.update()

        # Threshold behavior definition: warn on <2000.0 or <=2000.0
        # This test asserts "no warning at exactly 2000.0"
        assert not any("WARNING: ALTITUDE LOW - LANDING GEAR NOT DEPLOYED" in m for m in messages)

    def test_sr002_warning_when_below_2000ft_and_gear_not_down(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=True)

        controller.enter_state(GearState.UP_LOCKED)

        messages: list[str] = []
        controller.log = lambda msg: messages.append(msg)

        sim.set_altitude_ft(1999.0)
        controller.update()

        assert any("WARNING: ALTITUDE LOW - LANDING GEAR NOT DEPLOYED" in m for m in messages)

    def test_sr002_no_warning_when_gear_is_down_locked(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=True)

        controller.enter_state(GearState.DOWN_LOCKED)

        messages: list[str] = []
        controller.log = lambda msg: messages.append(msg)

        sim.set_altitude_ft(1999.0)
        controller.update()

        assert not any("WARNING: ALTITUDE LOW - LANDING GEAR NOT DEPLOYED" in m for m in messages)

    def test_sr002_no_warning_when_not_normal_conditions(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=False)

        controller.enter_state(GearState.UP_LOCKED)

        messages: list[str] = []
        controller.log = lambda msg: messages.append(msg)

        sim.set_altitude_ft(1999.0)
        controller.update()

        assert not any("WARNING: ALTITUDE LOW - LANDING GEAR NOT DEPLOYED" in m for m in messages)

    def test_sr002_invalid_altitude_nan_no_warning(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=True)

        controller.enter_state(GearState.UP_LOCKED)

        messages: list[str] = []
        controller.log = lambda msg: messages.append(msg)

        sim.set_altitude_ft(float("nan"))
        controller.update()

        assert not any("WARNING: ALTITUDE LOW - LANDING GEAR NOT DEPLOYED" in m for m in messages)

    # -----------------------------
    # LGCS-SR003
    # -----------------------------

    def test_sr003_inhibits_retract_when_weight_on_wheels_true(self):
        controller, sim, clock = make_controller_with_fake_clock()

        controller.enter_state(GearState.DOWN_LOCKED)
        controller.set_weight_on_wheels(True)

        accepted = controller.command_gear_up(True)
        assert accepted is False
        assert controller.state == GearState.DOWN_LOCKED

    def test_sr003_allows_retract_when_weight_on_wheels_false_and_power_present(self):
        controller, sim, clock = make_controller_with_fake_clock()

        controller.enter_state(GearState.DOWN_LOCKED)
        controller.set_weight_on_wheels(False)

        accepted = controller.command_gear_up(True)
        assert accepted is True

        # If accepted, state should move out of DOWN_LOCKED on subsequent update
        if accepted:
            controller.update()
            assert controller.state != GearState.DOWN_LOCKED

    def test_sr003_retract_rejected_when_not_down_locked(self):
        controller, sim, clock = make_controller_with_fake_clock()

        controller.enter_state(GearState.UP_LOCKED)
        controller.set_weight_on_wheels(False)

        accepted = controller.command_gear_up(True)
        assert accepted is False
        assert controller.state == GearState.UP_LOCKED

    # -----------------------------
    # LGCS-SR004
    # -----------------------------

    def test_sr004_power_loss_forces_deploy_from_up_locked(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=True)

        power_present = True

        def power_provider():
            return power_present

        controller.primary_power_present_provider = power_provider

        controller.enter_state(GearState.UP_LOCKED)
        power_present = False

        controller.update()
        assert controller.state in (GearState.TRANSITIONING_DOWN, GearState.DOWN_LOCKED)

    def test_sr004_pilot_retract_ignored_when_power_not_present(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=True)

        power_present = False

        def power_provider():
            return power_present

        controller.primary_power_present_provider = power_provider

        controller.enter_state(GearState.DOWN_LOCKED)
        controller.set_weight_on_wheels(False)

        accepted = controller.command_gear_up(True)
        assert accepted is False
        assert controller.state == GearState.DOWN_LOCKED

    def test_sr004_power_loss_no_exception_if_provider_missing(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=True)

        controller.enter_state(GearState.UP_LOCKED)

        # primary_power_present_provider may be None or unset depending on implementation
        controller.update()

        assert controller.state in (
            GearState.UP_LOCKED,
            GearState.TRANSITIONING_DOWN,
            GearState.DOWN_LOCKED,
        )

    def test_sr004_power_restored_allows_pilot_retract_when_wow_false(self):
        controller, sim, clock = make_controller_with_fake_clock(normal_conditions=True)

        power_present = False

        def power_provider():
            return power_present

        controller.primary_power_present_provider = power_provider

        controller.enter_state(GearState.DOWN_LOCKED)
        controller.set_weight_on_wheels(False)

        accepted_off = controller.command_gear_up(True)
        assert accepted_off is False

        power_present = True

        accepted_on = controller.command_gear_up(True)
        assert accepted_on is True
