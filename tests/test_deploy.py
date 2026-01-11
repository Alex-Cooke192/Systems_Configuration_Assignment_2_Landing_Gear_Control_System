"""
Title: Landing Gear Deploy Function Unit Tests
Author: Alex Cooke
Date Created: 2026-01-09
Last Modified: 2026-01-09
Version: 1.0

Purpose:
Provides unit-level verification of landing gear deployment logic implemented
in the LandingGearController. These tests validate correct state transitions,
actuator commands, and timing behavior under nominal and rejection conditions.

Targeted Requirements (Verification Only):
- LGCS-FR001: Verification of UP-to-DOWN transition behavior and timing.
- LGCS-FR004: Verification that deploy commands are ignored in FAULT/ABNORMAL states.

Scope and Limitations:
- Tests controller logic only; no real hardware or timing sources are used.
- Uses a deterministic fake clock for repeatable execution.
- Does not validate real-time performance or physical actuator behavior.

Safety Notice:
This file is a test artefact intended solely for verification and assessment.
It must not be used in operational or flight-certified systems.

Dependencies:
- Python 3.10+
- pytest
- landing_gear_controller.py
- gear_configuration.py

Related Documents:
- LGCS Unit Test Plan
- LGCS Requirements Specification

Safety and Certification Disclaimer:
All artefacts in this repository are produced for academic assessment purposes only.
They do not represent certified software and must not be used in real-world aviation
or safety-critical systems.
"""

import pytest

from gear_configuration import GearConfiguration
from gear_states import GearState
from landing_gear_controller import LandingGearController


class FakeClock:
    # Deterministic clock manually advanced by tests
    def __init__(self, start: float = 0.0):
        self.t: float = float(start)

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += float(dt)


class SpyLandingGearController(LandingGearController):
    # Controller capturing actuator command outputs and log lines
    def __init__(self, config: GearConfiguration, clock):
        super().__init__(config=config, clock=clock)
        self.down_cmds: list[bool] = []
        self.up_cmds: list[bool] = []
        self.logs: list[str] = []

    def log(self, msg: str) -> None:
        self.logs.append(msg)

    def _actuate_down(self, enabled: bool) -> None:
        self.down_cmds.append(enabled)

    def _actuate_up(self, enabled: bool) -> None:
        self.up_cmds.append(enabled)


@pytest.fixture
def config() -> GearConfiguration:
    return GearConfiguration(
        name="TEST",
        pump_latency_ms=100,
        actuator_speed_mm_per_100ms=100.0,
        extension_distance_mm=100,
        lock_time_ms=100,
        requirement_time_ms=8000,
    )


# -----------------------------
# LGCS-FR001 deploy command behavior
# -----------------------------

def test_command_gear_down_happy_path_enters_transition_and_actuates(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    assert c._state == GearState.UP_LOCKED

    ok = c.command_gear_down()
    assert ok is True
    assert c._state == GearState.TRANSITIONING_DOWN
    assert c.down_cmds[-1] is True


@pytest.mark.parametrize(
    "initial_state",
    [
        GearState.DOWN_LOCKED,
        GearState.TRANSITIONING_DOWN,
        GearState.TRANSITIONING_UP,
        GearState.FAULT,
        GearState.RESET,
    ],
)
def test_command_gear_down_rejected_if_not_up_locked(initial_state, config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    c.enter_state(initial_state)

    ok = c.command_gear_down()
    assert ok is False
    assert c._state == initial_state
    assert any("Deploy rejected" in msg for msg in c.logs)


def test_command_gear_down_idempotent_rejected_when_already_transitioning(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    assert c.command_gear_down() is True
    assert c._state == GearState.TRANSITIONING_DOWN

    ok2 = c.command_gear_down()
    assert ok2 is False
    assert c._state == GearState.TRANSITIONING_DOWN


def test_update_does_not_start_deploy_without_request(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    assert c._state == GearState.UP_LOCKED

    c.update()
    assert c._state == GearState.UP_LOCKED
    assert c.down_cmds == []


# -----------------------------
# LGCS-FR001 timing behavior
# -----------------------------

def test_update_completes_deploy_after_configured_time(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    assert c.command_gear_down() is True
    assert c._state == GearState.TRANSITIONING_DOWN
    assert c.down_cmds[-1] is True

    clock.advance(c._deploy_time_s - 1e-6)
    c.update()
    assert c._state == GearState.TRANSITIONING_DOWN
    assert c.down_cmds[-1] is True

    clock.advance(1e-3)
    c.update()
    assert c._state == GearState.DOWN_LOCKED
    assert c.down_cmds[-1] is False


def test_update_deploy_edge_exactly_at_deploy_time(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    assert c.command_gear_down() is True
    assert c._state == GearState.TRANSITIONING_DOWN

    clock.advance(c._deploy_time_s)
    c.update()

    assert c._state == GearState.DOWN_LOCKED
    assert c.down_cmds[-1] is False


def test_update_deploy_large_time_step_completes_in_one_tick(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    assert c.command_gear_down() is True
    assert c._state == GearState.TRANSITIONING_DOWN

    clock.advance(c._deploy_time_s * 10.0)
    c.update()

    assert c._state == GearState.DOWN_LOCKED
    assert c.down_cmds[-1] is False


def test_update_deploy_time_goes_backward_does_not_complete(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    assert c.command_gear_down() is True
    assert c._state == GearState.TRANSITIONING_DOWN

    clock.advance(-1.0)
    c.update()

    assert c._state == GearState.TRANSITIONING_DOWN


def test_update_during_transition_keeps_actuator_enabled_until_complete(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    assert c.command_gear_down() is True
    assert c._state == GearState.TRANSITIONING_DOWN
    assert c.down_cmds[-1] is True

    clock.advance(c._deploy_time_s / 2.0)
    c.update()

    assert c._state == GearState.TRANSITIONING_DOWN
    assert c.down_cmds[-1] is True


# -----------------------------
# LGCS-FR004 deploy ignored in FAULT / abnormal states
# -----------------------------

@pytest.mark.parametrize("abnormal_state", [GearState.FAULT, GearState.RESET])
def test_deploy_rejected_in_abnormal_states(abnormal_state, config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    c.enter_state(abnormal_state)

    ok = c.command_gear_down()
    assert ok is False
    assert c._state == abnormal_state
    assert c.down_cmds == []
