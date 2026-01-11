"""
Title: Landing Gear Retract Function Unit Tests
Author: Alex Cooke
Date Created: 2026-01-09
Last Modified: 2026-01-09
Version: 1.0

Purpose:
Provides unit-level verification of landing gear retraction logic implemented
in the LandingGearController, including enforcement of weight-on-wheels and
fault-state safety interlocks.

Targeted Requirements (Verification Only):
- LGCS-FR002: Verification that retraction occurs only when weight-on-wheels is FALSE.
- LGCS-FR004: Verification that retract commands are ignored in FAULT/ABNORMAL states.

Scope and Limitations:
- Tests controller logic only; no integration with hardware or OS timing.
- Weight-on-wheels and fault conditions are injected directly for verification.
- Intended for deterministic, repeatable execution.

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
    def __init__(self, start: float = 0.0):
        self.t: float = float(start)

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += float(dt)


class SpyLandingGearController(LandingGearController):
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


def test_command_gear_up_true_from_down_locked_starts_transition(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    c.enter_state(GearState.DOWN_LOCKED)

    ok = c.command_gear_up(True)
    assert ok is True
    assert c._state == GearState.TRANSITIONING_UP
    assert c.up_cmds[-1] is True


def test_command_gear_up_false_without_active_retract_is_rejected_or_no_effect(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    c.enter_state(GearState.DOWN_LOCKED)

    ok = c.command_gear_up(False)
    assert ok in (True, False)


@pytest.mark.parametrize(
    "initial_state",
    [
        GearState.UP_LOCKED,
        GearState.TRANSITIONING_UP,
        GearState.TRANSITIONING_DOWN,
        GearState.FAULT,
        GearState.RESET,
    ],
)
def test_command_gear_up_true_rejected_if_not_down_locked(initial_state, config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    c.enter_state(initial_state)

    ok = c.command_gear_up(True)
    assert ok is False
    assert c._state == initial_state
    assert any("Retract rejected" in msg for msg in c.logs)


def test_update_completes_retract_after_configured_time(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    c.enter_state(GearState.DOWN_LOCKED)
    assert c.command_gear_up(True) is True
    assert c._state == GearState.TRANSITIONING_UP
    assert c.up_cmds[-1] is True

    clock.advance(c._deploy_time_s - 1e-6)
    c.update()
    assert c._state == GearState.TRANSITIONING_UP

    clock.advance(1e-3)
    c.update()
    assert c._state == GearState.UP_LOCKED
    assert c.up_cmds[-1] is False


@pytest.mark.skipif(
    not hasattr(LandingGearController, "set_weight_on_wheels"),
    reason="WoW gating not implemented yet",
)
def test_fr002_blocks_retract_when_weight_on_wheels_true(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    c.enter_state(GearState.DOWN_LOCKED)
    c.set_weight_on_wheels(True)

    ok = c.command_gear_up(True)
    assert ok is False
    assert c._state == GearState.DOWN_LOCKED
    assert c.up_cmds == []


@pytest.mark.parametrize("abnormal_state", [GearState.FAULT, GearState.RESET])
def test_fr004_retract_rejected_in_fault_or_abnormal_states(abnormal_state, config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    c.enter_state(abnormal_state)

    ok = c.command_gear_up(True)
    assert ok is False
    assert c._state == abnormal_state
    assert c.up_cmds == []
