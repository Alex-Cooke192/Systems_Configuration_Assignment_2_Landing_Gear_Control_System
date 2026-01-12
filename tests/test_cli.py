"""
Title: Landing Gear State Annunciation and Fault Handling Unit Tests
Author: Alex Cooke
Date Created: 2026-01-10
Last Modified: 2026-01-10
Version: 1.0

Purpose:
Provides unit-level verification of landing gear state annunciation and
command rejection behavior implemented in the LandingGearController and
CLI StateAnnunciator. These tests validate that required gear states are
annunciated correctly and that invalid retract commands are ignored in
abnormal or fault conditions.

Targeted Requirements (Verification Only):
- LGCS-FR003: Verification that TRANSITIONING_UP, TRANSITIONING_DOWN,
  UP_LOCKED, and DOWN_LOCKED states are annunciated to the CLI.
- LGCS-FR004: Verification that retract commands are ignored in FAULT
  and other abnormal states.

Scope and Limitations:
- Tests controller and CLI annunciation logic only.
- No real hardware, UI, or timing sources are used.
- Output verification is limited to CLI stdout behavior.
- Does not validate real-time scheduling or operator interface hardware.

Safety Notice:
This file is a test artefact intended solely for verification and assessment.
It must not be used in operational or flight-certified systems.

Dependencies:
- Python 3.10+
- pytest
- cli.py (StateAnnunciator)
- landing_gear_controller.py
- gear_configuration.py
- gear_states.py

Related Documents:
- LGCS Unit Test Plan
- LGCS Requirements Specification

Safety and Certification Disclaimer:
All artefacts in this repository are produced for academic assessment purposes only.
They do not represent certified software and must not be used in real-world aviation
or safety-critical systems.
"""

import pytest

from cli import StateAnnunciator
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


@pytest.fixture
def controller():
    clock = FakeClock()

    config = GearConfiguration(
        name="TEST",
        pump_latency_ms=0,
        actuator_speed_mm_per_100ms=100.0,
        extension_distance_mm=100,
        lock_time_ms=0,
        requirement_time_ms=8000,
    )

    return LandingGearController(config=config, clock=clock)


@pytest.mark.parametrize(
    "state",
    [
        GearState.TRANSITIONING_UP,
        GearState.TRANSITIONING_DOWN,
        GearState.UP_LOCKED,
        GearState.DOWN_LOCKED,
    ],
)
def test_fr003_state_annunciator_prints_required_states(controller, capsys, state):
    ann = StateAnnunciator()

    controller.enter_state(state)
    ann(controller)

    out = capsys.readouterr().out.strip().splitlines()
    assert out == [f"STATE: {state.name}"]


def test_fr003_state_annunciator_prints_only_on_change(controller, capsys):
    ann = StateAnnunciator()

    controller.enter_state(GearState.UP_LOCKED)
    ann(controller)
    ann(controller)

    out = capsys.readouterr().out.strip().splitlines()
    assert out == ["STATE: UP_LOCKED"]


def test_fr003_state_annunciator_prints_again_when_state_changes(controller, capsys):
    ann = StateAnnunciator()

    controller.enter_state(GearState.UP_LOCKED)
    ann(controller)

    controller.enter_state(GearState.TRANSITIONING_DOWN)
    ann(controller)

    controller.enter_state(GearState.DOWN_LOCKED)
    ann(controller)

    out = capsys.readouterr().out.strip().splitlines()
    assert out == [
        "STATE: UP_LOCKED",
        "STATE: TRANSITIONING_DOWN",
        "STATE: DOWN_LOCKED",
    ]


@pytest.mark.parametrize("abnormal_state", [GearState.FAULT, GearState.RESET])
def test_fr004_retract_ignored_in_fault_or_abnormal_states(controller, abnormal_state):
    controller.enter_state(abnormal_state)

    accepted = controller.command_gear_up(True)
    assert accepted is False
    assert controller.state == abnormal_state
