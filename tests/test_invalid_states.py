"""
Title: Landing Gear Retract Command Rejection Unit Tests
Author: Alex Cooke
Date Created: 2026-01-10
Last Modified: 2026-01-10
Version: 1.0

Purpose:
Provides unit-level verification that retract (gear-up) commands are
correctly rejected in FAULT, ABNORMAL, and RESET states. These tests
validate that no actuator energisation occurs, no state transitions
are triggered, and repeated or release commands are safely ignored.

Targeted Requirements (Verification Only):
- LGCS-FR004: Verification that retract commands are ignored when the
  system is in FAULT, ABNORMAL, or RESET states.

Scope and Limitations:
- Tests command-handling and actuator-inhibition logic only.
- Uses a spy controller to observe actuator command attempts.
- No real hardware, sensors, or real-time scheduling are involved.
- Does not validate deploy (gear-down) command behavior.

Safety Notice:
This file is a test artefact intended solely for verification and assessment.
It must not be used in operational or flight-certified systems.

Dependencies:
- Python 3.10+
- pytest
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

from gear_configuration import GearConfiguration
from gear_states import GearState
from landing_gear_controller import LandingGearController


class FakeClock:
    def __init__(self, start: float = 0.0):
        self.t = float(start)

    def __call__(self) -> float:
        return self.t


class SpyLandingGearController(LandingGearController):
    def __init__(self, config: GearConfiguration, clock):
        super().__init__(config=config, clock=clock)
        self.up_cmds: list[bool] = []
        self.down_cmds: list[bool] = []
        self.logs: list[str] = []

    def log(self, msg: str) -> None:
        self.logs.append(msg)

    def _actuate_up(self, enabled: bool) -> None:
        self.up_cmds.append(enabled)
        super()._actuate_up(enabled)

    def _actuate_down(self, enabled: bool) -> None:
        self.down_cmds.append(enabled)
        super()._actuate_down(enabled)


@pytest.fixture
def controller():
    clock = FakeClock()
    cfg = GearConfiguration(
        name="TEST",
        pump_latency_ms=0,
        actuator_speed_mm_per_100ms=100.0,
        extension_distance_mm=100,
        lock_time_ms=0,
        requirement_time_ms=8000,
    )
    return SpyLandingGearController(config=cfg, clock=clock)


ABNORMAL_STATES = [GearState.FAULT, GearState.ABNORMAL]


@pytest.mark.parametrize("state", ABNORMAL_STATES)
def test_fr004_retract_command_rejected_in_fault_or_abnormal(controller, state):
    controller.enter_state(state)

    pre_state = controller.state
    pre_up_cmds = list(controller.up_cmds)

    accepted = controller.command_gear_up(True)

    assert accepted is False
    assert controller.state == pre_state
    assert controller.up_cmds == pre_up_cmds


@pytest.mark.parametrize("state", ABNORMAL_STATES)
def test_fr004_retract_spam_always_rejected(controller, state):
    controller.enter_state(state)

    pre_state = controller.state

    for _ in range(10):
        accepted = controller.command_gear_up(True)
        assert accepted is False
        assert controller.state == pre_state


@pytest.mark.parametrize("state", ABNORMAL_STATES)
def test_fr004_retract_release_does_not_energize_actuator(controller, state):
    controller.enter_state(state)

    pre_state = controller.state
    controller.command_gear_up(False)

    assert controller.state == pre_state
    assert True not in controller.up_cmds


def test_reset_rejects_retract_command(controller):
    controller.enter_state(GearState.RESET)

    accepted = controller.command_gear_up(True)
    assert accepted is False
    assert controller.state == GearState.RESET
