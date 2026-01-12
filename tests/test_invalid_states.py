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
