import pytest

from gear_configuration import GearConfiguration
from gear_states import GearState
from landing_gear_controller import LandingGearController  # <-- change to your module name


class FakeClock:
    """Deterministic clock you can manually advance."""
    def __init__(self, start: float = 0.0):
        self.t = start

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class SpyLandingGearController(LandingGearController):
    """Controller that captures actuator commands instead of printing."""
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


def test_command_gear_up_happy_path_transitions_and_actuator(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    c.enter_state(GearState.DOWN_LOCKED)

    ok = c.command_gear_up(True)
    assert ok is True
    assert c._state == GearState.TRANSITIONING_UP
    assert c.up_cmds[-1] is True

    ok = c.command_gear_up(False)
    assert ok is True
    assert c._state == GearState.UP_LOCKED
    assert c.up_cmds[-1] is False


def test_command_gear_up_rejected_if_not_down_locked(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    assert c._state == GearState.UP_LOCKED

    ok = c.command_gear_up(True)
    assert ok is False
    assert c._state == GearState.UP_LOCKED
    assert any("Retract rejected" in msg for msg in c.logs)


def test_update_completes_retract_after_configured_time(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    c.enter_state(GearState.DOWN_LOCKED)
    c._retract_requested = True

    # First tick starts retract
    c.update()
    assert c._state == GearState.TRANSITIONING_UP
    assert c.up_cmds[-1] is True

    # Not done yet
    clock.advance(c._deploy_time_s - 1e-6)
    c.update()
    assert c._state == GearState.TRANSITIONING_UP

    # Done
    clock.advance(1e-3)
    c.update()
    assert c._state == GearState.UP_LOCKED
    assert c.up_cmds[-1] is False


@pytest.mark.skipif(
    not hasattr(LandingGearController, "set_weight_on_wheels"),
    reason="WoW gating not implemented yet",
)
def test_fr002_blocks_retract_when_weight_on_wheels_true(config):
    # Used to test if weight-on0-wheels input works as expected
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    c.enter_state(GearState.DOWN_LOCKED)
    c.set_weight_on_wheels(True)  # on ground
    c._retract_requested = True

    c.update()

    # Must not start retract
    assert c._state == GearState.DOWN_LOCKED
    assert c.up_cmds == [] or c.up_cmds[-1] is False
