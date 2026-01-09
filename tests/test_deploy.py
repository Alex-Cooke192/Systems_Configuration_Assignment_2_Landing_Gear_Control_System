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


def test_command_gear_down_happy_path_transitions_and_actuator(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    assert c._state == GearState.UP_LOCKED

    ok = c.command_gear_down(True)
    assert ok is True
    assert c._state == GearState.TRANSITIONING_DOWN
    assert c.down_cmds[-1] is True

    ok = c.command_gear_down(False)
    assert ok is True
    assert c._state == GearState.DOWN_LOCKED
    assert c.down_cmds[-1] is False


def test_command_gear_down_rejected_if_not_up_locked(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    c.enter_state(GearState.DOWN_LOCKED)

    ok = c.command_gear_down(True)
    assert ok is False
    assert c._state == GearState.DOWN_LOCKED
    assert any("Deploy rejected" in msg for msg in c.logs)


def test_update_completes_deploy_after_configured_time(config):
    clock = FakeClock()
    c = SpyLandingGearController(config=config, clock=clock)

    # Trigger deploy via request flag (current design)
    c._deploy_requested = True

    # First tick starts deploy
    c.update()
    assert c._state == GearState.TRANSITIONING_DOWN
    assert c.down_cmds[-1] is True

    # Not done yet
    clock.advance(c._deploy_time_s - 1e-6)
    c.update()
    assert c._state == GearState.TRANSITIONING_DOWN

    # Done
    clock.advance(1e-3)
    c.update()
    assert c._state == GearState.DOWN_LOCKED
    assert c.down_cmds[-1] is False
