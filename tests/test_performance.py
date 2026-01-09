from landing_gear_controller import LandingGearController
from gear_configuration import GearConfiguration


class FakeClock:
    def __init__(self, start: float = 0.0):
        self._t = start

    def __call__(self) -> float:
        return self._t

    def advance(self, dt: float) -> None:
        self._t += dt


def make_controller_with_fake_clock():
    clock = FakeClock()

    config = GearConfiguration(
        name="TEST",
        pump_latency_ms=0,
        actuator_speed_mm_per_100ms=100.0,
        extension_distance_mm=100,
        lock_time_ms=0,
        requirement_time_ms=8000,
    )

    controller = LandingGearController(config=config, clock=clock)
    return controller, clock


class TestPerformance:
    def test_pr001_deploy_actuates_within_200ms(self):
        controller, clock = make_controller_with_fake_clock()

        controller.command_gear_down(True)
        controller.update()

        latency = controller.deploy_actuation_latency_ms()
        assert latency is not None
        assert latency <= 200.0
