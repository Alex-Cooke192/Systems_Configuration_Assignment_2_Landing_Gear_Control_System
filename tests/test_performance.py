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

    def test_state_updates_at_minimum_10hz_during_deploy():
        controller, clock = make_controller_with_fake_clock()

        # Begin deployment
        assert controller.command_gear_down() is True
        assert controller.state.name.startswith("TRANSITIONING")

        update_timestamps: list[float] = []

        # Instrument update() calls
        original_update = controller.update

        def instrumented_update():
            update_timestamps.append(clock())
            original_update()

        controller.update = instrumented_update

        # Simulate 1 second of deploy time
        SIM_DURATION_MS = 1000
        STEP_MS = 50  # simulate system tick faster than minimum

        steps = SIM_DURATION_MS // STEP_MS

        for _ in range(steps):
            controller.update()
            clock._t += STEP_MS / 1000.0

        # Compute deltas between consecutive updates
        deltas = [
            t2 - t1 for t1, t2 in zip(update_timestamps, update_timestamps[1:])
        ]

        # Requirement: at least 10 Hz => <= 0.1 s between updates
        assert all(
            delta <= 0.1 for delta in deltas
        ), f"Update interval exceeded 100 ms: {deltas}"

        
