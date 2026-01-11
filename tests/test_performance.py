import pytest

from landing_gear_controller import LandingGearController
from gear_configuration import GearConfiguration
from sims.position_simulator import SensorStatus, PositionSensorReading


class FakeClock:
    def __init__(self, start: float = 0.0):
        self._t = float(start)

    def __call__(self) -> float:
        return self._t

    def advance(self, dt: float) -> None:
        self._t += float(dt)


def make_controller_with_fake_clock(
    *,
    pump_latency_ms: int = 0,
    actuator_speed_mm_per_100ms: float = 100.0,
    extension_distance_mm: int = 100,
    lock_time_ms: int = 0,
    requirement_time_ms: int = 8000,
):
    clock = FakeClock()

    config = GearConfiguration(
        name="TEST",
        pump_latency_ms=pump_latency_ms,
        actuator_speed_mm_per_100ms=actuator_speed_mm_per_100ms,
        extension_distance_mm=extension_distance_mm,
        lock_time_ms=lock_time_ms,
        requirement_time_ms=requirement_time_ms,
    )

    controller = LandingGearController(config=config, clock=clock)
    return controller, clock


def run_for(controller, clock: FakeClock, *, duration_s: float, step_s: float, on_tick=None):
    # Fixed-rate update loop used to simulate periodic scheduling in tests
    if step_s <= 0:
        raise ValueError("step_s must be > 0")

    steps = int(duration_s / step_s)
    for _ in range(steps):
        if on_tick is not None:
            on_tick(clock())
        controller.update()
        clock.advance(step_s)


def deltas(timestamps: list[float]) -> list[float]:
    return [b - a for a, b in zip(timestamps, timestamps[1:])]


class TestPR001:
    def test_pr001_latency_none_before_any_deploy(self):
        controller, _clock = make_controller_with_fake_clock()
        assert controller.deploy_actuation_latency_ms() is None

    def test_pr001_latency_boundary_exact_200ms(self):
        controller, clock = make_controller_with_fake_clock()

        assert controller.command_gear_down(True) is True
        clock.advance(0.200)
        controller.update()

        latency = controller.deploy_actuation_latency_ms()
        assert latency is not None
        assert latency <= 200.0

    def test_pr001_latency_invalid_over_200ms(self):
        controller, clock = make_controller_with_fake_clock()

        assert controller.command_gear_down(True) is True
        clock.advance(0.201)
        controller.update()

        latency = controller.deploy_actuation_latency_ms()
        assert latency is not None
        assert latency > 200.0

    def test_pr001_repeated_deploy_does_not_clear_latency(self):
        controller, clock = make_controller_with_fake_clock()

        assert controller.command_gear_down(True) is True
        clock.advance(0.050)
        controller.update()
        first = controller.deploy_actuation_latency_ms()
        assert first is not None

        controller.command_gear_down(True)
        clock.advance(0.050)
        controller.update()
        second = controller.deploy_actuation_latency_ms()
        assert second is not None


class TestPR002:
    def test_pr002_transition_updates_at_10hz_boundary(self):
        controller, clock = make_controller_with_fake_clock()

        assert controller.command_gear_down(True) is True
        controller.update()
        assert controller.state.name.startswith("TRANSITIONING")

        stamps: list[float] = []
        run_for(controller, clock, duration_s=1.0, step_s=0.1, on_tick=lambda t: stamps.append(t))

        ds = deltas(stamps)
        assert ds
        assert all(d <= 0.1 + 1e-9 for d in ds), f"Intervals: {ds}"

    def test_pr002_transition_invalid_slower_than_10hz(self):
        controller, clock = make_controller_with_fake_clock()

        assert controller.command_gear_down(True) is True
        controller.update()
        assert controller.state.name.startswith("TRANSITIONING")

        stamps: list[float] = []
        run_for(controller, clock, duration_s=1.0, step_s=0.11, on_tick=lambda t: stamps.append(t))

        ds = deltas(stamps)
        assert ds
        assert any(d > 0.1 for d in ds), f"Intervals: {ds}"

    def test_pr002_transition_invalid_jitter_includes_violations(self):
        controller, clock = make_controller_with_fake_clock()

        assert controller.command_gear_down(True) is True
        controller.update()
        assert controller.state.name.startswith("TRANSITIONING")

        stamps: list[float] = []
        steps = [0.08, 0.08, 0.12, 0.08, 0.12, 0.08]

        for dt in steps:
            stamps.append(clock())
            controller.update()
            clock.advance(dt)

        ds = deltas(stamps)
        assert ds
        assert any(d > 0.1 for d in ds), f"Intervals: {ds}"


class TestPR003:
    @pytest.mark.parametrize(
        "step_s, expected_pass",
        [
            (0.25, True),
            (0.249, True),
            (0.251, False),
            (0.10, True),
        ],
    )
    def test_pr003_steady_state_update_rate(self, step_s, expected_pass):
        controller, clock = make_controller_with_fake_clock()

        assert controller.state.name.startswith(("UP", "DOWN"))

        stamps: list[float] = []
        run_for(controller, clock, duration_s=2.0, step_s=step_s, on_tick=lambda t: stamps.append(t))

        ds = deltas(stamps)
        assert ds
        ok = all(d <= 0.25 + 1e-9 for d in ds)
        assert ok is expected_pass, f"step_s={step_s}, intervals={ds}"


class TestPR004:
    def test_pr004_no_conflict_no_classification(self):
        controller, clock = make_controller_with_fake_clock()

        readings = [
            PositionSensorReading(SensorStatus.OK, 0.0),
            PositionSensorReading(SensorStatus.OK, 0.0),
        ]
        controller.position_sensors_provider = lambda: readings

        controller.update()
        clock.advance(1.0)
        controller.update()

        latency_ms = controller.fault_classification_latency_ms("FTHR002_SENSOR_CONFLICT_PERSISTENT")
        assert latency_ms is None

    def test_pr004_conflict_clears_before_400ms_not_classified(self):
        controller, clock = make_controller_with_fake_clock()

        conflict = [
            PositionSensorReading(SensorStatus.OK, 0.0),
            PositionSensorReading(SensorStatus.OK, 1.0),
        ]
        ok = [
            PositionSensorReading(SensorStatus.OK, 0.0),
            PositionSensorReading(SensorStatus.OK, 0.0),
        ]

        controller.position_sensors_provider = lambda: conflict
        controller.update()

        clock.advance(0.30)
        controller.position_sensors_provider = lambda: ok
        controller.update()

        clock.advance(0.60)
        controller.update()

        latency_ms = controller.fault_classification_latency_ms("FTHR002_SENSOR_CONFLICT_PERSISTENT")
        assert latency_ms is None

    def test_pr004_conflict_persists_classified_within_400ms_boundary(self):
        controller, clock = make_controller_with_fake_clock()

        conflict = [
            PositionSensorReading(SensorStatus.OK, 0.0),
            PositionSensorReading(SensorStatus.OK, 1.0),
        ]
        controller.position_sensors_provider = lambda: conflict

        controller.update()
        clock.advance(0.40)
        controller.update()

        latency_ms = controller.fault_classification_latency_ms("FTHR002_SENSOR_CONFLICT_PERSISTENT")
        assert latency_ms is not None
        assert latency_ms <= 400.0

    def test_pr004_conflict_persists_invalid_just_over_400ms(self):
        controller, clock = make_controller_with_fake_clock()

        conflict = [
            PositionSensorReading(SensorStatus.OK, 0.0),
            PositionSensorReading(SensorStatus.OK, 1.0),
        ]
        controller.position_sensors_provider = lambda: conflict

        controller.update()
        clock.advance(0.401)
        controller.update()

        latency_ms = controller.fault_classification_latency_ms("FTHR002_SENSOR_CONFLICT_PERSISTENT")
        assert latency_ms is not None
        assert latency_ms > 400.0

    def test_pr004_failed_sensor_does_not_count_as_ok_ok_conflict(self):
        controller, clock = make_controller_with_fake_clock()

        readings = [
            PositionSensorReading(SensorStatus.FAILED, 0.0),
            PositionSensorReading(SensorStatus.OK, 1.0),
        ]
        controller.position_sensors_provider = lambda: readings

        controller.update()
        clock.advance(1.0)
        controller.update()

        latency_ms = controller.fault_classification_latency_ms("FTHR002_SENSOR_CONFLICT_PERSISTENT")
        assert latency_ms is None


class TestPositionNorm:
    @pytest.mark.parametrize("pos", [-0.1, -1.0, 1.1, 2.0])
    def test_position_norm_out_of_range_does_not_raise(self, pos):
        controller, _clock = make_controller_with_fake_clock()

        readings = [
            PositionSensorReading(SensorStatus.OK, pos),
            PositionSensorReading(SensorStatus.OK, 0.0),
        ]
        controller.position_sensors_provider = lambda: readings

        controller.update()

    @pytest.mark.parametrize("pos", [float("inf"), float("-inf"), float("nan")])
    def test_position_norm_non_finite_does_not_raise(self, pos):
        controller, _clock = make_controller_with_fake_clock()

        readings = [
            PositionSensorReading(SensorStatus.OK, pos),
            PositionSensorReading(SensorStatus.OK, 0.0),
        ]
        controller.position_sensors_provider = lambda: readings

        controller.update()

    def test_position_norm_endpoints_does_not_raise(self):
        controller, _clock = make_controller_with_fake_clock()

        controller.position_sensors_provider = lambda: [
            PositionSensorReading(SensorStatus.OK, 0.0),
            PositionSensorReading(SensorStatus.OK, 0.0),
        ]
        controller.update()

        controller.position_sensors_provider = lambda: [
            PositionSensorReading(SensorStatus.OK, 1.0),
            PositionSensorReading(SensorStatus.OK, 1.0),
        ]
        controller.update()
