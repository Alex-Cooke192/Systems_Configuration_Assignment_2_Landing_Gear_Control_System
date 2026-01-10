from landing_gear_controller import LandingGearController
from gear_configuration import GearConfiguration
from fault_recorder import FaultRecorder
from gear_states import GearState
from sims.position_simulator import PositionSensorReading, SensorStatus

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

class TestFaultTolerance():
    def test_fthr001_single_sensor_failure_continues_using_remaining_sensors(self):
        # LGCS-FTHR001:
        # Confirm system computes a position estimate from remaining sensors and flags maintenance fault.

        controller, clock = make_controller_with_fake_clock()

        readings = [
            PositionSensorReading(SensorStatus.OK, 0.8),
            PositionSensorReading(SensorStatus.OK, 0.9),
            PositionSensorReading(SensorStatus.FAILED, 0.0),
        ]

        controller.position_sensors_provider = lambda: readings

        controller.update()

        assert controller._maintenance_fault_active is True
        assert "FTHR001_SINGLE_SENSOR_FAILURE" in controller._maintenance_fault_codes
        assert controller.position_estimate_norm is not None
        assert abs(controller.position_estimate_norm - 0.85) < 1e-6

    def test_fthr002_sensor_conflict_persisting_500ms_enters_fault(self):
        # LGCS-FTHR002:
        # Conflicting position sensor inputs persisting >500 ms cause FAULT and inhibit commands.

        controller, clock = make_controller_with_fake_clock()

        readings = [
            PositionSensorReading(SensorStatus.OK, 0.0),
            PositionSensorReading(SensorStatus.OK, 1.0),
        ]
        controller.position_sensors_provider = lambda: readings

        # First tick starts the persistence timer
        controller.update()
        assert controller.state != GearState.FAULT

        # Still conflicting but not long enough
        clock.advance(0.49)
        controller.update()
        assert controller.state != GearState.FAULT

        # Cross the 500 ms threshold
        clock.advance(0.02)
        controller.update()
        assert controller.state == GearState.FAULT

        # Confirm commands are inhibited in FAULT
        assert controller.command_gear_down(True) is False
        assert controller.command_gear_up(True) is False

    def test_fthr003_records_fault_with_timestamp_and_code(self, tmp_path):
        # LGCS-FTHR003:
        # Confirm detected fault is recorded with timestamp and fault code in non-volatile storage.

        controller, clock = make_controller_with_fake_clock()

        log_path = tmp_path / "fault_log.txt"
        controller.fault_recorder = FaultRecorder(filepath=log_path, clock=clock)

        readings = [
            PositionSensorReading(SensorStatus.OK, 0.8),
            PositionSensorReading(SensorStatus.OK, 0.9),
            PositionSensorReading(SensorStatus.FAILED, 0.0),
        ]
        controller.position_sensors_provider = lambda: readings

        clock.advance(1.234)
        controller.update()

        text = log_path.read_text(encoding="utf-8")
        assert "FTHR001_SINGLE_SENSOR_FAILURE" in text

        # Confirms a timestamp-like prefix exists before the comma.
        first_line = text.strip().splitlines()[0]
        ts_str, code = first_line.split(",", maxsplit=1)
        assert float(ts_str) > 0.0
        assert code == "FTHR001_SINGLE_SENSOR_FAILURE"

    def test_fthr003_does_not_duplicate_same_fault_code(tmp_path):
        # Ensures each fult is only logged once, even if problem persists (no continuous logging)
        controller, clock = make_controller_with_fake_clock()

        log_path = tmp_path / "fault_log.txt"
        controller.fault_recorder = FaultRecorder(filepath=log_path, clock=clock)

        readings = [
            PositionSensorReading(SensorStatus.OK, 0.8),
            PositionSensorReading(SensorStatus.OK, 0.9),
            PositionSensorReading(SensorStatus.FAILED, 0.0),
        ]
        controller.position_sensors_provider = lambda: readings

        clock.advance(1.0)
        controller.update()

        clock.advance(1.0)
        controller.update()

        lines = log_path.read_text(encoding="utf-8").strip().splitlines()
        assert len(lines) == 1

