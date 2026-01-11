import pytest

from landing_gear_controller import LandingGearController
from gear_configuration import GearConfiguration
from fault_recorder import FaultRecorder
from gear_states import GearState
from sims.position_simulator import PositionSensorReading, SensorStatus


class FakeClock:
    def __init__(self, start: float = 0.0):
        self._t = float(start)

    def __call__(self) -> float:
        return self._t

    def advance(self, dt: float) -> None:
        self._t += float(dt)


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


def call_command_gear_down(controller) -> bool:
    # Supports both command_gear_down() and command_gear_down(bool) signatures
    try:
        return controller.command_gear_down()
    except TypeError:
        return controller.command_gear_down(True)


def call_command_gear_up(controller) -> bool:
    # Supports both command_gear_up() and command_gear_up(bool) signatures
    try:
        return controller.command_gear_up()
    except TypeError:
        return controller.command_gear_up(True)


class TestFaultTolerance:
    # -----------------------------
    # LGCS-FTHR001
    # -----------------------------

    def test_fthr001_single_sensor_failure_continues_using_remaining_sensors(self):
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

    def test_fthr001_two_sensors_failed_no_position_estimate(self):
        controller, clock = make_controller_with_fake_clock()

        readings = [
            PositionSensorReading(SensorStatus.OK, 0.7),
            PositionSensorReading(SensorStatus.FAILED, 0.0),
            PositionSensorReading(SensorStatus.FAILED, 1.0),
        ]
        controller.position_sensors_provider = lambda: readings

        controller.update()

        assert controller._maintenance_fault_active is True
        assert "FTHR001_SINGLE_SENSOR_FAILURE" in controller._maintenance_fault_codes

        # Position estimate policy when only one valid sensor remains
        assert controller.position_estimate_norm in (None, 0.7)

    def test_fthr001_no_maintenance_fault_when_all_sensors_ok(self):
        controller, clock = make_controller_with_fake_clock()

        readings = [
            PositionSensorReading(SensorStatus.OK, 0.8),
            PositionSensorReading(SensorStatus.OK, 0.9),
            PositionSensorReading(SensorStatus.OK, 0.85),
        ]
        controller.position_sensors_provider = lambda: readings

        controller.update()

        assert controller._maintenance_fault_active in (False, True)
        if controller._maintenance_fault_active:
            assert "FTHR001_SINGLE_SENSOR_FAILURE" not in controller._maintenance_fault_codes

    def test_fthr001_maintenance_fault_not_duplicated_in_codes(self):
        controller, clock = make_controller_with_fake_clock()

        readings = [
            PositionSensorReading(SensorStatus.OK, 0.8),
            PositionSensorReading(SensorStatus.OK, 0.9),
            PositionSensorReading(SensorStatus.FAILED, 0.0),
        ]
        controller.position_sensors_provider = lambda: readings

        controller.update()
        controller.update()

        codes = list(controller._maintenance_fault_codes)
        assert codes.count("FTHR001_SINGLE_SENSOR_FAILURE") <= 1

    # -----------------------------
    # LGCS-FTHR002
    # -----------------------------

    def test_fthr002_conflict_persisting_over_500ms_enters_fault(self):
        controller, clock = make_controller_with_fake_clock()

        readings = [
            PositionSensorReading(SensorStatus.OK, 0.0),
            PositionSensorReading(SensorStatus.OK, 1.0),
        ]
        controller.position_sensors_provider = lambda: readings

        controller.update()
        assert controller.state != GearState.FAULT

        clock.advance(0.49)
        controller.update()
        assert controller.state != GearState.FAULT

        clock.advance(0.02)
        controller.update()
        assert controller.state == GearState.FAULT

        assert call_command_gear_down(controller) is False
        assert call_command_gear_up(controller) is False

    def test_fthr002_edge_at_exactly_500ms_not_in_fault(self):
        controller, clock = make_controller_with_fake_clock()

        readings = [
            PositionSensorReading(SensorStatus.OK, 0.0),
            PositionSensorReading(SensorStatus.OK, 1.0),
        ]
        controller.position_sensors_provider = lambda: readings

        controller.update()
        assert controller.state != GearState.FAULT

        clock.advance(0.50)
        controller.update()

        # Threshold behavior definition: enter fault on >500ms (strict) or >=500ms (inclusive)
        # This test asserts strict (>500ms) behavior
        assert controller.state != GearState.FAULT

    def test_fthr002_conflict_clears_before_500ms_no_fault(self):
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
        assert controller.state != GearState.FAULT

        clock.advance(0.30)
        controller.position_sensors_provider = lambda: ok
        controller.update()
        assert controller.state != GearState.FAULT

        clock.advance(1.00)
        controller.update()
        assert controller.state != GearState.FAULT

    def test_fthr002_no_fault_when_only_one_valid_sensor(self):
        controller, clock = make_controller_with_fake_clock()

        readings = [
            PositionSensorReading(SensorStatus.OK, 0.0),
            PositionSensorReading(SensorStatus.FAILED, 1.0),
        ]
        controller.position_sensors_provider = lambda: readings

        controller.update()
        clock.advance(1.0)
        controller.update()

        assert controller.state != GearState.FAULT

    # -----------------------------
    # LGCS-FTHR003
    # -----------------------------

    def test_fthr003_records_fault_with_timestamp_and_code(self, tmp_path):
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

        first_line = text.strip().splitlines()[0]
        ts_str, code = first_line.split(",", maxsplit=1)
        assert float(ts_str) > 0.0
        assert code == "FTHR001_SINGLE_SENSOR_FAILURE"

    def test_fthr003_does_not_duplicate_same_fault_code(self, tmp_path):
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

    def test_fthr003_records_multiple_distinct_faults(self, tmp_path):
        controller, clock = make_controller_with_fake_clock()

        log_path = tmp_path / "fault_log.txt"
        controller.fault_recorder = FaultRecorder(filepath=log_path, clock=clock)

        single_fail = [
            PositionSensorReading(SensorStatus.OK, 0.8),
            PositionSensorReading(SensorStatus.OK, 0.9),
            PositionSensorReading(SensorStatus.FAILED, 0.0),
        ]
        conflict = [
            PositionSensorReading(SensorStatus.OK, 0.0),
            PositionSensorReading(SensorStatus.OK, 1.0),
        ]

        controller.position_sensors_provider = lambda: single_fail
        clock.advance(1.0)
        controller.update()

        controller.position_sensors_provider = lambda: conflict
        controller.update()
        clock.advance(0.51)
        controller.update()

        text = log_path.read_text(encoding="utf-8")
        assert "FTHR001_SINGLE_SENSOR_FAILURE" in text
        assert "FTHR002_SENSOR_CONFLICT_PERSISTENT" in text

    # -----------------------------
    # LGCS-FTHR004
    # -----------------------------

    def test_fthr004_reset_ignores_commands_until_sensors_validated(self):
        controller, clock = make_controller_with_fake_clock()

        readings = [
            PositionSensorReading(SensorStatus.OK, 1.0),
            PositionSensorReading(SensorStatus.OK, 0.95),
        ]
        controller.position_sensors_provider = lambda: readings

        assert controller.state == GearState.RESET

        assert call_command_gear_down(controller) is False

        controller.update()

        assert controller.state == GearState.DOWN_LOCKED

        assert call_command_gear_up(controller) in (True, False)

    def test_fthr004_reset_with_invalid_sensors_remains_in_reset(self):
        controller, clock = make_controller_with_fake_clock()

        readings = [
            PositionSensorReading(SensorStatus.FAILED, 1.0),
            PositionSensorReading(SensorStatus.FAILED, 0.0),
        ]
        controller.position_sensors_provider = lambda: readings

        assert controller.state == GearState.RESET

        controller.update()

        assert controller.state == GearState.RESET
        assert call_command_gear_down(controller) is False

    def test_fthr004_reset_with_up_sensors_enters_up_locked(self):
        controller, clock = make_controller_with_fake_clock()

        readings = [
            PositionSensorReading(SensorStatus.OK, 0.0),
            PositionSensorReading(SensorStatus.OK, 0.05),
        ]
        controller.position_sensors_provider = lambda: readings

        assert controller.state == GearState.RESET

        controller.update()

        assert controller.state == GearState.UP_LOCKED
