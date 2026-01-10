from landing_gear_controller import LandingGearController
from gear_configuration import GearConfiguration
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

        controller, sim, clock = make_controller_with_fake_clock()

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
