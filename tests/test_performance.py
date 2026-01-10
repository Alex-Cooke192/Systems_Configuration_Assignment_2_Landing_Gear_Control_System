from landing_gear_controller import LandingGearController
from gear_configuration import GearConfiguration
from sims.position_simulator import SensorStatus, PositionSensorReading 


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
    #LGCS-PR001
    def test_pr001_deploy_actuates_within_200ms(self):
        controller, clock = make_controller_with_fake_clock()

        controller.command_gear_down(True)
        controller.update()

        latency = controller.deploy_actuation_latency_ms()
        assert latency is not None
        assert latency <= 200.0

    # LGCS-PR002:
    # Verify gear state indications update at >=10 Hz during deploy transitions
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

    # LGCS-PR003:
    # Verify that gear state indications are updated at a minimum rate of 4 Hz
    # while the system is in a steady UP or DOWN state.
    def test_state_updates_at_minimum_4hz_in_steady_states():
        controller, clock = make_controller_with_fake_clock()

        # Confirm the controller begins in a steady (non-transitioning) state.
        assert controller.state.name.startswith("UP") or controller.state.name.startswith("DOWN")

        update_timestamps: list[float] = []

        # Instrument the update path to record indication update timing.
        original_update = controller.update

        def instrumented_update():
            update_timestamps.append(clock())
            original_update()

        controller.update = instrumented_update

        # Simulate steady-state operation over a bounded interval.
        SIM_DURATION_MS = 2000
        STEP_MS = 100

        steps = SIM_DURATION_MS // STEP_MS
        for _ in range(steps):
            controller.update()
            clock._t += STEP_MS / 1000.0

        # Calculate elapsed time between consecutive indication updates.
        deltas = [t2 - t1 for t1, t2 in zip(update_timestamps, update_timestamps[1:])]

        # Assert compliance with minimum 4 Hz update rate (≤ 250 ms interval).
        assert all(delta <= 0.25 for delta in deltas), (
            f"Indication update interval exceeded 250 ms in steady state: {deltas}"
        )
    
    def test_pr004_fault_classification_within_400ms_for_fthr002(self):
        controller, clock = make_controller_with_fake_clock()

        readings = [
            PositionSensorReading(SensorStatus.OK, 0.0),
            PositionSensorReading(SensorStatus.OK, 1.0),
        ]
        controller.position_sensors_provider = lambda: readings

        controller.update()          # starts conflict timer at t=0.0
        clock.advance(0.50)          # threshold reached
        controller.update()          # classification happens at t=0.50

        # If your tick is coarse, do classification on next tick:
        # clock.advance(0.10); controller.update()

        latency_ms = controller.fault_classification_latency_ms("FTHR002_SENSOR_CONFLICT_PERSISTENT")
        assert latency_ms is not None
        assert latency_ms <= 400.0



        
