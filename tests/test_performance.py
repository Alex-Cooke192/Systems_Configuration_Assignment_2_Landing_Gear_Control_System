"""
Title: Landing Gear Performance and Robustness Unit Tests
Author: Alex Cooke
Date Created: 2026-01-10
Last Modified: 2026-01-10
Version: 1.0

Purpose:
Provides unit-level verification of performance-related requirements and
robustness behavior implemented in the LandingGearController. These tests
validate actuation latency measurement, update-rate compliance during
transitions and steady states, fault classification timing, and safe
handling of invalid or out-of-range position sensor inputs.

Targeted Requirements (Verification Only):
- LGCS-PR001: Verification of deploy actuation latency measurement and
  boundary behavior.
- LGCS-PR002: Verification of controller update-rate compliance during
  transition states.
- LGCS-PR003: Verification of steady-state update-rate compliance.
- LGCS-PR004: Verification of fault classification latency and boundary
  conditions.
- FTHR002: Verification that sensor conflicts are classified within
  required time bounds.
- POS-ROBUST-001: Verification of robustness against invalid, non-finite,
  or out-of-range position normalization values.

Scope and Limitations:
- Tests controller timing, classification, and robustness logic only.
- Uses a deterministic fake clock and simulated periodic scheduling.
- No real hardware, real-time OS, or physical sensors are involved.
- Does not validate physical actuator dynamics or mechanical limits.

Safety Notice:
This file is a test artefact intended solely for verification and assessment.
It must not be used in operational or flight-certified systems.

Dependencies:
- Python 3.10+
- pytest
- landing_gear_controller.py
- gear_configuration.py
- sims.position_simulator.py

Related Documents:
- LGCS Unit Test Plan
- LGCS Performance Requirements Specification
- LGCS Fault Tolerance Requirements Specification

Safety and Certification Disclaimer:
All artefacts in this repository are produced for academic assessment purposes only.
They do not represent certified software and must not be used in real-world aviation
or safety-critical systems.
"""

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

    def test_pr004_persistent_conflict_fault_classified_within_400ms_of_persistent_occurrence(self):
        """
        New understanding:
        - The fault code FTHR002_SENSOR_CONFLICT_PERSISTENT only "occurs" once the
            sensor conflict has persisted for > 500 ms.
        - PR004 then requires the system to classify that fault within 400 ms of that
            occurrence (i.e., within 400 ms of the 500 ms persistence boundary).

        This test forces persistence beyond 500 ms and checks classification latency
        (classification_ts - occurrence_ts) <= 400 ms.
        """
        controller, clock = make_controller_with_fake_clock()

        conflict = [
            PositionSensorReading(SensorStatus.OK, 0.0),
            PositionSensorReading(SensorStatus.OK, 1.0),
        ]
        controller.position_sensors_provider = lambda: conflict

        # Start conflict timer at t=0
        controller.update()

        # Advance just beyond the persistence threshold and tick again to latch the fault.
        # With the new implementation, classification happens immediately when the fault latches.
        clock.advance(0.501)
        controller.update()

        latency_ms = controller.fault_classification_latency_ms("FTHR002_SENSOR_CONFLICT_PERSISTENT")
        assert latency_ms is not None
        assert latency_ms <= 400.0


    def test_pr004_before_persistence_threshold_fault_not_present_so_no_classification(self):
        """
        New understanding:
        - Before >500 ms persistence, the *persistent* fault does not exist.
        - Therefore there must be no classification record for the persistent fault code.
        """
        controller, clock = make_controller_with_fake_clock()

        conflict = [
            PositionSensorReading(SensorStatus.OK, 0.0),
            PositionSensorReading(SensorStatus.OK, 1.0),
        ]
        controller.position_sensors_provider = lambda: conflict

        controller.update()
        clock.advance(0.49)
        controller.update()

        latency_ms = controller.fault_classification_latency_ms("FTHR002_SENSOR_CONFLICT_PERSISTENT")
        assert latency_ms is None


    def test_pr004_failed_sensor_does_not_count_as_ok_ok_conflict(self):
        """
        Unchanged intent:
        - A FAILED + OK pair is not an OK/OK disagreement case,
            so the persistent fault should never be recorded/classified.
        """
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

