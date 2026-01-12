# test_simulators.py
#
# Unit tests for simulator-layer data models and utilities.
# Scope:
# - sims.position_simulator: SensorStatus, PositionSensorReading
# - sims.altitude_simulator: AltitudeSimulator

import dataclasses
import math
import random

import pytest

from sims.position_simulator import SensorStatus, PositionSensorReading
from sims.altitude_simulator import AltitudeSimulator


# -----------------------------
# sims.position_simulator tests
# -----------------------------

def test_sensor_status_members_exist():
    assert SensorStatus.OK in SensorStatus
    assert SensorStatus.FAILED in SensorStatus


def test_sensor_status_members_are_distinct():
    assert SensorStatus.OK != SensorStatus.FAILED


def test_sensor_status_is_enum_type():
    assert isinstance(SensorStatus.OK, SensorStatus)
    assert isinstance(SensorStatus.FAILED, SensorStatus)


def test_position_sensor_reading_is_frozen_dataclass():
    assert dataclasses.is_dataclass(PositionSensorReading)
    assert getattr(PositionSensorReading, "__dataclass_params__").frozen is True


def test_position_sensor_reading_construction_valid_endpoints():
    r_up = PositionSensorReading(SensorStatus.OK, 0.0)
    r_down = PositionSensorReading(SensorStatus.OK, 1.0)

    assert r_up.status is SensorStatus.OK
    assert r_up.position_norm == 0.0

    assert r_down.status is SensorStatus.OK
    assert r_down.position_norm == 1.0


@pytest.mark.parametrize("pos", [0.5, 0.0001, 0.9999])
def test_position_sensor_reading_construction_valid_interior(pos):
    r = PositionSensorReading(SensorStatus.OK, pos)
    assert r.status is SensorStatus.OK
    assert r.position_norm == pos


@pytest.mark.parametrize("pos", [-0.1, -1.0, 1.1, 2.0])
def test_position_sensor_reading_allows_out_of_range_values_as_data(pos):
    r = PositionSensorReading(SensorStatus.OK, pos)
    assert r.status is SensorStatus.OK
    assert r.position_norm == pos


@pytest.mark.parametrize("pos", [float("inf"), float("-inf")])
def test_position_sensor_reading_allows_infinite_values_as_data(pos):
    r = PositionSensorReading(SensorStatus.OK, pos)
    assert r.status is SensorStatus.OK
    assert r.position_norm == pos
    assert math.isfinite(r.position_norm) is False


def test_position_sensor_reading_allows_nan_as_data():
    r = PositionSensorReading(SensorStatus.OK, float("nan"))
    assert r.status is SensorStatus.OK
    assert math.isnan(r.position_norm) is True


def test_position_sensor_reading_failed_status_is_stored():
    r = PositionSensorReading(SensorStatus.FAILED, 0.5)
    assert r.status is SensorStatus.FAILED
    assert r.position_norm == 0.5


def test_position_sensor_reading_is_immutable():
    r = PositionSensorReading(SensorStatus.OK, 0.25)

    with pytest.raises(dataclasses.FrozenInstanceError):
        r.position_norm = 0.9  # type: ignore[misc]

    with pytest.raises(dataclasses.FrozenInstanceError):
        r.status = SensorStatus.FAILED  # type: ignore[misc]


def test_position_sensor_reading_equality_semantics():
    a = PositionSensorReading(SensorStatus.OK, 0.25)
    b = PositionSensorReading(SensorStatus.OK, 0.25)
    c = PositionSensorReading(SensorStatus.OK, 0.26)
    d = PositionSensorReading(SensorStatus.FAILED, 0.25)

    assert a == b
    assert a != c
    assert a != d


def test_position_sensor_reading_hashable_for_use_in_sets():
    a = PositionSensorReading(SensorStatus.OK, 0.25)
    b = PositionSensorReading(SensorStatus.OK, 0.25)

    s = {a, b}
    assert len(s) == 1


# -----------------------------
# sims.altitude_simulator tests
# -----------------------------

class FakeClock:
    def __init__(self, start: float = 0.0):
        self._t = float(start)

    def __call__(self) -> float:
        return self._t

    def advance(self, dt: float) -> None:
        self._t += float(dt)


def test_altitude_simulator_initial_altitude_within_bounds():
    rng = random.Random(123)
    sim = AltitudeSimulator(min_alt=500.0, max_alt=10_000.0, rng=rng)

    alt = sim.read_altitude_ft()
    assert 500.0 <= alt <= 10_000.0


def test_step_dt_zero_does_not_change_state():
    rng = random.Random(123)
    sim = AltitudeSimulator(min_alt=500.0, max_alt=10_000.0, rng=rng)

    alt0 = sim.read_altitude_ft()
    vs0 = sim.vert_speed

    alt1 = sim.step(0.0)

    assert alt1 == alt0
    assert sim.read_altitude_ft() == alt0
    assert sim.vert_speed == vs0


def test_step_dt_negative_does_not_change_state():
    rng = random.Random(123)
    sim = AltitudeSimulator(min_alt=500.0, max_alt=10_000.0, rng=rng)

    alt0 = sim.read_altitude_ft()
    vs0 = sim.vert_speed

    alt1 = sim.step(-1.0)

    assert alt1 == alt0
    assert sim.read_altitude_ft() == alt0
    assert sim.vert_speed == vs0


def test_step_altitude_remains_bounded_over_many_steps():
    rng = random.Random(123)
    sim = AltitudeSimulator(min_alt=500.0, max_alt=10_000.0, rng=rng)

    for _ in range(1000):
        alt = sim.step(0.1)
        assert 500.0 <= alt <= 10_000.0


def test_step_velocity_is_clamped_to_max_speed():
    rng = random.Random(123)
    sim = AltitudeSimulator(
        min_alt=500.0,
        max_alt=10_000.0,
        max_fpm=60.0,
        max_accel_fps2=1e6,
        rng=rng,
    )

    sim.step(10.0)

    assert -sim.max_speed <= sim.vert_speed <= sim.max_speed


def test_step_bounces_off_min_alt_and_sets_positive_velocity():
    rng = random.Random(123)
    sim = AltitudeSimulator(
        min_alt=500.0,
        max_alt=10_000.0,
        max_fpm=6000.0,
        max_accel_fps2=0.0,
        rng=rng,
    )

    sim.set_altitude_ft(500.0)
    sim.vert_speed = -50.0

    alt = sim.step(1.0)

    assert alt == 500.0
    assert sim.vert_speed >= 0.0


def test_step_bounces_off_max_alt_and_sets_negative_velocity():
    rng = random.Random(123)
    sim = AltitudeSimulator(
        min_alt=500.0,
        max_alt=10_000.0,
        max_fpm=6000.0,
        max_accel_fps2=0.0,
        rng=rng,
    )

    sim.set_altitude_ft(10_000.0)
    sim.vert_speed = 50.0

    alt = sim.step(1.0)

    assert alt == 10_000.0
    assert sim.vert_speed <= 0.0


def test_update_requires_clock():
    rng = random.Random(123)
    sim = AltitudeSimulator(rng=rng, clock=None)

    with pytest.raises(RuntimeError):
        sim.update()


def test_update_uses_injected_clock_and_advances_state():
    rng = random.Random(123)
    clock = FakeClock(0.0)
    sim = AltitudeSimulator(rng=rng, clock=clock)

    alt0 = sim.read_altitude_ft()

    clock.advance(1.0)
    alt1 = sim.update()

    assert alt1 == sim.read_altitude_ft()
    assert alt1 != alt0


def test_update_advances_twice_with_two_clock_steps():
    rng = random.Random(123)
    clock = FakeClock(0.0)
    sim = AltitudeSimulator(rng=rng, clock=clock)

    alt0 = sim.read_altitude_ft()

    clock.advance(0.5)
    alt1 = sim.update()

    clock.advance(0.5)
    alt2 = sim.update()

    assert alt1 != alt0
    assert alt2 != alt1


def test_update_with_no_time_advance_returns_same_altitude():
    rng = random.Random(123)
    clock = FakeClock(10.0)
    sim = AltitudeSimulator(rng=rng, clock=clock)

    alt0 = sim.read_altitude_ft()
    alt1 = sim.update()

    assert alt1 == alt0
    assert sim.read_altitude_ft() == alt0


def test_set_altitude_ft_sets_float_value():
    rng = random.Random(123)
    sim = AltitudeSimulator(rng=rng)

    sim.set_altitude_ft(1234)
    assert sim.read_altitude_ft() == 1234.0


def test_read_altitude_ft_does_not_advance_state():
    rng = random.Random(123)
    sim = AltitudeSimulator(rng=rng)

    alt0 = sim.read_altitude_ft()
    alt1 = sim.read_altitude_ft()

    assert alt1 == alt0
