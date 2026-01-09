import random

from landing_gear_controller import LandingGearController
from gear_configuration import GearConfiguration
from gear_states import GearState
from altitude_simulator import AltitudeSimulator

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

    sim = AltitudeSimulator(
        clock=clock,
        rng=random.Random(0),
        min_alt=500.0,
        max_alt=10_000.0,
    )

    controller = LandingGearController(
        config=config,
        clock=clock,
        altitude_provider=sim.read_altitude_ft,
        normal_conditions_provider=lambda: True,
    )

    return controller, sim, clock


class TestSafety:
    def test_auto_deploys_when_altitude_below_1000ft_and_gear_not_down_in_normal_conditions(self):
        # LGCS-SR001:
        # Confirm automatic deploy initiates when altitude drops below 1000 ft under normal conditions
        # while landing gear state is not DOWN.

        controller, sim, clock = make_controller_with_fake_clock()

        # Confirm initial condition is not a DOWN state.
        assert controller.state not in (GearState.DOWN_LOCKED, GearState.TRANSITIONING_DOWN)

        # Confirm no automatic deploy while altitude remains above threshold.
        sim.set_altitude_ft(1500.0)
        controller.update()
        assert controller.state == GearState.UP_LOCKED

        # Drop altitude below threshold and advance one control tick.
        clock.advance(0.1)
        sim.set_altitude_ft(999.0)
        controller.update()

        # Confirm deploy initiation.
        assert controller.state in (GearState.TRANSITIONING_DOWN, GearState.DOWN_LOCKED), (
            f"Automatic deploy was not initiated after altitude dropped below 1000 ft (state={controller.state})"
        )
