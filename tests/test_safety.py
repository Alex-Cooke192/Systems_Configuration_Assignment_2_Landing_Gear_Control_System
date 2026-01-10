import random
import pytest

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

    def test_sr002_warns_below_2000ft_when_gear_not_down(self):
        # LGCS-SR002:
        # Confirm warning is delivered when altitude drops below 2000 ft under normal conditions
        # while landing gear state is not DOWN.

        controller, sim, clock = make_controller_with_fake_clock()

        # Capture warnings via log override
        messages: list[str] = []
        controller.log = lambda msg: messages.append(msg)

        # Altitude above threshold produces no warning
        sim.set_altitude_ft(2500.0)
        controller.update()
        assert not any(
            "WARNING: ALTITUDE LOW - LANDING GEAR NOT DEPLOYED" in m
            for m in messages
        )

        # Drop altitude below threshold
        sim.set_altitude_ft(1999.0)
        controller.update()

        # Verify visual warning text is delivered
        assert any(
            "WARNING: ALTITUDE LOW - LANDING GEAR NOT DEPLOYED" in m
            for m in messages
        )

    def test_sr003_inhibits_retract_when_weight_on_wheels_true(self):
        # LGCS-SR003:
        # Confirm retraction is inhibited when weight-on-wheels is TRUE.

        controller, sim, clock = make_controller_with_fake_clock()

        controller.enter_state(GearState.DOWN_LOCKED)
        controller.set_weight_on_wheels(True)

        accepted = controller.command_gear_up(True)
        assert accepted is False
        assert controller.state == GearState.DOWN_LOCKED

    def test_sr004_defaults_to_down_and_ignores_pilot_retract_on_power_loss(self):
        # LGCS-SR004:
        # Confirm loss of primary control power forces deploy and inhibits pilot input.

        power_present = True

        def power_provider():
            return power_present

        controller, sim, clock = make_controller_with_fake_clock()

        controller.primary_power_present_provider = power_provider

        # Start UP, then lose power.
        assert controller.state == GearState.UP_LOCKED
        power_present = False

        controller.update()
        assert controller.state == GearState.TRANSITIONING_DOWN

        # Pilot retract command is ignored while power not present.
        controller.enter_state(GearState.DOWN_LOCKED)
        controller.set_weight_on_wheels(False)
        assert controller.command_gear_up(True) is False
        assert controller.state == GearState.DOWN_LOCKED
