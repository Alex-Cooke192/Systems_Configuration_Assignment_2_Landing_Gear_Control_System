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

class testSafety:
    def test_auto_deploys_when_altitude_below_1000ft_and_gear_not_down_in_normal_conditions():
        # LGCS-SR001:
        # Verify automatic deploy occurs if altitude drops below 1000 ft during normal conditions
        # while landing gear state is not DOWN.

        controller, clock = make_controller_with_fake_clock()

        # Establish a non-DOWN steady state as the precondition.
        assert not controller.state.name.startswith("DOWN")

        # Provide normal-conditions input.
        # Adaptation point: replace with the project's actual API for conditions.
        if hasattr(controller, "set_flight_conditions"):
            controller.set_flight_conditions(normal=True)
        elif hasattr(controller, "normal_conditions"):
            controller.normal_conditions = True

        # Provide altitude input above threshold, then drop below threshold.
        # Adaptation point: replace with the project's actual altitude injection API.
        if hasattr(controller, "set_altitude_ft"):
            controller.set_altitude_ft(1500)
        elif hasattr(controller, "altitude_ft"):
            controller.altitude_ft = 1500

        controller.update()
        clock._t += 0.1

        # Drop below 1000 ft at a point during flight.
        if hasattr(controller, "set_altitude_ft"):
            controller.set_altitude_ft(999)
        elif hasattr(controller, "altitude_ft"):
            controller.altitude_ft = 999

        controller.update()

        # Verify the controller initiated deploy.
        # Acceptance criteria can be "transition begins" or "deploy command issued", depending on implementation.
        assert (
            controller.state.name.startswith("TRANSITIONING")
            or controller.state.name.startswith("DOWN")
        ), f"Automatic deploy was not initiated after altitude dropped below 1000 ft (state={controller.state})"
