## Top file comment

import time
from gear_configuration import GearConfiguration
from gear_states import GearState

class LandingGearController:
    def __init__(self, config: GearConfiguration, clock=time.monotonic):
        self._config = config
        self._state = GearState.UP_LOCKED

        self._clock = clock
        self._state_entered_at = self._clock()

        # Timing instrumentation
        self._deploy_cmd_ts: float | None = None
        self._deploy_transition_ts: float | None = None

        # Stores cached deploy timing derived from configuration
        self._deploy_time_s = self._config.compute_deploy_time_ms() / 1000.0

        # Stores request for deploy (eventually will be from the UI)
        self._deploy_requested = False

    def log(self, msg: str) -> None:
        print(msg)

    def enter_state(self, new_state: GearState):
        self._state = new_state
        self._state_entered_at = self._clock()
    
    def down_requested(self) -> bool:
        return self._deploy_requested

    def update(self) -> None:
        # Advances landing gear state machine by one control tick
        now = self._clock()
        elapsed_s = now - self._state_entered_at

        if self._state == GearState.UP_LOCKED:
            if self.down_requested():
                self.command_gear_down(True)
            return

        if self._state == GearState.TRANSITIONING_DOWN:
            # Maintains actuation during transition
            self._actuate_down(True)

            # Completes transition using computed deploy time
            if elapsed_s >= self._deploy_time_s:
                self.command_gear_down(False)
            return

        if self._state == GearState.DOWN_LOCKED:
            self._actuate_down(False)
            return

        if self._state == GearState.TRANSITIONING_DOWN:
            # Maintains extension command until deploy time elapses
            self.command_gear_down()

            # Completes transition when computed deploy time has elapsed
            if elapsed_s >= self._deploy_time_s:
                self.command_gear_down()
                self.enter_state(GearState.DOWN_LOCKED)
            return

        if self._state == GearState.DOWN_LOCKED:
            # Holds safe actuator state while gear is treated as locked down
            self.command_gear_down()
            return

    def command_gear_down(self, enabled: bool) -> bool:
         # Applies actuator command and performs any required state transition
        now = self._clock()

        if enabled:
            if self._state != GearState.UP_LOCKED:
                self.log(f"Deploy rejected: state={self._state.name}")
                return False

            self._deploy_cmd_ts = now
            self._deploy_transition_ts = now
            self._actuate_down(True)
            self.enter_state(GearState.TRANSITIONING_DOWN)
            return True

        if self._state == GearState.TRANSITIONING_DOWN:
            self._actuate_down(False)
            self.enter_state(GearState.DOWN_LOCKED)
            return True
    
    def _actuate_down(self, enabled: bool) -> None:
        self.log(f"Gear down actuator command: {enabled}")
        

cfg = GearConfiguration(
    name="LG-1",
    pump_latency_ms=200,
    actuator_speed_mm_per_100ms=50.0,
    extension_distance_mm=500,
    lock_time_ms=300,
    requirement_time_ms=5000,
)


controller = LandingGearController(cfg)
controller._deploy_requested = True


