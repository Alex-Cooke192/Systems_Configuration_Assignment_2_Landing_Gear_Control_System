## Top file comment

import time
from enum import Enum, auto
from gear_configuration import GearConfiguration

class GearState(Enum):
    UP_LOCKED = auto()
    TRANSITIONING_DOWN = auto()
    DOWN_LOCKED = auto()

class LandingGearController:
    def __init__(self, config: GearConfiguration, clock=time.monotonic):
        self._config = config
        self._state = GearState.UP_LOCKED
        self._clock = clock

        # Timing instrumentation
        self._deploy_cmd_ts: float | None = None
        self._deploy_transition_ts: float | None = None

        # Stores cached deploy timing derived from configuration
        self._deploy_time_s = self._config.compute_deploy_time_ms() / 1000.0

    def log(self, msg: str) -> None:
        print(msg)

    def enter_state(self, new_state: GearState):
        self._state = new_state
        self._state_entered_at = self._clock

    def update(self):
        # Advances landing gear state machine by one control tick
        elapsed_s = self._clock - self._state_entered_at

        if self._state == GearState.UP_LOCKED:
            # Initiates extension when down command is present
            if self.down_requested():
                self.command_gear_down(True)
                self.enter_state(GearState.TRANSITIONING_DOWN)
            return

        if self._state == GearState.TRANSITIONING_DOWN:
            # Maintains extension command until deploy time elapses
            self.command_gear_down(True)

            # Completes transition when computed deploy time has elapsed
            if elapsed_s >= self._deploy_time_s:
                self.command_gear_down(False)
                self.enter_state(GearState.DOWN_LOCKED)
            return

        if self._state == GearState.DOWN_LOCKED:
            # Holds safe actuator state while gear is treated as locked down
            self.command_gear_down(False)
            return

    def command_gear_down(self):
        # Only accept deploy when in UP_LOCKED state 
        if self._state != GearState.UP_LOCKED:
            self.log("""Deploy REJECTED\n
                        Current state: {self._state}\n
                        Not in UP_LOCKED""")
            return False
        
        # Begin actuation immediately
        self._state = GearState.TRANSITIONING_DOWN
        ### Calculate the time taken from function inviocation to landing gear deployment
        self.deploy_start_delay = time.monotonic()
        self.log("Gear deploying...")
        self.log("Time taken to begin actuation: {self.deploy_start_delay}")
        return True
        

controller = LandingGearController()
controller.command_gear_down()


