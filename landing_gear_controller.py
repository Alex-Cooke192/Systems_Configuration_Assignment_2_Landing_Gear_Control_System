## Top file comment

import time
from enum import Enum, auto
from gear_configuration import GearConfiguration

class GearState(Enum):
    UP_LOCKED = auto()
    TRANSITIONING_DOWN = auto()
    DOWN_LOCKED = auto()

class LandingGearController:
    def __init__(self, config: GearConfiguration):
        self.config = config
        self.state = GearState.UP_LOCKED

        # Timing instrumentation
        self._deploy_cmd_ts: float | None = None
        self._deploy_transition_ts: float | None = None

    def log(self, msg: str) -> None:
        print(msg)

    def _enter_state(self, new_state: GearState):
        self._state = new_state
        self._state_entered_at = time.now()

    def update():
        pass

    def command_gear_down(self):
        # Only accept deploy when in UP_LOCKED state 
        if self.state != GearState.UP_LOCKED:
            self.log("""Deploy REJECTED\n
                        Current state: {self.state}\n
                        Not in UP_LOCKED""")
            return False
        
        # Begin actuation immediately
        self.state = GearState.TRANSITIONING_DOWN
        ### Calculate the time taken from function inviocation to landing gear deployment
        self.deploy_start_delay = time.monotonic()
        self.log("Gear deploying...")
        self.log("Time taken to begin actuation: {self.deploy_start_delay}")
        return True
        

controller = LandingGearController()
controller.command_gear_down()


