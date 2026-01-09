## Top file comment

import time

class GearState(Enum):
    UP_LOCKED = auto()
    TRANSITIONING_DOWN = auto()
    DOWN_LOCKED = auto()

class LandingGearController:
    def __init__(self):
        self.state = GearState.UP_LOCKED

    def log(self, msg:str):
        print(msg)

    def command_gear_down(self):
        # Only accept deploy when in UP_LOCKED state 
        if self.state != GearState.UP_LOCKED:
            self.log("""Deploy REJECTED\n
                        Current state: {self.state}\n
                        Not in UP_LOCKED""")
            return False
        
        # Begin actuation immediately (LGCS-PR001)
        self.state = GearState.TRANSITIONING_DOWN
        self.ui_status = "DEPLOYING"
        self._deploy_start_ts = time.monotonic()
        self.log("Gear deploying...")
        

controller = LandingGearController()
controller.command_gear_down()


