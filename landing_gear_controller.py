## Top file comment

import time

class GearState(Enum):
    UP_LOCKED = auto()
    TRANSITIONING_DOWN = auto()
    DOWN_LOCKED = auto()

class LandingGearController:
    def __init__(self):
        self.state = GearState.UP_LOCKED
        self.deploy_start_delay = None

    def log(self, msg:str):
        print(msg)

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


