## Top file comment

class GearState(Enum):
    UP_LOCKED = auto()
    TRANSITIONING_DOWN = auto()
    DOWN_LOCKED = auto()

class LandingGearController:
    def __init__(self):
        self.state = GearState.UP_LOCKED

    def command_gear_down(self):
        if self.state == GearState.UP_LOCKED:
            self.state = GearState.TRANSITIONING_DOWN
            self.log("Gear deploying")
            self.state = GearState.DOWN_LOCKED
            self.log("Gear locked down")
        else:
            self.log("Command rejected")

controller = LandingGearController()
controller.command_gear_down()


