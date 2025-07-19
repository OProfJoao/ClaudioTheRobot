from coppeliasim import Components


class Motors(Components):
    def __init__(self, sim, motorName, robotName):
        super().__init__(sim, motorName, robotName)

    def setSpeed(self, speed):
        self.sim.setJointTargetVelocity(self.handle, speed)


class Led(Components):
    def __init__(self, sim, ledName):
        super().__init__(sim, ledName)
