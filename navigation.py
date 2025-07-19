from actuators import Motors
from sensors import ProximitySensor


class Navigation:

    def __init__(self, leftWheel, rightWheel):
        self.rightWheel = rightWheel
        self.leftWheel = leftWheel

    def moveStraight(self, speed=0.8):
        self.rightWheel.setSpeed(speed)
        self.leftWheel.setSpeed(speed)

    def stopRobot(self):
        self.moveStraight(0.0)

    def turn(self, angle):
        pass

    def deviate(self, side):
        pass
