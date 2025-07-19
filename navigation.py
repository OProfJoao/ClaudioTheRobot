from enum import Enum
import math
import time
from actuators import Motors
from sensors import ProximitySensor


class Navigation:

    def __init__(self, leftWheel, rightWheel):
        self.leftWheel = leftWheel
        self.rightWheel = rightWheel

    def _moveForward(self, left=0.8,right = 0.8):
        self.rightWheel.setSpeed(right)
        self.leftWheel.setSpeed(left)

    def _stopRobot(self):
        self._moveForward(0.0)

    def _turnRobot(self, side,speed=0.4):
        if side == 'RIGHT':
            self.rightWheel.setSpeed(0)
            self.leftWheel.setSpeed(speed)
        else:
            self.rightWheel.setSpeed(speed)
            self.leftWheel.setSpeed(0)

    def _turnRobot180(self):
            self.rightWheel.setSpeed(-0.4)
            self.leftWheel.setSpeed(0.4)

    def _moveBackward(self):
        self.rightWheel.setSpeed(-0.4)
        self.leftWheel.setSpeed(-0.4)

    def _deviate(self,side):
        if side == 'RIGHT':
            self.rightWheel.setSpeed(0.2)
            self.leftWheel.setSpeed(0.4)
        else:
            self.rightWheel.setSpeed(0.4)
            self.leftWheel.setSpeed(0.2)
