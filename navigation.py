from enum import Enum
import math
import time
from actuators import Motors
from sensors import ProximitySensor


class Navigation:

    def __init__(self, leftWheel, rightWheel):
        self.leftWheel = leftWheel
        self.rightWheel = rightWheel

    def _moveForward(self, speed=0.8):
        print("Movendo para frente...")
        self.rightWheel.setSpeed(speed)
        self.leftWheel.setSpeed(speed)

    def _stopRobot(self):
        self._moveForward(0.0)

    def _turnRobot(self, side):
        self._stopRobot()
        if side == 'RIGHT':
            print("Virando para direita...")
            self.rightWheel.setSpeed(-0.08)
            self.leftWheel.setSpeed(0.4)
        else:
            print("Virando para esquerda...")
            self.rightWheel.setSpeed(0.4)
            self.leftWheel.setSpeed(-0.08)
        self._stopRobot()

    def _moveBackward(self):
        print("Dando ré...")
        self._stopRobot()
        self.rightWheel.setSpeed(-0.4)
        self.leftWheel.setSpeed(-0.4)
        self._stopRobot()