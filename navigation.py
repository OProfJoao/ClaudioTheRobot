import math
import time
from actuators import Motors
from sensors import ProximitySensor


class Navigation:

    def __init__(self,
                 leftWheel,
                 rightWheel,
                 leftBumper,
                 rightBumper,
                 frontBumper,
                 leftDrop,
                 rightDrop,
                 frontDrop,
                 gyro,
                 leftIR,
                 frontIR,sim):
        self.leftWheel = leftWheel
        self.rightWheel = rightWheel
        self.leftBumper = leftBumper
        self.rightBumper = rightBumper
        self.frontBumper = frontBumper
        self.leftDrop = leftDrop
        self.rightDrop = rightDrop
        self.frontDrop = frontDrop
        self.gyro = gyro
        self.leftIR = leftIR
        self.frontIR = frontIR
        self.sim = sim
        self.turningSide = "RIGHT"

    def _moveStraight(self, speed=0.8):
        self.rightWheel.setSpeed(speed)
        self.leftWheel.setSpeed(speed)

    def _stopRobot(self):
        self._moveStraight(0.0)

    def _turn(self, side, targetAngle):    
        self._stopRobot()
        currAngle = 0
        startTime = self._getSimulationTime()
        while abs(currAngle) < targetAngle:
            currTime = self._getSimulationTime()
            dt = currTime - startTime
            startTime = currTime
            
            if side == 'RIGHT':
                print("Virando a Direita")
                self.rightWheel.setSpeed(-0.08)
                self.leftWheel.setSpeed(0.4)
                self.turningSide = 'LEFT'
            else:
                print("Virando a Esquerda")
                self.leftWheel.setSpeed(-0.08)
                self.rightWheel.setSpeed(0.4)
                self.turningSide = 'RIGHT'
            _,angularVelocity = self.gyro.measureGyro()
            currAngle += math.degrees(angularVelocity[2] * dt)

        self._stopRobot()

    def _reverse(self, targetDistance=0.2):
        print("Dando ré...")
        self._stopRobot()
        currDistance = 0
        startTime = self._getSimulationTime()
        while abs(currDistance) < targetDistance:
            currTime = self._getSimulationTime()
            dt = currTime - startTime
            startTime = currTime

            self.rightWheel.setSpeed(-0.5)
            self.leftWheel.setSpeed(-0.5)
            linearVelocity,_ = self.gyro.measureGyro()
            currDistance += linearVelocity[0] * dt
            
        self._stopRobot()

        

    def _deviate(self, side):
        pass

    def normalCleaning(self):
        self._moveStraight(0.8)
        if self.frontBumper.measureDistance()[0]:
            print("Parede detectada")
            self._reverse()
            self._turn(side=self.turningSide, targetAngle=180)


    def _getSimulationTime(self):
        return self.sim.getSimulationTime()