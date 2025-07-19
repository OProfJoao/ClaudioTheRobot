
from enum import Enum, auto
from sensors import *
from actuators import *
from navigation import Navigation
from sensors import *


class Robo():
    class robotState(Enum):
        STOPPED = auto()
        FORWARD = auto()
        BACKWARD = auto()
        TURNING = auto()
        MOVING_CURVE = auto()

    def __init__(self, sim, robotName):
        self.robotName = robotName
        self.sim = sim
        self.currentState = self.robotState.FORWARD
        self.nextState = self.robotState.FORWARD
        self.lastTime = 0
        self.currentTurn = "RIGHT"
        self.backwardDistanceTarget = 0.0
        self.backwardDistance = 0.0
        self.turningAngleTarget = 0.0
        self.turningAngle = 0.0

        left_bumper_name = "/bumperLeft"
        right_bumper_name = "/bumperRight"
        front_bumper_name = "/bumperFront"

        left_drop_name = "/dropLeft"
        right_drop_name = "/dropRight"
        front_drop_name = "/dropFront"

        left_IR_name = "/IRLeft"
        front_IR_name = "/IRFront"

        left_wheel_name = "/leftMotor"
        right_wheel_name = "/rightMotor"
        gyro_name = ""

        self.leftBumper = ProximitySensor(sim, left_bumper_name, robotName)
        self.rightBumper = ProximitySensor(sim, right_bumper_name, robotName)
        self.frontBumper = ProximitySensor(sim, front_bumper_name, robotName)

        self.leftDrop = ProximitySensor(sim, left_drop_name, robotName)
        self.rightDrop = ProximitySensor(sim, right_drop_name, robotName)
        self.frontDrop = ProximitySensor(sim, front_drop_name, robotName)

        self.leftIR = ProximitySensor(sim, left_IR_name, robotName)
        self.frontIR = ProximitySensor(sim, front_IR_name, robotName)

        self.gyro = GyroSensor(sim, gyro_name, robotName)

        self.leftWheel = Motors(sim, left_wheel_name, robotName)
        self.rightWheel = Motors(sim, right_wheel_name, robotName)

        self.navigation = Navigation(
            leftWheel=self.leftWheel, rightWheel=self.rightWheel)

    def normalCleaning(self):
        currentTime = self.sim.getSimulationTime()
        dt = currentTime - self.lastTime
        self.lastTime = currentTime

        frontBumperValue = self.frontBumper.measureDistance()[0]
        leftBumperValue = self.leftBumper.measureDistance()[0]
        rightBumperValue = self.rightBumper.measureDistance()[0]

        leftDropValue = self.leftDrop.measureDistance()[1]
        rightDropValue = self.rightDrop.measureDistance()[1]
        frontDropValue = self.frontDrop.measureDistance()[1]

        linearVelocityValue, angularVelocityvalue = self.gyro.measureGyro()

        if (frontDropValue > 0.06 and not (self.currentState == self.robotState.BACKWARD)) or (self.currentState == self.robotState.FORWARD and frontBumperValue):
            print("Obstaculo detectado a frente!")
            self.backwardDistanceTarget = 0.2
            self.backwardDistance = 0.0
            self.turningAngleTarget = 180
            self.turningAngle = 0.0
            self.currentState = (self.robotState.BACKWARD)
            self.nextState = self.robotState.TURNING
            self.currentTurn = "LEFT" if self.currentTurn == "RIGHT" else "RIGHT"
            return

        if (leftDropValue > 0.06 and not (self.currentState == self.robotState.BACKWARD)) or (self.currentState == self.robotState.FORWARD and leftBumperValue):
            print("Obstaculo detectado a esquerda!")
            self.backwardDistanceTarget = 0.2
            self.backwardDistance = 0.0
            self.turningAngleTarget = 45
            self.turningAngle = 0.0
            self.currentState = (self.robotState.BACKWARD)
            self.nextState = self.robotState.MOVING_CURVE
            self.currentTurn = "RIGHT"
            return

        if (rightDropValue > 0.06 and not (self.currentState == self.robotState.BACKWARD)) or (self.currentState == self.robotState.FORWARD and rightBumperValue):
            print("Obstaculo detectado a direita!")
            self.backwardDistanceTarget = 0.2
            self.backwardDistance = 0.0
            self.turningAngleTarget = 45
            self.turningAngle = 0.0
            self.currentState = (self.robotState.BACKWARD)
            self.nextState = self.robotState.MOVING_CURVE
            self.currentTurn = "LEFT"
            return










        match self.currentState:
            case self.robotState.FORWARD:
                self.navigation._moveForward()

            case self.robotState.BACKWARD:
                self.navigation._moveBackward()
                self.backwardDistance += max(abs(x)
                                             for x in linearVelocityValue) * dt
                if self.backwardDistance >= self.backwardDistanceTarget:
                    self.currentState = self.robotState.TURNING

            case self.robotState.STOPPED:
                self.navigation._stopRobot()


            case self.robotState.TURNING:
                self.navigation._turnRobot(self.currentTurn)
                self.turningAngle += math.degrees(angularVelocityvalue[2] * dt)
                if abs(self.turningAngle) >= self.turningAngleTarget:
                    if self.nextState == self.robotState.MOVING_CURVE:
                        self.currentState = (self.nextState)
                        self.turningAngleTarget = 90
                        self.turningAngle = 0.0
                    else:
                        self.currentState = self.robotState.FORWARD

           

            case self.robotState.MOVING_CURVE:
                turn = "LEFT" if self.currentTurn == "RIGHT" else "RIGHT"
                self.navigation._deviate(turn)
                self.turningAngle += math.degrees(angularVelocityvalue[2] * dt)
                if abs(self.turningAngle) >= self.turningAngleTarget:
                    self.turningAngleTarget = 45
                    self.turningAngle = 0.0
                    self.currentState = self.robotState.TURNING
                    self.nextState = self.robotState.FORWARD
