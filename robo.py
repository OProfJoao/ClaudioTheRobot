
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
        TURNING180 = auto()
        DEVIATING_FIRST = auto()
        DEVIATING_SECOND = auto()
        DEVIATING_THIRD = auto()

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
        self.rodeDistance = 0.0
        self.maxRodeDistance = 0.0
        self.turningAngle = 0.0

        self.deviateSide = "RIGHT"

        self.firstTurn = True

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
        self.rodeDistance += max(abs(x) for x in linearVelocityValue) * dt

        if (frontDropValue > 0.06 and self.currentState != self.robotState.BACKWARD) or (self.currentState == self.robotState.FORWARD and frontBumperValue):
            print("Obstaculo detectado a frente!")
            self.backwardDistanceTarget = 0.2
            self.backwardDistance = 0.0
            self.turningAngleTarget = 180
            self.turningAngle = 0.0
            self.currentState = (self.robotState.BACKWARD)

            if self.firstTurn:
                self.nextState = self.robotState.TURNING180
                self.firstTurn = False
            else:
                self.nextState = self.robotState.TURNING

                print(f'rode: {self.rodeDistance} max: {self.maxRodeDistance}')
                if self.rodeDistance > self.maxRodeDistance  :
                    print("A")
                    self.maxRodeDistance = self.rodeDistance
                    self.currentTurn = self.currentTurn
                    self.rodeDistance = 0

                else:
                    print("B")
                    self.maxRodeDistance = self.rodeDistance
                    
                    self.currentTurn = "LEFT" if self.currentTurn == "RIGHT" else "RIGHT"
                    self.rodeDistance = 0

            
            return

        if (leftDropValue > 0.06 and self.currentState != self.robotState.BACKWARD) or (self.currentState == self.robotState.FORWARD and leftBumperValue):
            print("Obstaculo detectado a esquerda!")
            self.backwardDistanceTarget = 0.2
            self.backwardDistance = 0.0
            self.turningAngleTarget = 45
            self.turningAngle = 0.0
            self.currentState = self.robotState.BACKWARD
            self.nextState = self.robotState.DEVIATING_FIRST
            self.deviateSide = "RIGHT"
            return

        if (rightDropValue > 0.06 and self.currentState != self.robotState.BACKWARD) or (self.currentState == self.robotState.FORWARD and rightBumperValue):
            print("Obstaculo detectado a direita!")
            self.backwardDistanceTarget = 0.2
            self.backwardDistance = 0.0
            self.turningAngleTarget = 45
            self.turningAngle = 0.0
            self.currentState = self.robotState.BACKWARD
            self.nextState = self.robotState.DEVIATING_FIRST
            self.deviateSide = "LEFT"
            return






        match self.currentState:
            case self.robotState.FORWARD:
                self.navigation._moveForward()

            case self.robotState.BACKWARD:
                self.navigation._moveBackward()
                self.backwardDistance += max(abs(x)
                                             for x in linearVelocityValue) * dt
                if self.backwardDistance >= self.backwardDistanceTarget:
                    match self.nextState:
                        case self.robotState.DEVIATING_FIRST:
                            self.currentState = self.robotState.DEVIATING_FIRST
                        case self.robotState.TURNING180:
                            self.currentState = self.robotState.TURNING180
                        case _:
                            self.currentState = self.robotState.TURNING
                        

            case self.robotState.STOPPED:
                self.navigation._stopRobot()


            case self.robotState.TURNING:
                self.navigation._turnRobot(self.currentTurn)

                self.turningAngle += math.degrees(angularVelocityvalue[2] * dt)
                if abs(self.turningAngle) >= self.turningAngleTarget:
                    self.currentState = self.robotState.FORWARD

            case self.robotState.TURNING180:
                self.navigation._turnRobot180()
                self.turningAngle += math.degrees(angularVelocityvalue[2] * dt)
                if abs(self.turningAngle) >= self.turningAngleTarget:
                    self.currentState = self.robotState.FORWARD


            case self.robotState.DEVIATING_FIRST:
                self.navigation._turnRobot(self.deviateSide)
                self.turningAngle += math.degrees(angularVelocityvalue[2] * dt)
                if abs(self.turningAngle) >= self.turningAngleTarget:
                    self.turningAngleTarget = 45
                    self.turningAngle = 0.0
                    self.currentState = self.robotState.DEVIATING_SECOND
            
            case self.robotState.DEVIATING_SECOND:
                side = "LEFT" if self.deviateSide == "RIGHT" else "RIGHT"
                self.navigation._deviate(side)
                self.turningAngle += math.degrees(angularVelocityvalue[2] * dt)
                if abs(self.turningAngle) >= 90:
                    self.turningAngleTarget = 45
                    self.turningAngle = 0.0
                    self.currentState = self.robotState.DEVIATING_THIRD

            case self.robotState.DEVIATING_THIRD:
                self.navigation._turnRobot(self.deviateSide)
                self.turningAngle += math.degrees(angularVelocityvalue[2] * dt)
                if abs(self.turningAngle) >= self.turningAngleTarget:
                    self.currentState = self.robotState.FORWARD