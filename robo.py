
from enum import Enum, auto
from sensors import *
from actuators import *
from navigation import Navigation
from sensors import *
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

MAXDEVIATION = 0.003
MIN_DROP_HEIGHT = 0.06
TOLERANCE = 0.01


MIN_SPEED = 0.02
MAX_SPEED = 0.8

ACCUM_ERROR = 0.0

fig, ax = plt.subplots()
line, = ax.plot([], [], 'r-')


class Robo():
    class robotState(Enum):
        STOPPED = auto()
        FORWARD = auto()
        BACKWARD = auto()
        TURNING = auto()
        TURNING_AXIS = auto()
        DEVIATING_FIRST = auto()
        DEVIATING_SECOND = auto()
        DEVIATING_THIRD = auto()

    class deviateSide(Enum):
        RIGHT = -1
        LEFT = 1

    def _setupSensors(self, sim, robotName):
        left_bumper_name = "/bumperLeft"
        right_bumper_name = "/bumperRight"
        front_bumper_name = "/bumperFront"

        left_drop_name = "/dropLeft"
        right_drop_name = "/dropRight"
        front_drop_name = "/dropFront"

        left_IR_name = "/IRLeft"
        front_IR_name = "/IRFront"

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

    def _setupActuators(self, sim, robotName):
        left_wheel_name = "/leftMotor"
        right_wheel_name = "/rightMotor"

        self.leftWheel = Motors(sim, left_wheel_name, robotName)
        self.rightWheel = Motors(sim, right_wheel_name, robotName)

        self.navigation = Navigation(
            leftWheel=self.leftWheel,
            rightWheel=self.rightWheel
        )

    def __init__(self, sim, robotName,savePath):
        self.sim = sim
        self.robotName = robotName
        self.savePath = savePath

        self._setupSensors(self.sim, self.robotName)
        self._setupActuators(self.sim, self.robotName)
        
        self.ACCUM_ERROR = ACCUM_ERROR

        self.currentState = self.robotState.FORWARD
        self.nextState = self.robotState.FORWARD

        self.lastTime = 0
        self.rodeDistance = 0.0
        self.maxRodeDistance = 0.0
        self.backwardDistance = 0.0
        self.backwardDistanceTarget = 0.0

        self.angularError = 0
        self.relativeAngle = 0
        self.absoluteOrientationRad = 0
        self.targetAngle = 0.0

        self.currentTurn = "RIGHT"
        self.currentDeviate = self.deviateSide.RIGHT
        self.firstTurn = True

    def normalCleaning(self):

        def _getRadians(angle):
            return angle * math.pi/180

        def _limitAngles(angle):
            return math.atan2(math.sin(angle), math.cos(angle))

        def _angleError(absolute, target):
            currentAngle = _limitAngles(absolute)
            target = _limitAngles(target)
            error = target - currentAngle

            if error > math.pi:
                error -= 2 * math.pi
            if error < -math.pi:
                error += 2 * math.pi
            return error

        currentTime = self.sim.getSimulationTime()
        dt = currentTime - self.lastTime
        self.lastTime = currentTime

        leftBumperValue = self.leftBumper.measureDistance()[0]
        frontBumperValue = self.frontBumper.measureDistance()[0]
        rightBumperValue = self.rightBumper.measureDistance()[0]

        leftDropValue = self.leftDrop.measureDistance()[1]
        frontDropValue = self.frontDrop.measureDistance()[1]
        rightDropValue = self.rightDrop.measureDistance()[1]

        left_speed, right_speed = self.navigation._getSpeed()

        linearVelocityValue, angularVelocityvalue = self.gyro.measureGyro()

        self.absoluteOrientationRad += angularVelocityvalue[2] * dt

       
        # print(f'LeftWheel: {left_speed}/ RightWheel:{right_speed} / AngularV: {self.absoluteOrientationRad}')

        with open(self.savePath, 'a') as file:
            file.write(
                f'{dt};{left_speed};{right_speed};{self.absoluteOrientationRad}\n')

        self.rodeDistance += max(abs(x) for x in linearVelocityValue) * dt

        if (frontDropValue > MIN_DROP_HEIGHT) or (self.currentState == self.robotState.FORWARD and frontBumperValue):
            print("Obstaculo detectado a frente!")
            self.backwardDistanceTarget = 0.3
            self.backwardDistance = 0.0

            self.initialAngle = self.absoluteOrientationRad
            self.targetAngle = self.initialAngle + _getRadians(180)

            self.currentState = (self.robotState.BACKWARD)

            if self.firstTurn:
                self.nextState = self.robotState.TURNING_AXIS
                self.currentDeviate = self.deviateSide.LEFT.value
                self.firstTurn = False
                self.rodeDistance = 0
                self.maxRodeDistance = 0
            else:
                self.nextState = self.robotState.TURNING

                print(f'rode: {self.rodeDistance} max: {self.maxRodeDistance}')
                if self.rodeDistance > (self.maxRodeDistance + 0.5):
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

        if (leftDropValue > MIN_DROP_HEIGHT) or (self.currentState == self.robotState.FORWARD and leftBumperValue):
            print("Obstaculo detectado a esquerda!")
            self.backwardDistanceTarget = 0.1
            self.backwardDistance = 0.0

            self.initialAngle = self.absoluteOrientationRad

            self.currentDeviate = self.deviateSide.LEFT.value

            self.targetAngle = self.initialAngle + \
                (_getRadians(45) * self.deviateSide.RIGHT.value)
            
            self.currentState = self.robotState.BACKWARD
            self.nextState = self.robotState.DEVIATING_FIRST

            return

        if (rightDropValue > MIN_DROP_HEIGHT) or (self.currentState == self.robotState.FORWARD and rightBumperValue):
            print("Obstaculo detectado a direita!")
            self.backwardDistanceTarget = 0.1
            self.backwardDistance = 0.0

            self.currentDeviate = self.deviateSide.RIGHT.value

            self.initialAngle = self.absoluteOrientationRad
            self.targetAngle = self.initialAngle + \
                (_getRadians(45) * self.deviateSide.LEFT.value)
            self.currentState = self.robotState.BACKWARD
            self.nextState = self.robotState.DEVIATING_FIRST

            return

        print(f'rad: {self.absoluteOrientationRad - self.targetAngle} / deg:{math.degrees(self.absoluteOrientationRad) - math.degrees(self.targetAngle)} / accum: {self.ACCUM_ERROR}')




        match self.currentState:
            case self.robotState.FORWARD:
                error = _angleError(
                    self.absoluteOrientationRad, self.targetAngle) * 1.5
                #print("error: "+str(error))

                self.navigation._moveForward(MAX_SPEED - error, MAX_SPEED + error)

            case self.robotState.BACKWARD:
                self.navigation._moveBackward()
                self.backwardDistance += max(abs(x)
                                             for x in linearVelocityValue) * dt

                if self.backwardDistance >= self.backwardDistanceTarget:
                    match self.nextState:
                        case self.robotState.DEVIATING_FIRST:
                            self.currentState = self.robotState.DEVIATING_FIRST

                        case self.robotState.TURNING_AXIS:
                            self.currentState = self.robotState.TURNING_AXIS

                        case _:
                            self.currentState = self.robotState.TURNING

            case self.robotState.STOPPED:
                self.navigation._stopRobot()

            case self.robotState.TURNING:
                error = _angleError(
                    self.absoluteOrientationRad, self.targetAngle)
                print(
                    f"target: {self.targetAngle} / current: {self.absoluteOrientationRad} / error: {error}")
                speed = abs(error) * 1.5
                speed = max(MIN_SPEED, min(speed, MAX_SPEED))

                if abs(error) < TOLERANCE:
                    self.ACCUM_ERROR += error
                    self.navigation._stopRobot()

                    # self.targetAngle = self.absoluteOrientationRad
                    self.currentState = self.robotState.FORWARD
                    return

                self.navigation._turnRobot(self.currentTurn, speed)

            case self.robotState.TURNING_AXIS:
                error = _angleError(
                    self.absoluteOrientationRad, self.targetAngle)
                print(
                    f"target: {self.targetAngle} / current: {self.absoluteOrientationRad} / error: {error}")
                speed = abs(error) * 1.5
                speed = max(MIN_SPEED, min(speed, MAX_SPEED))

                if abs(error) < TOLERANCE:
                    self.ACCUM_ERROR += error
                    self.navigation._stopRobot()

                    # self.targetAngle = self.absoluteOrientationRad
                    self.currentState = self.robotState.FORWARD
                    return
                self.navigation._turnRobot_in_axis(side="RIGHT", speed=speed)
        

            case self.robotState.DEVIATING_FIRST:
                error = _angleError(
                    self.absoluteOrientationRad, self.targetAngle)
                #print(f"target: {self.targetAngle} / current: {self.absoluteOrientationRad} / error: {error}")
                speed = abs(error) * 1.5
                speed = max(MIN_SPEED, min(speed, MAX_SPEED))

                if abs(error) < TOLERANCE:
                    self.ACCUM_ERROR += error
                    self.navigation._stopRobot()
                    self.initialAngle = self.absoluteOrientationRad

                    self.targetAngle = self.initialAngle + \
                        (_getRadians(90) * self.currentDeviate)

                    self.currentDeviate = self.currentDeviate * -1
                    self.currentState = self.robotState.DEVIATING_SECOND
                    return
                
                side = "LEFT" if self.currentDeviate == self.deviateSide.RIGHT.value else "RIGHT"
                self.navigation._turnRobot_in_axis(side=side, speed=speed)
                
               
            case self.robotState.DEVIATING_SECOND:

                error = _angleError(
                    self.absoluteOrientationRad, self.targetAngle)

                speed = abs(error) * 1.5
                speed = max(MIN_SPEED, min(speed, MAX_SPEED))

                #print(f"target: {self.targetAngle} / current: {self.absoluteOrientationRad} / error: {error}")

                if abs(error) < TOLERANCE:
                    self.ACCUM_ERROR += error
                    self.navigation._stopRobot()
                    self.initialAngle = self.absoluteOrientationRad
                    self.targetAngle = self.initialAngle + \
                        (_getRadians(50) * self.currentDeviate)        ### Não sei por que ainda, mas voltar 50 graus fez com que o robo se comportasse melhor
                    #self.currentState = self.robotState.STOPPED
                    self.currentState = self.robotState.DEVIATING_THIRD
                    return
                side = "LEFT" if self.currentDeviate == self.deviateSide.RIGHT.value else "RIGHT"


                self.navigation._deviate(side,0.5)

            case self.robotState.DEVIATING_THIRD:
                error = _angleError(
                    self.absoluteOrientationRad, self.targetAngle)
                #print(f"target: {self.targetAngle} / current: {self.absoluteOrientationRad} / error: {error}")
                speed = abs(error) * 1.5
                speed = max(MIN_SPEED, min(speed, MAX_SPEED))

                if abs(error) < TOLERANCE:
                    self.ACCUM_ERROR += error
                    self.navigation._stopRobot()

                    self.targetAngle = self.absoluteOrientationRad
                    self.currentState = self.robotState.FORWARD
                    return

                side = "RIGHT" if self.currentDeviate == self.deviateSide.RIGHT.value else "LEFT"
                self.navigation._turnRobot_in_axis(side, speed)
