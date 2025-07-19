
from enum import Enum
from sensors import *
from actuators import *
from navigation import Navigation
from sensors import *




class Robo():
    class robotState(Enum):
        STOPPED = 0
        FORWARD = 1
        BACKWARD = 2
        TURNING = 3

    def __init__(self, sim, robotName):
        self.robotName = robotName
        self.sim = sim
        self.currentState = self.robotState.FORWARD
        self.lastTime = 0
        self.currentTurn = "RIGHT"

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

        self.navigation = Navigation(leftWheel=self.leftWheel,rightWheel=self.rightWheel)

    def getCurrentState(self):
        return self.currentState
    
    def setCurrentState(self, state):
        self.currentState = state

    def normalCleaning(self):
        currentTime = self.sim.getSimulationTime()
        dt = currentTime - self.lastTime
        self.lastTime = currentTime

        frontBumperValue = self.frontBumper.measureDistance()[0]
        leftBumperValue = self.leftBumper.measureDistance()[0]
        rightBumperValue = self.rightBumper.measureDistance()[0]

        linearVelocityValue, angularVelocityvalue = self.gyro.measureGyro()

        if self.currentState == self.robotState.FORWARD and frontBumperValue:
            print("Parede detectada!")
            self.setCurrentState(self.robotState.STOPPED)

        match self.getCurrentState():
            case self.robotState.FORWARD:
                self.navigation._moveForward()

            case self.robotState.BACKWARD:
                self.navigation._moveBackward

            case self.robotState.TURNING:
                self.navigation._turnRobot(self.currentTurn)

            case self.robotState.STOPPED:
                self.navigation._stopRobot()

