
from sensors import *
from actuators import *
from navigation import Navigation
from sensors import *

class Robo():
    def __init__(self,sim, robotName):
        self.robotName = robotName
        self.sim = sim

        left_bumper_name  = "/bumperLeft"
        right_bumper_name = "/bumperRight"
        front_bumper_name = "/bumperFront"

        left_drop_name    = "/dropLeft"
        right_drop_name   = "/dropRight"
        front_drop_name   = "/dropFront"
        
        left_IR_name      = "/IRLeft"
        front_IR_name     = "/IRFront"

        left_wheel_name   = "/leftMotor"
        right_wheel_name  = "/rightMotor"
        gyro_name         = "" 

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

        self.navigation   = Navigation(
            leftWheel=self.leftWheel,
            rightWheel=self.rightWheel,
            
            leftBumper=self.leftBumper,
            rightBumper=self.rightBumper,
            frontBumper=self.frontBumper,

            leftDrop=self.leftDrop,
            rightDrop=self.rightDrop,
            frontDrop=self.frontDrop,

            frontIR= self.frontIR,
            leftIR=self.leftIR,
            gyro = self.gyro,
            sim = sim
            )