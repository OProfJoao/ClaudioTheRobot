from enum import Enum
import math
from coppeliasim_zmqremoteapi_client import *
import time

import _thread




client = RemoteAPIClient()
sim = client.require('sim')

sim.stopSimulation()
time.sleep(1)

robot_name = "PioneerP3DX"


#Constants

LAST_TURN = "left"
DEVIATION_LIMIT = 0.01
RETURN_REVERSE_DIST = 0.2
DEVIATE_REVERSE_DIST = 0.1

SAFETY_ACT = False

class Side(Enum):
    LEFT = 1
    RIGHT = 0

def dontKYS():
    print("VAI CAIRRRR!!!")
    motorControl(0.0,0.0)
    backupALitte(RETURN_REVERSE_DIST)
    turnBack()

def backupALitte(reverse_distance):
    print("Dando ré")
    dist_total = 0
    motorControl(-0.5,-0.5)
    starTime = sim.getSimulationTime()
    while(abs(dist_total) < reverse_distance):
        linearVelocity, _ = sim.getObjectVelocity(gyro_handle)
        delta = sim.getSimulationTime() - starTime  
        dist_total =+ linearVelocity[0] * delta
        #print(dist_total)
    motorControl(0.0,0.0)
    
def motorControl(left,right):
    sim.setJointTargetVelocity(left_motor_handle,left)
    sim.setJointTargetVelocity(right_motor_handle,right)

def turn(side,angle):
    print("Virando")
    degree = 0
    match side:
        case side.RIGHT:
            motorControl(0.4,-0.05)
        case side.LEFT:
            motorControl(-0.05,0.4)
            
        case _:
            motorControl(0.0,0.0)
            return 0
        
    currTime = lastTime = sim.getSimulationTime()
    while( degree < angle):
        currTime = sim.getSimulationTime()
        dt = currTime - lastTime
        lastTime = currTime
        degree += abs(angleDegree(dt))
        #print(f"Virou: {degree:.3f}")
    motorControl(0.0,0.0)
    
    
    
def angleDegree(delta):
    _, angularVelocity = sim.getObjectVelocity(gyro_handle)
    angulo_total_rad = angularVelocity[2] * delta
    return math.degrees(angulo_total_rad)
    
def deviate(side):
    print(side)
    match side:
        case side.LEFT:
            motorControl(0.0,0.0)
            backupALitte(DEVIATE_REVERSE_DIST)
            turn(side.LEFT,90)
            turn(side.RIGHT,90)
        case side.RIGHT:
            motorControl(0.0,0.0)
            backupALitte(DEVIATE_REVERSE_DIST)
            turn(side.RIGHT,90)
            turn(side.LEFT,90)
               
    
sim.startSimulation()
try:
    left_motor_handle = sim.getObject(f'/{robot_name}/leftMotor')
    right_motor_handle = sim.getObject(f'/{robot_name}/rightMotor')
    gyro_handle = sim.getObject(f'/{robot_name}')
    
    IRLeft_handle = sim.getObject(f'/{robot_name}/IRLeft')
    IRFront_handle = sim.getObject(f'/{robot_name}/IRFront')
    
    bumperFront_handle = sim.getObject(f'/{robot_name}/bumperFront')
    bumperLeft_handle  = sim.getObject(f'/{robot_name}/bumperLeft')
    bumperRight_handle = sim.getObject(f'/{robot_name}/bumperRight')
    
    frontDrop_handle = sim.getObject(f'/{robot_name}/dropFront')
    rightDrop_handle = sim.getObject(f'/{robot_name}/dropRight')
    leftDrop_handle  = sim.getObject(f'/{robot_name}/dropLeft')
    
    
    print("Todos os objetos encontrados")
    safety = _thread.start_new_thread(dontKYS,())
    
except Exception as e:
    print(f"Erro ao obter os handles: {e}")
    sim.stopSimulation()
    exit()
    
def turnBack():
    global LAST_TURN
    if LAST_TURN == Side.RIGHT:
        backupALitte(RETURN_REVERSE_DIST)
        turn(Side.LEFT,180)
        LAST_TURN = Side.LEFT
    else:
        backupALitte(RETURN_REVERSE_DIST)
        turn(Side.RIGHT,180)
        LAST_TURN = Side.RIGHT
    
    
currTime = lastTime = sim.getSimulationTime()
angleDeviation = 0
LAST_TURN = Side.LEFT

while sim.getSimulationState() != sim.simulation_stopped:
    if angleDeviation > DEVIATION_LIMIT:
        motorControl(0.8+angleDeviation,0.8)
    elif angleDeviation < -DEVIATION_LIMIT:
        motorControl(0.8,0.8-angleDeviation)
    else:
        motorControl(0.8,0.8)
    
    
    currTime = sim.getSimulationTime()
    dt = currTime - lastTime
    lastTime = currTime
    degree = angleDegree(dt)
    result_bumper_front, bumper_front_dist, _, _, _ = sim.readProximitySensor(bumperFront_handle)
    result_bumper_right, bumper_right_dist, _, _, _ = sim.readProximitySensor(bumperRight_handle)
    result_bumper_left,  bumper_left_dist,  _, _, _ = sim.readProximitySensor(bumperLeft_handle)

    
    
    angleDeviation += degree * dt * 0.6 #K integral
    print(f"integral: {angleDeviation}")
    
    #print(f'SR: {bumper_right_dist:.2f}/ SF: {bumper_front_dist:.2f}/ SL: {bumper_left_dist:.2f}/')
    if SAFETY_ACT:
        print('Detectada queda!')
        turnBack()
    
    if result_bumper_front:     
        print("Tocou!")
        turnBack()
        angleDeviation = 0
        lastTime = sim.getSimulationTime()
        
    if result_bumper_left:
        print("Tocou LEFT!")
        deviate(Side.RIGHT)
        angleDeviation = 0
        lastTime = sim.getSimulationTime()
    
    if result_bumper_right:
        print("Tocou! RIGHT")
        deviate(Side.LEFT)
        angleDeviation = 0
        lastTime = sim.getSimulationTime()
        
    time.sleep(0.05)
 
print("Teste finalizado")
sim.stopSimulation()