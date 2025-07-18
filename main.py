import math
from coppeliasim_zmqremoteapi_client import *
import time

client = RemoteAPIClient()
sim = client.require('sim')

sim.stopSimulation()
time.sleep(1)

robot_name = "PioneerP3DX"


#Constants

BACKUP_DISTANCE = 0.3
LAST_TURN = "left"
DEVIATION_LIMIT = 0.01


def backupALitte():
    print("Dando ré")
    dist_total = 0
    motorControl(-0.5,-0.5)
    starTime = sim.getSimulationTime()
    while(abs(dist_total) < BACKUP_DISTANCE):
        linearVelocity, _ = sim.getObjectVelocity(gyro_handle)
        delta = sim.getSimulationTime() - starTime  
        dist_total =+ linearVelocity[0] * delta
        #print(dist_total)
    motorControl(0.0,0.0)
    
def motorControl(left,right):
    sim.setJointTargetVelocity(left_motor_handle,left)
    sim.setJointTargetVelocity(right_motor_handle,right)

def turn180(side):
    print("Virando")
    degree = 0
    backupALitte()
    match side:
        case "right":
            motorControl(0.4,-0.05)
        case "left":
            motorControl(-0.05,0.4)
            
        case _:
            motorControl(0.0,0.0)
            return 0
        
    currTime = lastTime = sim.getSimulationTime()
    while( degree < 180):
        currTime = sim.getSimulationTime()
        dt = currTime - lastTime
        lastTime = currTime
        degree += abs(angleDegree(dt))
        print(f"Virou: {degree:.3f}")
    motorControl(0.0,0.0)
    
    
    
def angleDegree(delta):
    _, angularVelocity = sim.getObjectVelocity(gyro_handle)
    angulo_total_rad = angularVelocity[2] * delta
    return math.degrees(angulo_total_rad)
    
    
    
sim.startSimulation()
try:
    left_motor_handle = sim.getObject(f'/{robot_name}/leftMotor')
    right_motor_handle = sim.getObject(f'/{robot_name}/rightMotor')
    gyro_handle = sim.getObject(f'/{robot_name}')
    
    IRLeft_handle = sim.getObject(f'/{robot_name}/IRLeft')
    IRFront_handle = sim.getObject(f'/{robot_name}/IRFront')
    
    bumperFront_handle = sim.getObject(f'/{robot_name}/bumperFront')
    bumperLeft_handle = sim.getObject(f'/{robot_name}/bumperLeft')
    bumperRight_handle = sim.getObject(f'/{robot_name}/bumperRight')
    
    
    print("Todos os objetos encontrados")
    
except Exception as e:
    print(f"Erro ao obter os handles: {e}")
    sim.stopSimulation()
    exit()
    

    
currTime = lastTime = sim.getSimulationTime()
angleDeviation = 0  # Initialize angleDeviation before the loop
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
    result_bumper_frente, _, _, _, _ = sim.readProximitySensor(bumperFront_handle)

    angleDeviation += degree * dt * 0.5 #K integral
    print(f"integral: {angleDeviation}")
    
    if(result_bumper_frente == 1):
        
        print("Tocou!")
        if LAST_TURN == "right":
            turn180("left")
            LAST_TURN = "left"
        else:
            turn180("right")
            LAST_TURN = "right"
        angleDeviation = 0
        lastTime = sim.getSimulationTime()
        
    time.sleep(0.05)
 
print("Teste finalizado")
sim.stopSimulation()

