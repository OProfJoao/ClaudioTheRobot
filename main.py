from coppeliasim_zmqremoteapi_client import *
import time
from robo import Robo
import os

def main():
   
    client = RemoteAPIClient()
    sim = client.require('sim')
    robot_name = "PioneerP3DX"
    sim.stopSimulation()
    time.sleep(1)
    dataPath = './SensorData/'
    files = os.listdir(dataPath)
    if len(files)>0:
        lastfile = 0
        for file in files:
            currFile = int(file.split('_')[1])
            if currFile > lastfile:
                lastfile = currFile
        filename = f'{dataPath}{file.split('_')[0]}_{lastfile+1}'
        
    else:
        filename = f'{dataPath}SensorData_0'
    print(filename)
    try:
        print("Criando Robo...")
        vacuum = Robo(sim=sim, robotName=robot_name,savePath=filename)

        print("Todos os objetos encontrados")
        print("Iniciando teste")
        sim.startSimulation()
        
        while True:
            vacuum.normalCleaning()
            time.sleep(0.01)
    
        
        

    except Exception as e:
        print(f"Erro ao obter os handles: {e}")
        sim.stopSimulation()


if __name__ == "__main__":
    print("Iniciando programa...")
    main()
