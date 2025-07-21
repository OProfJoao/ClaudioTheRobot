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

    try:
        print("Criando Robo...")
        vacuum = Robo(sim=sim, robotName=robot_name)

        print("Todos os objetos encontrados")
        print("Iniciando teste")
        sim.startSimulation()
        print(os.listdir('./'))
        while True:
            vacuum.normalCleaning()
            time.sleep(0.05)
    
        
        

    except Exception as e:
        print(f"Erro ao obter os handles: {e}")
        sim.stopSimulation()


if __name__ == "__main__":
    print("Iniciando programa...")
    main()
