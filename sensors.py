from coppeliasim import Components
import math
class ProximitySensor(Components):
    def __init__(self, sim, sensorName, robotName):
        super().__init__(sim, sensorName, robotName)
        
    def measureDistance(self):
        result, distance, _, _, _ = self.sim.readProximitySensor(self.handle)
        return [result, distance]
    
class GyroSensor(Components):
    def __init__(self, sim, sensorName, robotName):
        super().__init__(sim, sensorName, robotName)
        
    def measureGyro(self):
        linearVelocity, angularVelocity = self.sim.getObjectVelocity(self.handle)
        return [linearVelocity, angularVelocity]
    
    def angleDegree(self, dt):
        _, angularVelocity = self.measureGyro()
        angleRadians = angularVelocity[2] * dt
        return math.degrees(angleRadians)