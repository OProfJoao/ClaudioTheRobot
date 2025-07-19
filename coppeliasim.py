class Components():
    def __init__(self,sim,objName,robotName):
        self.sim = sim
        self.objName = objName
        self.robotName = robotName
        self.handle = self._searchObj()
        
        
    def _searchObj(self):
        handle = self.sim.getObject(f'/{self.robotName}{self.objName}')
        return handle