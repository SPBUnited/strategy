import bridge.processors.const as const
import bridge.processors.auxiliary as auxiliary

class Team:
    def __init__(self, gk_id):
        self._robots = []
        self.gk_id = gk_id

    def play(self, enemy, ball):
        pass

    def gk(self, enemy):
        pass
        
    def add_robot(self, robot):
        self._robots.append(robot)

    def robots_amount(self):
        return len(self._robots)
    
    def robot(self, id):
        return self._robots[id]
    
    def robots(self):
        return self._robots.copy()
