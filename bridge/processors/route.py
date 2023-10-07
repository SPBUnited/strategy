"""
Класс, описывающий текущий маршрут робота
"""

import bridge.processors.const as const
import bridge.processors.waypoint as wp
import bridge.processors.field as field
import bridge.processors.auxiliary as aux
import bridge.processors.robot as robot

class Route:

    def __init__(self, robot: robot.Robot):
        self.__robot = [wp.Waypoint(robot.pos, robot.angle, wp.WType.ROBOT)]
        self.__destination = [wp.Waypoint(const.GRAVEYARD_POS, 0, wp.WType.GRAVEYARD)]
        self.__routewp = []
        # self.__route = [*self.__robot, *self.__routewp, *self.__destination]

    def update(self, robot: robot.Robot):
        self.__robot = [wp.Waypoint(robot.pos, robot.angle, wp.WType.ROBOT)]
        if self.getNextVec().mag() < const.VANISH_DIST and \
            len(self.__routewp) >= 1:
            del self.__routewp[0]

    def clear(self):
        self.__routewp = []

    def __getRoute(self):
        return [*self.__robot, *self.__routewp, *self.__destination]

    def setDestWP(self, dest: wp.Waypoint):
        self.__destination = [dest]
   
    def getDestWP(self):
        return self.__destination[0]

    def getNextWP(self) -> wp.Waypoint:
        return self.__getRoute()[1]

    def getNextSegment(self):
        return self.__getRoute()[0:1]

    def getNextVec(self) -> aux.Point:
        return (self.__getRoute()[1].pos - self.__getRoute()[0].pos)

    def getNextAngle(self) -> float:
        return self.__getRoute()[1].angle

    def getNextType(self) -> wp.WType:
        return self.__getRoute()[1].type
    
    def insertWP(self, wp: wp.Waypoint):
        self.__routewp.insert(0, wp)

    def isUsed(self):
        return self.__destination[0].type != wp.WType.GRAVEYARD

    def getLenght(self):
        dist = 0
        last_wp_pos = self.__robot[0].pos
        for wpt in self.__getRoute():
            dist += (wpt.pos - last_wp_pos).mag()
            last_wp_pos = wpt.pos
        return dist
        
    def __str__(self):
        strin = "ROUTE: "
        for wp in self.__getRoute():
            strin += " -> " + str(wp)
        # for wp in [*self.__robot, *self.__routewp, *self.__destination]:
        #     strin += " -> " + str(wp)
        return strin

    