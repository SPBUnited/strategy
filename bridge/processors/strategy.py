## @package Stratery
# 
# Расчет требуемых положений роботов исходя из ситуации на поле

import bridge.processors.field as field
import bridge.processors.waypoint as wp
import bridge.processors.const as const
import bridge.processors.auxiliary as aux
import math

class Strategy:
    def __init__(self) -> None:
        pass

    """
    Рассчитать конечные точки для каждого робота
    """
    def process(self, field: field.Field):
        waypoints = [None]*const.TEAM_ROBOTS_MAX_COUNT
        for i in range(1, 6):
            bbotpos = field.b_team[i].getPos()
            ybotpos = field.y_team[i].getPos()
            pos = aux.point_on_line(bbotpos, aux.Point(4500, 0), 300)
            
            dpos = bbotpos - ybotpos
            angle = math.atan2(dpos.y, dpos.x)

            waypoint = wp.Waypoint(pos, angle, wp.WType.ENDPOINT)
            waypoints[i] = waypoint
        return waypoints