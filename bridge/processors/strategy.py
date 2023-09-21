## @package Stratery
# 
# Расчет требуемых положений роботов исходя из ситуации на поле

import bridge.processors.field as field
import bridge.processors.waypoint as wp
import bridge.processors.const as const
import bridge.processors.auxiliary as aux
import math
from enum import Enum

class States(Enum):
    DEBUG = 0
    DEFENCE = 1

class Strategy:
    def __init__(self) -> None:
        self.state = States.DEBUG

    """
    Рассчитать конечные точки для каждого робота
    """
    def process(self, field: field.Field):
        if self.state == States.DEBUG:
            return self.debug(field)
        elif self.state == States.DEFENCE:
            return self.defence(field)

    def debug(self, field: field.Field):
        waypoints = [None]*const.TEAM_ROBOTS_MAX_COUNT
        for i in range(1, 6):
            bbotpos = field.b_team[i].getPos()
            ybotpos = field.y_team[i].getPos()
            pos = aux.point_on_line(bbotpos, aux.Point(4500, 0), 300)
            
            dpos = bbotpos - ybotpos
            angle = math.atan2(dpos.y, dpos.x)

            waypoint = wp.Waypoint(pos, angle, wp.WType.ENDPOINT)
            waypoints[i] = waypoint

        gk_pos = aux.point_on_line(field.y_goal, field.ball.pos, 800)

        if field.ball.vel.mag() > 1000:
            gk_pos = aux.closest_point_on_line(field.ball.pos, field.ball.vel.unity()*100000, field.y_team[0].pos)

        gk_angle = math.pi
        waypoints[0] = wp.Waypoint(gk_pos, gk_angle, wp.WType.ENDPOINT)
        return waypoints

    def defence(self, field: field.Field):
        waypoints = [None]*const.TEAM_ROBOTS_MAX_COUNT
        for i in range(6):
            bbotpos = field.b_team[i].getPos()
            ybotpos = field.y_team[i].getPos()
            pos = aux.point_on_line(bbotpos, aux.Point(4500, 0), 300)
            
            dpos = bbotpos - ybotpos
            angle = math.atan2(dpos.y, dpos.x)

            waypoint = wp.Waypoint(pos, angle, wp.WType.ENDPOINT)
            waypoints[i] = waypoint
        return waypoints
