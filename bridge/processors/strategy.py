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
        for i in range(6):
            bbotpos = field.b_team[i].getPos()
            ybotpos = field.y_team[i].getPos()
            pos = aux.point_on_line(bbotpos, aux.Point(4500, 0), 300)
            
            dpos = bbotpos - ybotpos
            angle = math.atan2(dpos.y, dpos.x)

            waypoint = wp.Waypoint(aux.Point(0, 0), angle, wp.WType.ENDPOINT)
            waypoints[i] = waypoint
        return waypoints

    def defence(self, field: field.Field):
        enemy = field.b_team
        waypoints = [None]*const.TEAM_ROBOTS_MAX_COUNT
        for i in range(6):
            bbotpos = field.b_team[i].getPos()
            ybotpos = field.y_team[i].getPos()
            pos = aux.point_on_line(bbotpos, aux.Point(4500, 0), 300)
            
            dpos = bbotpos - ybotpos
            angle = math.atan2(dpos.y, dpos.x)

            waypoint = wp.Waypoint(aux.Point(100, 100), angle, wp.WType.ENDPOINT)
            waypoints[i] = waypoint


        used_robots_id = []
        robot_with_ball = aux.find_nearest_robot(field.ball, enemy)

        def1 = aux.find_nearest_robot(robot_with_ball, enemy, used_robots_id)

        ybotpos = def1.getPos()
        bbotpos = robot_with_ball.getPos()
        
        dpos = bbotpos - ybotpos
        angle = math.atan2(dpos.y, dpos.x)

        waypoint = wp.Waypoint(aux.point_on_line(robot_with_ball.getPos(), aux.Point(const.SIDE * 4500, 0), 300), angle + math.pi, wp.WType.ENDPOINT)
        waypoints[def1.rId] = waypoint

        used_robots_id.append(def1.rId)
        rbs = sorted(enemy, reverse=True, key=lambda x: x.getPos().x)
        rbs_rIds = []
        for r in rbs:
            if ((const.SIDE * r.getPos().x > (const.SIDE * robot_with_ball.getPos().x - 150)) or (const.SIDE * r.getPos().x > 0)) and r.rId != robot_with_ball.rId and r.isUsed:  
                rbs_rIds.append(r.rId)

        '''for i in range(len(rbs_rIds)):
            def2 = self.find_nearest_robot(enemy.robot(rbs_rIds[i]), self, used_robots_id)
            used_robots_id.append(def2.rId)
            self.robot(def2.rId).go_to_point_with_detour(aux.point_on_line(enemy.robot(rbs_rIds[i]) , ball, 300), enemy, self)
            self.robot(def2.rId).rotate_to_point(enemy.robot(rbs_rIds[i]))


        self.robot(def1.rId).go_to_point_with_detour(aux.point_on_line(robot_with_ball, aux.Point(const.SIDE * 4500, 0), 300), enemy, self)
        self.robot(def1.rId).rotate_to_point(robot_with_ball)
        if len(self.used_robots()) - len(used_robots_id) == 1:
            for i in range(len(self.used_robots())):
                if self.robot(i).rId not in used_robots_id:
                    self.robot(i).go_to_point_with_detour(aux.Point(const.SIDE * 4000, 2000), enemy, self)
                    self.robot(i).rotate_to_point(ball)
        elif len(self.used_robots()) - len(used_robots_id) == 2:
            def2 = self.find_nearest_robot(aux.Point(const.SIDE * -4000, 1500), self, used_robots_id)
            used_robots_id.append(def2.rId)
            self.robot(def2.rId).go_to_point_with_detour(aux.Point(const.SIDE * -4000, 1500), enemy, self)
            self.robot(def2.rId).rotate_to_point(ball)

            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if self.robot(i).rId not in used_robots_id:
                    self.robot(i).go_to_point_with_detour(aux.Point(const.SIDE * -4000, -1500), enemy, self)
                    self.robot(i).rotate_to_point(ball)
        else:
            def2 = self.find_nearest_robot(aux.Point(const.SIDE * -4000, 1500), self, used_robots_id)
            used_robots_id.append(def2.rId)
            self.robot(def2.rId).go_to_point_with_detour(aux.Point(const.SIDE * -4000, 1500), enemy, self)
            self.robot(def2.rId).rotate_to_point(ball)
            
            def2 = self.find_nearest_robot(aux.Point(const.SIDE * -4000, -1500), self, used_robots_id)
            used_robots_id.append(def2.rId)
            self.robot(def2.rId).go_to_point_with_detour(aux.Point(const.SIDE * -4000, -1500), enemy, self)
            self.robot(def2.rId).rotate_to_point(ball)

            k = len(self.used_robots()) - len(used_robots_id)
            c = 0
            for i in range(len(self.used_robots())):
                if self.robot(i).rId not in used_robots_id:
                    if abs(ball.y) > 800:
                        bY =  800 * ball.y / abs(ball.y)
                        print(ball.x)
                        if abs(ball.x) < 3500:
                            bX = 3200
                        else:
                            bX = ball.x
                            
                    else:
                        bY = ball.y
                        bX = 3400
                    if k % 2 == 0:
                        if bX == ball.x:
                            self.robot(i).go_to_point(aux.Point(const.SIDE * bX - 200 * k / 2 + 200 * c, 1200 * ball.y / abs(ball.y)))
                            self.robot(i).rotate_to_point(aux.Point(self.robot(i).x, 5000 * ball.y / abs(ball.y)))
                            
                        else:
                            self.robot(i).go_to_point(aux.Point(const.SIDE * bX, bY - 200 * k / 2 + 200 * c))
                            self.robot(i).rotate_to_point(aux.Point(0, self.robot(i).y))
                            
                    else:
                        if bX == ball.x:
                            self.robot(i).go_to_point(aux.Point(const.SIDE * bX - 200 * (k - 1) + 200 * c, 1200 * ball.y / abs(ball.y)))
                            self.robot(i).rotate_to_point(aux.Point(self.robot(i).x, 5000 * ball.y / abs(ball.y)))
                            
                        else:
                            self.robot(i).go_to_point(aux.Point(const.SIDE * bX, bY - 200 * (k - 1) + 200 * c))
                            self.robot(i).rotate_to_point(aux.Point(0, self.robot(i).y))
                    c += 1'''

        return waypoints
