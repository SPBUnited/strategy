## @package Stratery
# 
# Расчет требуемых положений роботов исходя из ситуации на поле

import bridge.processors.field as field
import bridge.processors.waypoint as wp
import bridge.processors.const as const
import bridge.processors.auxiliary as aux
import math
from enum import Enum

#!v DEBUG ONLY
import time

class States(Enum):
    DEBUG = 0
    DEFENCE = 1

class Strategy:
    def __init__(self) -> None:
        self.state = States.DEFENCE

        #DEFENCE
        self.old_def_helper = -1
        self.old_def = -1
        self.steal_flag = 0


    """
    Рассчитать конечные точки для каждого робота
    """
    def process(self, field: field.Field):
        if self.state == States.DEBUG:
            return self.debug(field)
        elif self.state == States.DEFENCE:
            return self.defence(field)
        
        if self.state != States.DEFENCE:
            self.old_def_helper = -1
            self.old_def = -1
            self.steal_flag = 0

    def debug(self, field: field.Field):
        waypoints = [None]*const.TEAM_ROBOTS_MAX_COUNT
        for i in range(1, 6):
            bbotpos = field.b_team[i].getPos()
            ybotpos = field.y_team[i].getPos()
            pos = aux.point_on_line(bbotpos, aux.Point(const.GOAL_X, 0), 300)
            
            dpos = bbotpos - ybotpos
            angle = math.atan2(dpos.y, dpos.x)

            waypoint = wp.Waypoint(pos, angle, wp.WType.ENDPOINT)
            waypoints[i] = waypoint

        timer = time.time()
        # timer = 0
        dpos = aux.Point(500 + 1500*((timer//6)%2), -1000)
        # print( dpos )
        dangle = 0*math.pi
        waypoints[1] = wp.Waypoint(dpos, dangle, wp.WType.ENDPOINT)

        gk_pos = aux.point_on_line(field.y_goal, field.ball.pos, 800)

        if field.ball.vel.mag() > 1000:
            gk_pos = aux.closest_point_on_line(field.ball.pos, field.ball.vel.unity()*100000, field.y_team[0].pos)

        gk_angle = math.pi
        waypoints[0] = wp.Waypoint(gk_pos, gk_angle, wp.WType.ENDPOINT)
        return waypoints

    def defence(self, field: field.Field):
        rivals = field.y_team
        allies = field.b_team
        dist_between = 200
        waypoints = [None]*const.TEAM_ROBOTS_MAX_COUNT
        for i in range(6):
            waypoint = wp.Waypoint(allies[i].getPos(), allies[i].getAngle(), wp.WType.ENDPOINT)
            waypoints[i] = waypoint
        gk_pos = aux.point_on_line(field.y_goal, field.ball.pos, 800)
        gk_angle = math.pi
        waypoints[0] = wp.Waypoint(gk_pos, gk_angle, wp.WType.ENDPOINT)

        worksRobots = []
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if allies[i].is_used():
                allies[i].dribblerEnable = 0
                allies[i].autoKick = 0
                worksRobots.append(allies[i])
        totalRobots = len(worksRobots)

        used_robots_id = []
        if allies[0].is_used():
            used_robots_id = [0]
        robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), rivals)
        

        def1 = aux.find_nearest_robot(robot_with_ball.getPos(), allies, used_robots_id)

        if def1.rId == self.old_def_helper:
            def1 = allies[self.old_def]
        targetPoint = aux.point_on_line(robot_with_ball.getPos(), aux.Point(const.SIDE * const.GOAL_X, 0), dist_between)
        waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, robot_with_ball.getPos()), wp.WType.ENDPOINT)
        waypoints[def1.rId] = waypoint
        self.old_def = def1.rId


        used_robots_id.append(def1.rId)
        rbs = sorted(rivals, reverse=True, key=lambda x: x.getPos().x)
        rbs_rIds = []
        xAttack = 2500

        for r in rbs:
            if ((const.SIDE * r.getPos().x > (const.SIDE * robot_with_ball.getPos().x - 150)) or (const.SIDE * r.getPos().x > 0)) and r.rId != robot_with_ball.rId and r.is_used():  
                rbs_rIds.append(r.rId)

        for i in range(len(rbs_rIds)):
            defender = aux.find_nearest_robot(rivals[rbs_rIds[i]].getPos(), allies, used_robots_id)
            used_robots_id.append(defender.rId)

            targetPoint = aux.point_on_line(rivals[rbs_rIds[i]].getPos(), field.ball.getPos(), dist_between)
            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), wp.WType.ENDPOINT)
            waypoints[defender.rId] = waypoint

        if totalRobots - len(used_robots_id) > 0:
            def1Helper = aux.find_nearest_robot(robot_with_ball.getPos(), allies, used_robots_id)
            #targetPoint = aux.point_on_line(field.ball.getPos(), robot_with_ball.getPos(), dist_between)
            targetPoint = aux.point_on_line(field.ball.getPos(), robot_with_ball.getPos(), -dist_between + 150)
            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(def1Helper.getPos(), field.ball.getPos()), wp.WType.ENDPOINT)
            used_robots_id.append(def1Helper.rId)
            self.old_def_helper = def1Helper.rId
            if aux.dist(def1Helper.getPos(), targetPoint) < 100 or self.steal_flag:
                self.steal_flag = 1
                def1Helper.dribblerEnable = 1
                def1Helper.speedDribbler = 15
                if aux.dist(def1Helper.getPos(), targetPoint) < 200:
                    def1Helper.kick_up()
                waypoint = wp.Waypoint(field.ball.getPos(), aux.angle_to_point(def1Helper.getPos(), field.ball.getPos()), wp.WType.IGNOREOBSTACLES)   
            waypoints[def1Helper.rId] = waypoint
            if aux.dist(def1Helper.getPos(), targetPoint) < 1500:
                targetPoint = aux.point_on_line(robot_with_ball.getPos(), aux.Point(const.SIDE * const.GOAL_X, 0), dist_between + 300)
                waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, robot_with_ball.getPos()), wp.WType.ENDPOINT)
                waypoints[def1.rId] = waypoint



        if totalRobots - len(used_robots_id) == 1:
             for r in worksRobots:
                i = r.rId
                if i not in used_robots_id:
                    targetPoint = aux.Point(const.SIDE * -xAttack, 1500)
                    waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), wp.WType.ENDPOINT)
                    waypoints[i] = waypoint
                    used_robots_id.append(i)
        elif totalRobots - len(used_robots_id) == 2:
            def2 = aux.find_nearest_robot(aux.Point(const.SIDE * -xAttack, 1500), allies, used_robots_id)
            used_robots_id.append(def2.rId)
            targetPoint = aux.Point(const.SIDE * -xAttack, 1500)
            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), wp.WType.ENDPOINT)
            waypoints[def2.rId] = waypoint


            for r in worksRobots:
                i = r.rId
                if allies[i].rId not in used_robots_id:
                    used_robots_id.append(allies[i].rId)
                    targetPoint = aux.Point(const.SIDE * -xAttack, -1500)
                    waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), wp.WType.ENDPOINT)
                    waypoints[allies[i].rId] = waypoint

        else:
            def2 = aux.find_nearest_robot(aux.Point(const.SIDE * -xAttack, 1500), allies, used_robots_id)
            used_robots_id.append(def2.rId)
            targetPoint = aux.Point(const.SIDE * -xAttack, 1500)
            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), wp.WType.ENDPOINT)
            waypoints[def2.rId] = waypoint
            
            def2 = aux.find_nearest_robot(aux.Point(const.SIDE * -xAttack, -1500), allies, used_robots_id)
            used_robots_id.append(def2.rId)
            targetPoint = aux.Point(const.SIDE * -xAttack, -1500)
            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), wp.WType.ENDPOINT)
            waypoints[def2.rId] = waypoint

            # Some math for wall. Should be remade with gk.
            k = totalRobots - len(used_robots_id)
            c = 0
            for r in worksRobots:
                i = r.rId
                if i not in used_robots_id:
                    if abs(field.ball.getPos().y) > 800:
                        bY =  800 * aux.sign(field.ball.getPos().y)
                        if abs(field.ball.getPos().x) < 3500:
                            bX = 3200
                        else:
                            bX = field.ball.getPos().x                            
                    else:
                        bY = field.ball.getPos().y
                        bX = 3400
                    if k % 2 == 0:
                        if bX == field.ball.getPos().x:
                            used_robots_id.append(allies[i].rId)
                            targetPoint = aux.Point(const.SIDE * bX - 200 * k / 2 + 200 * c, 1200 * aux.sign(field.ball.getPos().y))
                            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(allies[i].getPos(), aux.Point(allies[i].getPos().x, 5000 * aux.sign(field.ball.getPos().y))), wp.WType.ENDPOINT)
                            waypoints[allies[i].rId] = waypoint                       
                        else:
                            used_robots_id.append(allies[i].rId)
                            targetPoint = aux.Point(const.SIDE * bX, bY - 200 * k / 2 + 200 * c)
                            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(allies[i].getPos(), aux.Point(0, allies[i].getPos().y)), wp.WType.ENDPOINT)
                            waypoints[allies[i].rId] = waypoint                          
                    else:
                        if bX == field.ball.getPos().x:
                            used_robots_id.append(allies[i].rId)
                            targetPoint = aux.Point(const.SIDE * bX - 200 * (k - 1) + 200 * c, 1200 * aux.sign(field.ball.getPos().y))
                            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(allies[i].getPos(), aux.Point(allies[i].getPos().x, 5000 * aux.sign(field.ball.getPos().y))), wp.WType.ENDPOINT)
                            waypoints[allies[i].rId] = waypoint          
                        else:
                            used_robots_id.append(allies[i].rId)
                            targetPoint = aux.Point(const.SIDE * bX, bY - 200 * (k - 1) + 200 * c)
                            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(allies[i].getPos(), aux.Point(0, allies[i].getPos().y)), wp.WType.ENDPOINT)
                            waypoints[allies[i].rId] = waypoint  
                    c += 1

        return waypoints
