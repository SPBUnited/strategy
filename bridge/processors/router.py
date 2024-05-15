"""
Модуль маршрутизации роботов
Получает от стратега требуемые координаты для каждого робота и
считает оптимальный маршрут для достижения этой точки
"""

import math
import typing
from typing import Optional

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.field as field
import bridge.processors.quickhull as qh
import bridge.processors.route as route
import bridge.processors.waypoint as wp


class Router:
    """
    Маршрутизатор
    """

    def __init__(self, fld: field.Field) -> None:
        """
        Конструктор
        """
        self.routes = [route.Route(fld.allies[i]) for i in range(const.TEAM_ROBOTS_MAX_COUNT)]

    def update(self, fld: field.Field) -> None:
        """
        Обновить маршруты актуальным состоянием поля
        """
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            self.routes[i].update(fld.allies[i])

    def set_dest(self, idx: int, target: wp.Waypoint, fld: field.Field) -> None:
        """
        Установить единственную путевую точку для робота с индексом idx
        """
        if idx != const.GK and target.type != wp.WType.R_IGNORE_GOAl_HULL:
            # self_pos = fld.allies[idx].get_pos()
            dest_pos = target.pos
            for goal in [fld.ally_goal, fld.enemy_goal]:
                if aux.is_point_inside_poly(dest_pos, goal.big_hull):
                    closest_out = aux.nearest_point_on_poly(dest_pos, goal.big_hull)
                    angle0 = target.angle
                    self.routes[idx].set_dest_wp(
                        wp.Waypoint(closest_out, angle0, wp.WType.R_PASSTHROUGH)
                    )
                    return
        self.routes[idx].set_dest_wp(target)

    def reroute(self, fld: field.Field) -> None:
        """
        Рассчитать маршруты по актуальным путевым точкам
        """
        for idx in range(const.TEAM_ROBOTS_MAX_COUNT):

            self_pos = fld.allies[idx].get_pos()

            if not self.routes[idx].is_used():
                continue

            if self.routes[idx].get_next_type() == wp.WType.S_VELOCITY:
                continue

            if (
                self.routes[idx].get_next_type() == wp.WType.S_BALL_KICK
                or self.routes[idx].get_next_type() == wp.WType.S_BALL_KICK_UP
                or self.routes[idx].get_next_type() == wp.WType.S_BALL_PASS
            ):
                # if not field.allies[idx].is_kick_aligned(self.routes[idx].get_dest_wp()):
                align_wp = self.calc_kick_wp(idx)
                self.routes[idx].insert_wp(align_wp)
            elif self.routes[idx].get_next_type() == wp.WType.S_BALL_GRAB:
                align_wp = self.calc_grab_wp(idx)
                self.routes[idx].insert_wp(align_wp)

            if idx == const.GK or self.routes[idx].get_dest_wp().type == wp.WType.R_IGNORE_GOAl_HULL:
                pth_wp = self.calc_passthrough_wp(fld, idx)
                if pth_wp is not None:
                    self.routes[idx].insert_wp(pth_wp)
                continue

            for goal in [fld.ally_goal, fld.enemy_goal]:
                if aux.is_point_inside_poly(self_pos, goal.big_hull):
                    closest_out = aux.find_nearest_point(self_pos, goal.big_hull, [goal.up, aux.GRAVEYARD_POS, goal.down])
                    angle0 = self.routes[idx].get_dest_wp().angle
                    self.routes[idx].set_dest_wp(
                        wp.Waypoint(goal.center + (closest_out - goal.center) * 1.2, angle0, wp.WType.S_ENDPOINT)
                    )
                    continue
                pint = aux.segment_poly_intersect(self_pos, self.routes[idx].get_next_wp().pos, goal.big_hull)
                if pint is not None:
                    if aux.is_point_inside_poly(self.routes[idx].get_dest_wp().pos, goal.big_hull):
                        angle0 = self.routes[idx].get_dest_wp().angle
                        self.routes[idx].set_dest_wp(wp.Waypoint(pint, angle0, wp.WType.S_ENDPOINT))
                        break
                    convex_hull = qh.shortesthull(self_pos, self_pos + self.routes[idx].get_next_vec(), goal.big_hull)
                    for j in range(len(convex_hull) - 2, 0, -1):
                        self.routes[idx].insert_wp(wp.Waypoint(convex_hull[j], 4, wp.WType.R_PASSTHROUGH))

            pth_wp = self.calc_passthrough_wp(fld, idx)
            if pth_wp is not None:
                is_inside = False
                for goal in [fld.ally_goal, fld.enemy_goal]:
                    if aux.is_point_inside_poly(pth_wp.pos, goal.big_hull):
                        is_inside = True
                        break
                if not is_inside:
                    self.routes[idx].insert_wp(pth_wp)

    def calc_passthrough_wp(self, fld: field.Field, idx: int) -> typing.Optional[wp.Waypoint]:
        pToGo = self.routes[idx].get_next_wp().pos
        nRobots = 3

        startP = PointTree(fld.allies[idx].get_pos().x, fld.allies[idx].get_pos().y)
        endP = PointTree(pToGo.x, pToGo.y)
        allRobots = []
        queue = [startP]
        nMax = float('inf')
        nSteps = 0
        finish = None
        ###print(queue[nSteps].x, queue[nSteps].y)
        tree = []
        pointGoNow = None
        for i in range(nRobots):
            allRobots.append(fld.allies[i])
            allRobots.append(fld.allies[i])
            ###print(allies.robot(i).x, allies.robot(i).y, enemies.robot(i).x, enemies.robot(i).y)
        allRobots.pop(self.rId * 2)
        while nSteps < nMax:
            onWay = []
            for i in range(len(allRobots)):
                if hLine(queue[nSteps], endP, allRobots[i]) < rRobots * 2 and queue[nSteps].myRobot != i:
                    onWay.append(i)
            if(len(onWay) == 0):
                flag = 1
                if endP.father:
                    oldWayL = 0
                    pointMas = endP
                    while pointMas.father != None:
                        oldWayL += auxiliary.dist(pointMas, pointMas.father)
                        pointMas = pointMas.father
                    newWayL = auxiliary.dist(endP, queue[nSteps])
                    pointMas = queue[nSteps]
                    while pointMas.father != None:
                        newWayL += auxiliary.dist(pointMas, pointMas.father)
                        pointMas = pointMas.father
                    if oldWayL < newWayL:
                        flag = 0
                if flag:
                    nMax = min(nSteps + min(5, len(queue) - nSteps - 1), nMax)
                    endP.father = queue[nSteps]
                    #print("start")
                    pointMas = endP
                    finish = True
                    while pointMas.father.father != None:
                        #print(pointMas.x, pointMas.y)
                        finish = False
                        pointMas = pointMas.father
                    pointGoNow = pointMas
            else:
                minNumOnWay = -1
                minDistOnWay = float('inf')
                for i in range(len(onWay)):
                    if auxiliary.dist(allRobots[onWay[i]], queue[nSteps]) < minDistOnWay:
                        minDistOnWay = auxiliary.dist(allRobots[onWay[i]], queue[nSteps])
                        minNumOnWay = i
                mas2 = [onWay[minNumOnWay]]
                i2 = 0
                while i2 < len(mas2):
                    for another in range(len(allRobots)):
                        if auxiliary.dist(allRobots[mas2[i2]], allRobots[another]) < rRobots * 4 and another not in mas2:
                            mas2.append(another)
                    i2 += 1
                angle1 = 0
                dist1 = 0
                angle2 = 0
                dist2 = 0
                gamma = math.atan2(endP.y - queue[nSteps].y, endP.x - queue[nSteps].x)
                for wall in mas2:
                    a = auxiliary.dist(allRobots[wall], queue[nSteps])
                    b = rRobots * 2
                    c = math.sqrt(abs(a ** 2 - b ** 2))
                    alpha = math.atan2(b, c)
                    beta = math.atan2(allRobots[wall].y - queue[nSteps].y, allRobots[wall].x - queue[nSteps].x)
                    tetta = beta + alpha - gamma
                    if abs(tetta) > math.pi:
                        tetta = math.pi*2 * -auxiliary.sgn(tetta) - tetta
                    if tetta > 0:
                        if abs(angle1) < abs(tetta):
                            angle1 = tetta
                            dist1 = c + 100
                    else:
                        if abs(angle2) < abs(tetta):
                            angle2 = tetta
                            dist2 = c + 100
                    tetta = beta - alpha - gamma
                    if abs(tetta) > math.pi:
                        tetta = math.pi*2 * -auxiliary.sgn(tetta) + tetta
                    if tetta > 0:
                        if abs(angle1) < abs(tetta):
                            angle1 = tetta
                            dist1 = c + 100
                    else:
                        if abs(angle2) < abs(tetta):
                            angle2 = tetta
                            dist2 = c + 100
                queue.append(PointTree(queue[nSteps].x + math.cos(angle2 + gamma) * dist2, queue[nSteps].y + math.sin(angle2 + gamma) * dist2, queue[nSteps], onWay[minNumOnWay]))
                queue.append(PointTree(queue[nSteps].x + math.cos(angle1 + gamma) * dist1, queue[nSteps].y + math.sin(angle1 + gamma) * dist1, queue[nSteps], onWay[minNumOnWay]))
            nSteps += 1
        self.go_to_point(pointGoNow, finish)
        self.rotate_to_point(pointGoNow)
        #print(pointGoNow.x, pointGoNow.y)

        return passthrough_wp

    def calc_kick_wp(self, idx: int) -> wp.Waypoint:
        """
        Рассчитать точку для выравнивания по мячу
        """
        target_point = self.routes[idx].get_dest_wp()
        target_pos = target_point.pos
        target_angle = target_point.angle

        align_pos = target_pos - aux.rotate(aux.RIGHT, target_angle) * const.KICK_ALIGN_DIST
        align_angle = target_angle
        align_type = wp.WType.R_BALL_ALIGN
        # align_type = wp.WType.S_ENDPOINT
        align_wp = wp.Waypoint(align_pos, align_angle, align_type)
        return align_wp

    def calc_grab_wp(self, idx: int) -> wp.Waypoint:
        """
        Рассчитать точку для захвата мяча
        """
        target_point = self.routes[idx].get_dest_wp()
        target_pos = target_point.pos
        target_angle = target_point.angle

        align_pos = target_pos - aux.rotate(aux.RIGHT, target_angle) * const.GRAB_ALIGN_DIST
        align_angle = target_angle
        align_type = wp.WType.R_BALL_ALIGN
        # align_type = wp.WType.S_ENDPOINT
        align_wp = wp.Waypoint(align_pos, align_angle, align_type)
        return align_wp

    def get_route(self, idx: int) -> route.Route:
        """
        Получить маршрут для робота с индексом idx
        """
        return self.routes[idx]

def withLine(edge1: aux.Point, edge2: aux.Point, rPoint: aux.Point, maxAngle:float = math.pi / 2) -> bool:
    alpha = math.atan2(edge2.y - edge1.y, edge2.x - edge1.x)
    beta1 = math.atan2(rPoint.y - edge1.y, rPoint.x - edge1.x)
    beta2 = math.atan2(rPoint.x - edge2.x, edge2.y - rPoint.y)
    corner1 = abs(alpha - beta1)
    corner2 = abs(math.pi / 2 - alpha + beta2)
    corner1 = min(corner1, 2 * math.pi - corner1)
    corner2 = min(corner2, 2 * math.pi - corner2)
    # ##print(str(corner1) + " " + str(corner2))
    ###print(corner1, corner2)
    if corner1 > maxAngle or corner2 > maxAngle:
        return False
    return True

def hLine(edge1: aux.Point, edge2: aux.Point, rPoint: aux.Point, onLine: bool = True, maxAngle: float = math.pi / 2) -> float:
    a = aux.dist(edge1, edge2)
    b = aux.dist(edge1, rPoint)
    c = aux.dist(edge2, rPoint)
    p = (a + b + c) / 2
    sTr = math.sqrt(p * (p - a) * (p - b) * (p - c))
    hTr = sTr * 2 / a
    if onLine:
        if not withLine(edge1, edge2, rPoint, maxAngle):
            hTr = float('inf')
    return hTr

class PointTree:
    def __init__(self, x: float, y: float, father: Optional[int] = None, myRobot: Optional[int] = None) -> None:
        self.x = x
        self.y = y
        self.father = father
        self.myRobot = myRobot
