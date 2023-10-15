"""
Модуль маршрутизации роботов
Получает от стратега требуемые координаты для каждого робота и
считает оптимальный маршрут для достижения этой точки
"""

import bridge.processors.const as const
import bridge.processors.waypoint as wp
import bridge.processors.field as field
import bridge.processors.auxiliary as aux
import bridge.processors.route as route
import bridge.processors.quickhull as qh

import math

class Router:
    def __init__(self, field: field.Field) -> None:
        self.routes = [route.Route(field.allies[i]) for i in range(const.TEAM_ROBOTS_MAX_COUNT)]

    def update(self, field: field.Field):
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            self.routes[i].update(field.allies[i])

    """
    Установить единственную путевую точку для робота с индексом idx
    """
    def setDest(self, idx, target: wp.Waypoint, field: field.Field):
        if idx != const.GK:
            self_pos = field.allies[idx].get_pos()
            dest_pos = target.pos
            for goal in [field.ally_goal, field.enemy_goal]:
                if aux.is_point_inside_poly(dest_pos, goal.hull):
                    closest_out = aux.find_nearest_point(dest_pos, goal.hull, [goal.up, aux.GRAVEYARD_POS, goal.down])
                    angle0 = target.angle
                    self.routes[idx].setDestWP(wp.Waypoint(goal.center + (closest_out - goal.center) * 1.2, angle0, wp.WType.S_ENDPOINT))
                    return
        self.routes[idx].setDestWP(target)

    """
    Рассчитать маршруты по актуальным путевым точкам
    """
    def reRoute(self, field: field.Field):
        for idx in range(const.TEAM_ROBOTS_MAX_COUNT):

            self_pos = field.allies[idx].get_pos()

            if not self.routes[idx].isUsed():
                continue

            if self.routes[idx].getNextType() == wp.WType.S_BALL_KICK or \
                self.routes[idx].getNextType() == wp.WType.S_BALL_GRAB:
                # if not field.allies[idx].is_kick_aligned(self.routes[idx].getDestWP()):
                align_wp = self.calcKickWP(idx, field)
                self.routes[idx].insertWP(align_wp)

            if idx == const.GK:
                pth_wp = self.calcVectorField(idx, field)
                if pth_wp is not None:
                    self.routes[idx].insertWP(pth_wp)
                continue


            for goal in [field.ally_goal, field.enemy_goal]:
                if aux.is_point_inside_poly(self_pos, goal.hull):
                    closest_out = aux.find_nearest_point(self_pos, goal.hull, [goal.up, aux.GRAVEYARD_POS, goal.down])
                    angle0 = self.routes[idx].getDestWP().angle
                    self.routes[idx].setDestWP(wp.Waypoint(goal.center + (closest_out - goal.center) * 1.2, angle0, wp.WType.S_ENDPOINT))
                    continue
                pint = aux.segment_poly_intersect(self_pos, self_pos + self.routes[idx].getNextVec(), goal.hull)
                if pint is not None:
                    if aux.is_point_inside_poly(self.routes[idx].getDestWP().pos, goal.hull):
                        angle0 = self.routes[idx].getDestWP().angle
                        self.routes[idx].setDestWP(wp.Waypoint(pint, angle0, wp.WType.S_ENDPOINT))
                        break
                    convex_hull = qh.shortesthull(self_pos, self_pos + self.routes[idx].getNextVec(), goal.hull)
                    for j in range(len(convex_hull) - 2, 0, -1):
                        self.routes[idx].insertWP(wp.Waypoint(
                            convex_hull[j],
                            0,
                            wp.WType.R_PASSTHROUGH
                        ))

            pth_wp = self.calcVectorField(idx, field)
            if pth_wp is not None:
                is_inside = False
                for goal in [field.ally_goal, field.enemy_goal]:
                    if aux.is_point_inside_poly(pth_wp.pos, goal.hull):
                        is_inside = True
                        break
                if not is_inside:
                    self.routes[idx].insertWP(pth_wp)

    # def calcPenDetour(self, idx, field: field.Field):
    #     self_pos = field.allies[idx].get_pos()
    #     detwp = None
    #     for goal in [*field.ally_goal, *field.enemy_goal]:
    #         vec1 = aux.vect_mult(self.routes[idx].getNextVec().unity(), (goal.forwup - self_pos).unity())
    #         vec2 = aux.vect_mult(self.routes[idx].getNextVec().unity(), (goal.forwdown - self_pos).unity())
    #         vecc = aux.vect_mult(self.routes[idx].getNextVec().unity(), (goal.center - self_pos).unity())
    #         if(aux.sign(vec1) != aux.sign(vecc)):
    #             if(aux.sign(vec2) != aux.sign(vecc)):
    #                 detwp =


    def calcVectorField(self, idx, field: field.Field):
        allies = field.allies
        enemy = field.enemies
        self_pos = allies[idx].get_pos()
        target_point = self.routes[idx].getNextWP()
        dist = (self_pos - target_point.pos).mag()

        vector_field_threshold = 200
        if dist < vector_field_threshold:
            return None

        if self.routes[idx].getDestWP().type == wp.WType.S_IGNOREOBSTACLES or \
              self.routes[idx].getDestWP().type == wp.WType.S_BALL_GO:
            return None

        sep_dist = 500
        ball_sep_dist = 150

        if self.routes[idx].getDestWP().type == wp.WType.S_KEEP_BALL_DISTANCE:
            ball_sep_dist = const.KEEP_BALL_DIST

        closest_robot = None
        closest_dist = dist
        closest_separation = 0
        angle_to_point = math.atan2(target_point.pos.y - self_pos.y, target_point.pos.x - self_pos.x)

        # Расчет теней роботов для векторного поля
        for r in field.all_bots:
            if r == self or r.is_used() == 0:
                continue
            robot_separation = aux.dist(aux.closest_point_on_line(self_pos, target_point.pos, r.get_pos()), r.get_pos())
            robot_dist = aux.dist(self_pos, r.get_pos())
            if robot_dist == 0:
                continue
            if robot_separation < sep_dist and robot_dist < closest_dist:
                closest_robot = r
                closest_dist = robot_dist
                closest_separation = robot_separation

        ball_separation = aux.dist(aux.closest_point_on_line(self_pos, target_point.pos, field.ball.get_pos()), field.ball.get_pos())
        ball_dist = aux.dist(self_pos, field.ball.get_pos())
        if ball_separation < ball_sep_dist and ball_dist < closest_dist:
            closest_robot = field.ball
            closest_dist = ball_dist
            closest_separation = ball_separation

        passthrough_wp_pos = target_point.pos - self_pos
        # angle_cos = aux.scal_mult((closest_robot.get_pos() - self_pos).unity(), (target_point.pos - self_pos).unity())
        if closest_robot is not None and closest_dist != 0:
            side = aux.vect_mult(closest_robot.get_pos() - self_pos, target_point.pos - self_pos)
            # offset_angle_val = 200*math.pi/6 * closest_dist**(-2)
            offset_angle_val = -2 * math.atan((sep_dist - closest_separation)/(2*(closest_dist)))
            offset_angle = offset_angle_val if side < 0 else -offset_angle_val
            passthrough_wp_pos = self_pos + aux.rotate(target_point.pos - self_pos, offset_angle)

            passthrough_wp = wp.Waypoint(
                passthrough_wp_pos,
                0,
                wp.WType.R_PASSTHROUGH
            )
        else:
            passthrough_wp = None

        return passthrough_wp

    def calcKickWP(self, idx, field: field.Field):
        target_point = self.routes[idx].getDestWP()
        target_pos = target_point.pos
        target_angle = target_point.angle

        align_pos = target_pos - aux.rotate(aux.RIGHT, target_angle) * const.KICK_ALIGN_DIST
        align_angle = target_angle
        align_type = wp.WType.R_BALL_ALIGN
        align_wp = wp.Waypoint(
            align_pos,
            align_angle,
            align_type
        )
        return align_wp

    """
    Получить маршрут для робота с индексом idx
    """
    def getRoute(self, idx):
        return self.routes[idx]
